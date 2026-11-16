#!/usr/bin/env python3
"""VF6/VF7 InfoCAN camera-object interface (FCAM ``Camera_Object01``–``06``).

These platforms have no MRR radar bus; FCAM publishes fused object tracks on
InfoCAN (logical bus 4) for the MHU. We parse them into ``RadarData.RadarPoint``
for ``radard.py`` vision fusion.
"""
import logging
import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.vinfast.radar_interface import (
  MAX_AREL,
  MAX_DX,
  MAX_VREL,
  MIN_DX,
  MSTATUS_MOVING,
  MSTATUS_UNKNOWN,
  ORIENT_PRECEEDING,
  ORIENT_UNKNOWN,
  _radar_point_supports_motion_fields,
)
from opendbc.car.vinfast.values import CANBUS, DBC, is_vf6_safety_platform

log = logging.getLogger("info_radar_interface")

INFO_RADAR_HZ = 10
INVALID_OBJ_ID = 255
INVALID_CLASS = 7
INVALID_LANE = 7
INVALID_LGT_DIS = 399.575  # DBC invalid raw 0x3FFF

INFO_LANE_HOST = 2

# Position-only FCAM: LgtDis quantizes to 2.5 cm; use a lighter deadband than MRR CAN.
INFO_VREL_DEADBAND = 0.12  # [m/s]
# Alpha-beta-gamma gains for ~10 Hz distance updates (tuned for 0.1 s nominal dt).
INFO_KIN_ALPHA = 0.72
INFO_KIN_BETA = 0.22
INFO_KIN_GAMMA = 0.045

# Warmup / ramp for position-only FCAM (see docs/vinfast-debug/VF6_HIGHWAY_INFOCAN_REPLAY.md)
KIN_WARMUP_KIN_UPDATES = 4
KIN_AREL_RAMP_UPDATES = 10
KIN_AREL_RAMP_MAX = 2.0   # [m/s²] cap |aRel| during ramp
KIN_VREL_RAMP_MAX = 2.0   # [m/s] max closing speed magnitude during ramp
# Sustained output cap after ramp (mature tracks — replay capture hit ±10 aRel here)
INFO_SUSTAINED_MAX_AREL = 2.0  # [m/s²]
# Limit single-frame vRel jumps from LgtDis stick-slip (quantized position)
INFO_MAX_V_STEP = 2.5  # [m/s] per ~100 ms update after warmup

INFO_TRIGGER_ADDR = 851
INFO_OBJECT_ADDRS = {833, 835, 842, 844, 845, 846}

OBJECT_SLOTS = (
  ("Camera_Object01", "01"),
  ("Camera_Object02", "02"),
  ("Camera_Object03", "03"),
  ("Camera_Object04", "04"),
  ("Camera_Object05", "05"),
  ("Camera_Object06", "06"),
)

FOCUS_CLASSES = {1, 2, 3, 4, 5, 6}
MIN_VALID_CNT = 1
MAX_HOST_LAT = 2.3  # [m] allow undefined-lane objects near path centre

# InfoCAN lane enum → MRR-style laneAssignment for radard.py filters
INFO_TO_RADAR_LANE = {
  0: 0,
  1: 2,  # left lane
  2: 3,  # host lane
  3: 4,  # right lane
}


def get_info_radar_can_parser(CP):
  if not is_vf6_safety_platform(CP.carFingerprint):
    return None
  if Bus.body not in DBC[CP.carFingerprint]:
    return None

  dbc_name = DBC[CP.carFingerprint][Bus.body]
  messages = [("Camera_Object01", INFO_RADAR_HZ)]
  for msg, _ in OBJECT_SLOTS[1:]:
    messages.append((msg, math.nan))  # parse when present; don't fault if absent
  messages.append(("ADAS_CMP_ACC", 20))
  bus = CANBUS.info_bus(CP.carFingerprint)
  try:
    parser = CANParser(dbc_name, messages, bus)
    log.info("Created VF6/VF7 InfoCAN radar parser on bus %d (%d object msgs)", bus, len(OBJECT_SLOTS))
    return parser
  except Exception as e:
    log.error("InfoCAN radar parser failed: %s", e, exc_info=True)
    return None


def _parse_slot(msg: dict, idx: str) -> dict:
  prefix = f"ADAS_OBJ{idx}_"
  return {
    "id": msg.get(f"{prefix}ID"),
    "class": msg.get(f"{prefix}Class"),
    "lgt": msg.get(f"{prefix}LgtDis"),
    "lat": msg.get(f"{prefix}LatDis"),
    "lane": msg.get(f"{prefix}Lane"),
  }


def _object_valid(obj: dict) -> bool:
  obj_id = obj["id"]
  if obj_id is None or int(obj_id) in (0, INVALID_OBJ_ID):
    return False

  obj_class = obj["class"]
  if obj_class is None or int(obj_class) == INVALID_CLASS:
    return False

  lane = obj["lane"]
  if lane is not None and int(lane) == INVALID_LANE:
    return False

  lgt = obj["lgt"]
  if lgt is None or lgt >= (INVALID_LGT_DIS - 0.1) or lgt < MIN_DX or lgt > MAX_DX:
    return False

  lat = float(obj["lat"] or 0.0)
  lane_int = int(lane) if lane is not None else 0
  if lane_int == INFO_LANE_HOST:
    return True
  if lane_int == 0 and abs(lat) <= MAX_HOST_LAT:
    return True
  return False


class _DistanceRateFilter:
  """Estimate vRel/aRel from position-only FCAM distance (no CAN Rel_Vx)."""

  __slots__ = ("d", "v", "a", "t_nanos", "initialized", "kin_updates")

  def __init__(self):
    self.d: float | None = None
    self.v = 0.0
    self.a = 0.0
    self.t_nanos = 0
    self.initialized = False
    self.kin_updates = 0

  def _apply_output_caps(self, v: float, a: float) -> tuple[float, float]:
    if self.kin_updates <= KIN_WARMUP_KIN_UPDATES:
      return 0.0, 0.0
    ramp_i = self.kin_updates - KIN_WARMUP_KIN_UPDATES
    if ramp_i <= KIN_AREL_RAMP_UPDATES:
      a = max(-KIN_AREL_RAMP_MAX, min(KIN_AREL_RAMP_MAX, a))
      if v < -KIN_VREL_RAMP_MAX:
        v = -KIN_VREL_RAMP_MAX
    else:
      a = max(-INFO_SUSTAINED_MAX_AREL, min(INFO_SUSTAINED_MAX_AREL, a))
    return v, a

  def update(self, d_meas: float, t_nanos: int) -> tuple[float, float]:
    if not self.initialized or self.d is None or t_nanos <= self.t_nanos:
      self.d = d_meas
      self.t_nanos = t_nanos
      self.initialized = True
      return 0.0, 0.0

    dt = (t_nanos - self.t_nanos) / 1e9
    if dt <= 0.001 or dt > 1.0:
      self.d = d_meas
      self.t_nanos = t_nanos
      return self.v, self.a

    d_pred = self.d + self.v * dt + 0.5 * self.a * dt * dt
    v_pred = self.v + self.a * dt
    residual = d_meas - d_pred

    self.d = d_pred + INFO_KIN_ALPHA * residual
    v_new = v_pred + (INFO_KIN_BETA / dt) * residual
    if self.kin_updates > KIN_WARMUP_KIN_UPDATES:
      dv = max(-INFO_MAX_V_STEP, min(INFO_MAX_V_STEP, v_new - v_pred))
      self.v = v_pred + dv
    else:
      self.v = v_new
    self.a = self.a + (2.0 * INFO_KIN_GAMMA / (dt * dt)) * residual

    if abs(self.v) < INFO_VREL_DEADBAND:
      self.v = 0.0
    else:
      self.v = max(-MAX_VREL, min(MAX_VREL, self.v))
    self.a = max(-MAX_AREL, min(MAX_AREL, self.a))

    self.kin_updates += 1
    self.v, self.a = self._apply_output_caps(self.v, self.a)

    self.t_nanos = t_nanos
    return self.v, self.a


class InfoRadarInterface:
  """Parse FCAM camera objects on InfoCAN for VF6 / VF7 Plus."""

  def __init__(self, CP, CP_SP=None):
    self.CP = CP
    self.rcp = get_info_radar_can_parser(CP)
    self.trigger_msg = INFO_TRIGGER_ADDR
    self.updated_messages: set[int] = set()
    self.track_id = 0
    self.pts: dict[int, structs.RadarData.RadarPoint] = {}
    self.obj_id_to_track: dict[int, int] = {}
    self.valid_cnt: dict[int, int] = {}
    self.kin_filters: dict[int, _DistanceRateFilter] = {}
    self.acc_obj_detected = False

  def update(self, can_packets):
    if self.rcp is None:
      return None

    if not can_packets:
      return None

    vls = self.rcp.update(can_packets)
    self.updated_messages.update(vls)

    has_objects = bool(self.updated_messages & INFO_OBJECT_ADDRS)
    if self.trigger_msg not in self.updated_messages and not has_objects:
      return None

    try:
      current_time_nanos = can_packets[-1][0] if can_packets else 0
    except (IndexError, TypeError):
      current_time_nanos = 0

    ret = self._update(current_time_nanos)
    self.updated_messages.clear()
    return ret

  def _update(self, current_time_nanos: int) -> structs.RadarData:
    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    if "ADAS_CMP_ACC" in self.rcp.vl:
      self.acc_obj_detected = bool(self.rcp.vl["ADAS_CMP_ACC"].get("ADAS_ACC_ObjDetec", 0))

    seen_obj_ids: set[int] = set()

    for msg_name, idx in OBJECT_SLOTS:
      if msg_name not in self.rcp.vl:
        continue

      obj = _parse_slot(self.rcp.vl[msg_name], idx)
      if not _object_valid(obj):
        continue

      obj_id = int(obj["id"])
      dx = float(obj["lgt"])
      lat = float(obj["lat"] or 0.0)
      obj_class = int(obj["class"])
      lane_int = int(obj["lane"]) if obj["lane"] is not None else 0
      # openpilot yRel: positive = right; InfoCAN LatDis positive = left
      yrel = -lat

      is_focus = obj_class in FOCUS_CLASSES
      if obj_id not in self.valid_cnt:
        self.valid_cnt[obj_id] = 0
      self.valid_cnt[obj_id] += 1
      if self.valid_cnt[obj_id] < MIN_VALID_CNT:
        continue

      if obj_id not in self.obj_id_to_track:
        track_id = self.track_id
        self.track_id += 1
        self.obj_id_to_track[obj_id] = track_id
      else:
        track_id = self.obj_id_to_track[obj_id]

      if track_id not in self.pts:
        self.pts[track_id] = structs.RadarData.RadarPoint()
        self.pts[track_id].trackId = track_id

      # No Rel_Vx on InfoCAN — smooth distance rate for vRel/aRel (TTC, radard KF).
      if track_id not in self.kin_filters:
        self.kin_filters[track_id] = _DistanceRateFilter()
      vrel, a_rel = self.kin_filters[track_id].update(dx, current_time_nanos)

      pt = self.pts[track_id]
      pt.dRel = dx
      pt.yRel = yrel
      pt.vRel = float(vrel)
      pt.aRel = float(a_rel)
      pt.yvRel = 0.0
      pt.measured = False  # velocity is estimated, not from CAN

      if _radar_point_supports_motion_fields():
        pt.motionStatus = MSTATUS_MOVING if is_focus else MSTATUS_UNKNOWN
        pt.motionOrientation = ORIENT_PRECEEDING if lane_int == INFO_LANE_HOST else ORIENT_UNKNOWN
        pt.laneAssignment = INFO_TO_RADAR_LANE.get(lane_int, 0)

      seen_obj_ids.add(obj_id)

    # Drop tracks that disappeared
    stale_track_ids = []
    for oid, tid in list(self.obj_id_to_track.items()):
      if oid not in seen_obj_ids:
        stale_track_ids.append(tid)
        del self.obj_id_to_track[oid]
        self.valid_cnt.pop(oid, None)

    for tid in stale_track_ids:
      self.pts.pop(tid, None)
      self.kin_filters.pop(tid, None)

    ret.points = list(self.pts.values())
    return ret
