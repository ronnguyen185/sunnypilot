#!/usr/bin/env python3
"""VinFast radar interface for MRR + SCAM CAN messages (0x410–0x419).

Parses 10 CAN frames × 2 objects = 20 radar slots.
Applies noise filtering (existence probability, validity counters, uncertainty
gates) inspired by Hyundai / Toyota / Ford radar interfaces before emitting
``RadarData.RadarPoint`` objects consumed by ``radard_new.py``.
"""
import math
import logging

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.vinfast.values import is_vf6_safety_platform

log = logging.getLogger("radar_interface_new")

_radar_motion_fields_supported: bool | None = None
_radar_schema_warned = False


def _radar_point_supports_motion_fields() -> bool:
    global _radar_motion_fields_supported, _radar_schema_warned
    if _radar_motion_fields_supported is not None:
        return _radar_motion_fields_supported
    try:
        _p = structs.RadarData.RadarPoint()
        _p.motionStatus = 0
        _p.motionOrientation = 0
        _p.laneAssignment = 0
        _radar_motion_fields_supported = True
    except Exception:
        _radar_motion_fields_supported = False
        if not _radar_schema_warned:
            _radar_schema_warned = True
            log.warning(
                "RadarPoint has no motionStatus/motionOrientation/laneAssignment in loaded car.capnp; "
                "radar metadata omitted."
            )
    return _radar_motion_fields_supported


# ── CAN layout ───────────────────────────────────────────────────────────────
RADAR_START_ADDR = 0x410
RADAR_MSG_COUNT = 10
RADAR_END_ADDR = RADAR_START_ADDR + RADAR_MSG_COUNT - 1  # 0x419
RADAR_FREQ_HZ = 18  # typical radar frame rate

# ── Object type classification ────────────────────────────────────────────────
# Focus types receive lenient filtering (lower ExistProb threshold, fewer
# persistence frames) because false-negatives are more dangerous than
# false-positives for cars/trucks/pedestrians in the ego lane.
FOCUS_TYPES = {1, 2, 3, 4, 5}  # Car, Truck, Motorcycle, Pedestrian, Cyclist

# ── Filtering thresholds (tune per radar hardware) ────────────────────────────
MIN_DX = 1.5             # [m]  minimum reportable distance (DBC offset noise guard)
MAX_DX = 300.0           # [m]  maximum reportable distance
MIN_EXIST_PROB_FOCUS = 0.7  # lower for known real targets to reduce false negatives
MIN_EXIST_PROB_OTHER = 0.8  # stricter for non-focus / unknown to limit false positives
MIN_VALID_CNT_FOCUS = 2      # faster admission for known focus types
MIN_VALID_CNT_OTHER = 5      # persistence frames for non-focus / unidentified types
MAX_DX_STD = 15.0        # [m]  reject non-focus if DxStdDev exceeds this
MAX_VX_STD = 8.0         # [m/s]  reject non-focus if Rel_VxStdDev exceeds this
MAX_VY_STD = 4.0          # [m/s]  reject non-focus if Rel_VyStdDev exceeds this
MAX_AZIMUTH_DEG = 50.0    # [°]  reject non-focus if azimuth exceeds this
# Coarse sanity gates for all tracks (focus + non-focus)
MAX_DX_STD_ANY = 25.0     # [m]
MAX_VX_STD_ANY = 12.0     # [m/s]
MAX_VREL = 100.0          # [m/s]  clip estimated vRel to this (≈ 360 km/h)
MAX_AREL = 10.0           # [m/s²]  clip aRel estimate
VREL_DEADBAND = 0.5       # [m/s]  small vRel treated as static

# ── Motion_Orientation values (from DBC VAL_ definitions) ─────────────────────
# "The direction of the target's heading compared to the host."
ORIENT_INVALID        = 0
ORIENT_DRIFTING_RIGHT = 1   # same direction, drifting right
ORIENT_CROSSING_RIGHT = 3   # crossing from left to right
ORIENT_OC_DRIFT_RIGHT = 5   # oncoming, drifting right
ORIENT_ONCOMING       = 6   # head-on, opposite direction
ORIENT_OC_DRIFT_LEFT  = 7   # oncoming, drifting left
ORIENT_CROSSING_LEFT  = 9   # crossing from right to left
ORIENT_DRIFTING_LEFT  = 11  # same direction, drifting left
ORIENT_PRECEEDING     = 12  # same direction ahead (preceding)
ORIENT_UNKNOWN        = 13

# Orientations indicating same-direction travel (preceding / drifting alongside)
SAME_DIR_ORIENTATIONS = {ORIENT_PRECEEDING, ORIENT_DRIFTING_LEFT, ORIENT_DRIFTING_RIGHT}

# Orientations indicating crossing traffic
CROSSING_ORIENTATIONS = {ORIENT_CROSSING_LEFT, ORIENT_CROSSING_RIGHT}

# ── Lane_Assignment values (from DBC VAL_ definitions) ────────────────────────
# "The object's lane relative to the host lane."
LANE_UNKNOWN     = 0
LANE_LEFT_LEFT   = 1
LANE_LEFT        = 2
LANE_HOST        = 3   # our lane
LANE_RIGHT       = 4
LANE_RIGHT_RIGHT = 5

# Adjacent / non-host lanes — objects here that are same-direction are passing
ADJACENT_LANES = {LANE_LEFT_LEFT, LANE_LEFT, LANE_RIGHT, LANE_RIGHT_RIGHT}

# ── Motion_Status values (from DBC VAL_ definitions) ─────────────────────────
MSTATUS_INVALID       = 0
MSTATUS_UNKNOWN       = 1
MSTATUS_MOVING        = 2
MSTATUS_STATIONARY    = 3
MSTATUS_STOPPED       = 4
MSTATUS_MOVING_SLOWLY = 5

# Two-wheeler types (Motorcycle, Cyclist) — high false-positive rate for braking
# when overtaking from behind
TWO_WHEELER_TYPES = {3, 5}  # Motorcycle, Cyclist
PEDESTRIAN_TYPE = 4
PEDESTRIAN_SIDE_AZIMUTH_DEG = 8.0  # [deg] side-offset ped reject threshold

def _extract_signals(msg, prefix):
    """Extract all DBC signals for a given object prefix into a clean dict.

    DBC signal names look like 'obj01_Dx_oA_CAMERA_OD1_OD2_oMRR_'.
    This strips the prefix and the '_oA_...' suffix to produce {'Dx': value, ...}.
    """
    result = {}
    plen = len(prefix)
    for k, v in msg.items():
        if k.startswith(prefix):
            field = k[plen:]
            idx = field.find('_oA_')
            if idx > 0:
                field = field[:idx]
            result[field] = v
    return result


def get_radar_can_parser(CP):
    """Create ``CANParser`` for VinFast radar messages (0x410–0x419).

    Returns:
      CANParser instance or None on failure.
    """
    dbc_name = "vinfast_vf8_mrr_scam"

    messages = [(addr, RADAR_FREQ_HZ) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]

    radar_bus = getattr(CP, 'radarBus', 1) if hasattr(CP, 'radarBus') else 1

    try:
        parser = CANParser(dbc_name, messages, radar_bus)
        log.info("Created VinFast radar CAN parser: %d msgs on bus %d  addrs=%s",
                 len(messages), radar_bus, [hex(a) for a in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)])
        return parser
    except Exception as e:
        log.error("Failed to create VinFast radar CAN parser: %s", e, exc_info=True)
        return None


class RadarInterface(RadarInterfaceBase):
    """VinFast MRR+SCAM radar interface.

    Lifecycle: ``__init__`` → ``update(can_packets)`` every CAN frame.
    Outputs ``structs.RadarData`` with a list of ``RadarPoint``s.
    """

    def __init__(self, CP, CP_SP=None):
        super().__init__(CP, CP_SP)
        self.updated_messages: set[int] = set()
        self.trigger_msg = RADAR_END_ADDR
        self.track_id = 0

        self._info_radar = None
        if is_vf6_safety_platform(CP.carFingerprint):
            # No radar DBC on bus 1 yet. Keep InfoCAN camera objects on bus 5.
            from opendbc.car.vinfast.info_radar_interface import InfoRadarInterface
            self._info_radar = InfoRadarInterface(CP, CP_SP)
            self.rcp = self._info_radar.rcp
            self.radar_off_can = self.rcp is None
        else:
            self.radar_off_can = CP.radarUnavailable
            self.rcp = get_radar_can_parser(CP)

        self.obj_id_to_track: dict[int, int] = {}   # obj_id → trackId
        self.valid_cnt: dict[int, int] = {}           # obj_id → persistence counter

        # Previous frame data per track_id for Δv/Δt estimation.
        # value: (vRel, dRel, timestamp_nanos, from_can)
        #   from_can=True if vRel came from CAN Rel_Vx (trusted);
        #   from_can=False if vRel was estimated from Δd/Δt (use for aRel but not
        #   as seed for next Δd/Δt to prevent feedback loops).
        self.prev_data: dict[int, tuple] = {}

    def update(self, can_packets):
        """Update radar interface with CAN packets.

        Args:
            can_packets: list of (timestamp_nanos, list[frames]) tuples
            frames is a list of (address, dat, src) tuples from can_capnp_to_list
        """
        if self._info_radar is not None:
            return self._info_radar.update(can_packets)

        if self.radar_off_can or (self.rcp is None):
            return super().update(None)

        # Convert can_packets to format expected by CANParser.update
        # can_packets is already in the format: list of (timestamp, list of (address, dat, src) tuples)
        # CANParser.update expects the same format
        can_strings = can_packets

        if not can_strings:
            return None

        vls = self.rcp.update(can_strings)
        self.updated_messages.update(vls)

        # Wait for trigger message (0x419) before processing
        # But also process if we have any radar messages (0x410-0x419) to be more lenient
        has_radar_msgs = any(addr in self.updated_messages for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT))
        if self.trigger_msg not in self.updated_messages and not has_radar_msgs:
            return None

        # If we have radar messages but not the trigger, process anyway (more lenient)
        # This allows processing even if 0x419 is missing

        # Get timestamp from first CAN packet for acceleration calculation
        # Safety: ensure can_strings is not empty and first element has expected structure
        if can_strings and len(can_strings) > 0:
            try:
                current_time_nanos = can_strings[0][0] if isinstance(can_strings[0], (list, tuple)) and len(can_strings[0]) > 0 else 0
            except (IndexError, TypeError):
                current_time_nanos = 0
        else:
            current_time_nanos = 0

        rr = self._update(self.updated_messages, current_time_nanos)
        self.updated_messages.clear()

        return rr

    def _update(self, updated_messages, current_time_nanos=0):
        """Process updated CAN messages and create radar tracks."""
        ret = structs.RadarData()
        if self.rcp is None:
            return ret

        if not self.rcp.can_valid:
            ret.errors.canError = True

        # Track which obj_ids we've seen in this update
        seen_obj_ids = set()

        # Process each radar message (0x410-0x419)
        # Check which addresses are actually available in the parser
        for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
            # Check if address is in parser's vl (value list)
            if addr not in self.rcp.vl:
                continue

            msg = self.rcp.vl[addr]

            # Each message has two objects (obj01, obj02, etc.)
            base = (addr - RADAR_START_ADDR) * 2 + 1
            prefixes = [f"obj{base:02d}_", f"obj{base+1:02d}_"]

            for pref in prefixes:
                # Extract all signals for this object into a clean dict
                sig = _extract_signals(msg, pref)

                obj_id = sig.get("ID")
                dx = sig.get("Dx")
                vrel = sig.get("Rel_Vx")
                exist_prob = sig.get("ExistProb", 1.0)
                motion_status = sig.get("Motion_Status")
                obj_type = sig.get("Type", 0)
                dx_std = sig.get("DxStdDev")
                vx_std = sig.get("Rel_VxStdDev")
                phi_left = sig.get("PhiLeft")
                phi_right = sig.get("PhiRight")
                vy_rel = sig.get("Rel_Vy")
                vy_std = sig.get("Rel_VyStdDev")
                motion_orient = sig.get("Motion_Orientation")
                lane_assign = sig.get("Lane_Assignment")

                # ── Basic validation ───────────────────────────────────────
                if dx is None or dx < MIN_DX or dx > MAX_DX:
                    continue
                if obj_id is None:
                    continue

                obj_type_int = int(obj_type) if obj_type is not None else 0
                is_focus_type = obj_type_int in FOCUS_TYPES

                # Unknown type (Type=0): keep only plausible center/closing targets.
                # This allows unstable bike/motorbike classifications while still
                # rejecting most phantom side clutter.
                is_unknown_type = (obj_type_int == 0)

                # ── Lateral position from azimuth ─────────────────────────
                # openpilot: yRel positive = RIGHT;  ISO azimuth: positive = LEFT
                yrel = 0.0
                azimuth = None
                if phi_left is not None and phi_right is not None:
                    azimuth = (phi_left + phi_right) / 2.0
                elif phi_left is not None:
                    azimuth = phi_left
                elif phi_right is not None:
                    azimuth = phi_right

                if azimuth is not None:
                    yrel = -(dx * math.sin(azimuth))

                motion_status_int = int(motion_status) if motion_status is not None else MSTATUS_INVALID
                motion_orient_int = int(motion_orient) if motion_orient is not None else ORIENT_INVALID
                lane_assign_int = int(lane_assign) if lane_assign is not None else LANE_UNKNOWN

                # ── Noise filtering (ExistProb / Motion_Status / distance) ─
                is_valid = True

                # Dynamic easing for likely real targets: close + moving focus
                # objects should pass slightly faster than static/unknown clutter.
                is_moving_obj = motion_status_int in (MSTATUS_MOVING, MSTATUS_MOVING_SLOWLY)
                is_close_obj = dx < 35.0
                can_vrel = float(vrel) if vrel is not None else 0.0
                unknown_centered = abs(yrel) < max(0.7, 0.03 * dx)
                unknown_closing = can_vrel < -0.2

                if is_unknown_type:
                    # Keep unknown tracks only if they look lead-like.
                    # Tight enough to avoid ghost side objects.
                    if not (is_moving_obj and unknown_centered and unknown_closing and dx < 45.0):
                        is_valid = False

                if exist_prob is not None:
                    min_prob = MIN_EXIST_PROB_FOCUS if is_focus_type else MIN_EXIST_PROB_OTHER
                    if is_unknown_type:
                        min_prob = max(min_prob, 0.85)
                    # Keep easing mild so phantom tracks don't pass too easily.
                    if (is_focus_type and is_moving_obj and is_close_obj and
                            abs(yrel) < 1.8 and (vx_std is None or vx_std <= 6.0)):
                        min_prob = max(0.55, min_prob - 0.03)
                    if exist_prob < min_prob:
                        is_valid = False

                if motion_status is not None and motion_status == 0:
                    is_valid = False

                # Global coarse sanity checks (all object types)
                if dx_std is not None and dx_std > MAX_DX_STD_ANY:
                    is_valid = False
                if vx_std is not None and vx_std > MAX_VX_STD_ANY:
                    is_valid = False

                # Unidentified objects are common phantom sources; require motion
                # evidence if laterally offset, otherwise drop early.
                if obj_type_int == 0 and abs(yrel) > 0.6 and not is_moving_obj:
                    is_valid = False

                # ── Persistence counter (Toyota / Ford style) ─────────────
                if obj_id not in self.valid_cnt:
                    self.valid_cnt[obj_id] = 0

                if is_valid:
                    self.valid_cnt[obj_id] += 1
                else:
                    self.valid_cnt[obj_id] = max(self.valid_cnt[obj_id] - 1, 0)

                min_valid = MIN_VALID_CNT_FOCUS if is_focus_type else MIN_VALID_CNT_OTHER
                if is_unknown_type:
                    min_valid = max(min_valid, 6)
                if (is_focus_type and is_moving_obj and is_close_obj and
                        (exist_prob is None or exist_prob >= 0.70)):
                    min_valid = max(1, min_valid - 1)

                # Require stronger persistence for moving preceding pedestrians.
                # This suppresses short-lived phantom lead snaps.
                motion_orient_int_p = motion_orient_int
                lane_assign_int_p = lane_assign_int
                ped_preceding = (obj_type_int == PEDESTRIAN_TYPE and
                                 motion_orient_int_p == ORIENT_PRECEEDING and
                                 motion_status_int in (MSTATUS_MOVING, MSTATUS_MOVING_SLOWLY))
                if ped_preceding:
                    ped_side_like = (lane_assign_int_p in ADJACENT_LANES or
                                     abs(yrel) > max(0.3, 0.045 * dx) or
                                     (vrel is not None and float(vrel) < -0.3))
                    min_valid = max(min_valid, 8 if ped_side_like else 6)

                if self.valid_cnt[obj_id] < min_valid:
                    continue

                # ── Extra gates for non-focus types ───────────────────────
                if not is_focus_type:
                    if dx_std is not None and dx_std > MAX_DX_STD:
                        continue
                    if vx_std is not None and vx_std > MAX_VX_STD:
                        continue
                    if vy_std is not None and vy_std > MAX_VY_STD:
                        continue
                    if azimuth is not None:
                        max_lat = 3.0 if dx < 20.0 else 10.0
                        if abs(yrel) > max_lat:
                            continue
                        if abs(azimuth) > math.radians(MAX_AZIMUTH_DEG):
                            continue

                # ── Overtaking two-wheeler filter (DBC-based) ─────────────
                # A Motorcycle/Cyclist travelling same direction is NOT a
                # braking target when it's clearly passing on the side.
                #
                # Two cases handled (Vietnam: narrow streets, no lane
                # discipline — bikes squeeze past within the same lane):
                #
                # Case A — Adjacent lane: DBC Lane_Assignment is LEFT /
                #   RIGHT / etc.  High confidence, no extra checks needed.
                #
                # Case B — HOST lane but laterally offset & not closing:
                #   The bike is squeezing past within our lane on a narrow
                #   street.  Extra safety: require |yRel| > 0.8 m AND
                #   CAN Rel_Vx > -0.5 m/s (bike is matching speed or
                #   faster, not something we're catching up to).
                #
                # If DBC signals are INVALID/UNKNOWN, radard.py Criterion 9
                # (vRel + yRel physics check) acts as a fallback.
                is_two_wheeler = obj_type_int in TWO_WHEELER_TYPES
                is_same_dir = motion_orient_int in SAME_DIR_ORIENTATIONS
                is_actively_moving = motion_status_int in (MSTATUS_MOVING, MSTATUS_MOVING_SLOWLY)

                if is_two_wheeler and is_same_dir and is_actively_moving:
                    # Case A: adjacent lane — confident overtake
                    if lane_assign_int in ADJACENT_LANES:
                        log.debug("Filtered overtaking two-wheeler (adj lane): obj_id=%s type=%d orient=%d lane=%d dx=%.1f yrel=%.1f",
                                  obj_id, obj_type_int, motion_orient_int, lane_assign_int, dx, yrel)
                        continue

                    # Case B: HOST lane — bike squeezing past on narrow street
                    # In Vietnam traffic, motorbikes overtake within the same
                    # lane constantly.  The radar may classify them as either
                    # DRIFTING or PRECEDING — both happen for a passing bike.
                    #
                    # Filter when laterally offset AND not closing on us:
                    #   Drifting:  |yRel| > 0.8 m, vRel > -0.5 (lenient)
                    #   Preceding: |yRel| > 1.0 m, vRel > -1.0 (slightly stricter
                    #              to protect genuine in-lane leads that wobble)
                    can_vrel = float(vrel) if vrel is not None else None
                    is_drifting = motion_orient_int in (ORIENT_DRIFTING_LEFT, ORIENT_DRIFTING_RIGHT)
                    is_preceding = motion_orient_int == ORIENT_PRECEEDING

                    if lane_assign_int == LANE_HOST and can_vrel is not None:
                        if is_drifting and abs(yrel) > 1.2 and can_vrel > -0.1 and dx < 20.0:
                            log.debug("Filtered drifting two-wheeler (HOST lane): "
                                      "obj_id=%s type=%d orient=%d yrel=%.1f vrel=%.1f dx=%.1f",
                                      obj_id, obj_type_int, motion_orient_int, yrel, can_vrel, dx)
                            continue
                        if is_preceding and abs(yrel) > 1.5 and can_vrel > -0.2 and dx < 20.0:
                            log.debug("Filtered preceding two-wheeler (HOST lane, offset): "
                                      "obj_id=%s type=%d orient=%d yrel=%.1f vrel=%.1f dx=%.1f",
                                      obj_id, obj_type_int, motion_orient_int, yrel, can_vrel, dx)
                            continue

                # ── Crossing traffic filter (DBC-based) ───────────────────
                # Objects with CROSSING orientation that are laterally offset
                # are passing through our path — not a sustained lead.
                # Only filter when lateral offset confirms they're not in
                # our lane (yrel > 1.0m).  Keeps objects near centre that
                # might be crossing INTO our lane.
                if (motion_orient_int in CROSSING_ORIENTATIONS and
                        abs(yrel) > 1.3 and
                        lane_assign_int in ADJACENT_LANES and
                        dx > 18.0 and
                        (vrel is None or float(vrel) > -1.0)):
                    log.debug("Filtered crossing traffic: obj_id=%s orient=%d lane=%d dx=%.1f yrel=%.1f",
                              obj_id, motion_orient_int, lane_assign_int, dx, yrel)
                    continue

                # ── Slow preceding pedestrian side-offset filter ─────────────
                # Similar to parked-side filtering: keep only path-centre peds as
                # braking leads. If ped is slower than ego (vRel < 0), apply a
                # tighter lateral gate because these are commonly side passers.
                ped_vrel = float(vrel) if vrel is not None else None
                ped_slower_than_ego = ped_vrel is not None and ped_vrel < -0.3
                ped_side_by_azimuth = (azimuth is not None and
                                       abs(azimuth) > math.radians(PEDESTRIAN_SIDE_AZIMUTH_DEG))
                ped_side_y_thresh = max(0.3, 0.045 * dx) if ped_slower_than_ego else max(0.5, 0.07 * dx)

                if (obj_type_int == PEDESTRIAN_TYPE and
                        motion_orient_int == ORIENT_PRECEEDING and
                        motion_status_int in (MSTATUS_MOVING_SLOWLY, MSTATUS_MOVING)):
                    # Adjacent lane + preceding ped -> always side object
                    if lane_assign_int in ADJACENT_LANES:
                        log.debug("Filtered preceding pedestrian in adjacent lane: "
                                  "obj_id=%s lane=%d dx=%.1f yrel=%.1f vrel=%.1f",
                                  obj_id, lane_assign_int, dx, yrel, ped_vrel if ped_vrel is not None else 0.0)
                        continue
                    # Host/unknown lane but side-offset and azimuth/lower-speed evidence
                    if abs(yrel) > ped_side_y_thresh and (ped_side_by_azimuth or ped_slower_than_ego):
                        log.debug("Filtered slow preceding pedestrian offset: "
                                  "obj_id=%s status=%d orient=%d lane=%d dx=%.1f yrel=%.1f az=%.2f vrel=%.1f",
                                  obj_id, motion_status_int, motion_orient_int, lane_assign_int, dx, yrel,
                                  float(azimuth) if azimuth is not None else 0.0,
                                  ped_vrel if ped_vrel is not None else 0.0)
                        continue

                # ── Stationary / stopped roadside obstacle filter (DBC) ───────
                # Objects classified STATIONARY or STOPPED by radar hardware
                # that are laterally offset are parked cars, guardrails, signs,
                # road furniture, etc.  The lateral planner can navigate around
                # them — don't emit as radar points to avoid phantom braking.
                #
                # Keep dead-centre objects (|yRel| ≤ threshold) — those are
                # genuine obstacles in our lane (stopped car ahead, barrier).
                #
                # Also use Lane_Assignment: objects that radar places in an
                # adjacent lane AND are stationary are definitely not our concern.
                if motion_status_int in (MSTATUS_STATIONARY, MSTATUS_STOPPED):
                    # Adjacent lane + stationary → always filter (high confidence)
                    if lane_assign_int in ADJACENT_LANES:
                        log.debug("Filtered stationary obj in adjacent lane: "
                                  "obj_id=%s type=%d status=%d lane=%d dx=%.1f yrel=%.1f",
                                  obj_id, obj_type_int, motion_status_int, lane_assign_int, dx, yrel)
                        continue

                    # Host/unknown lane + stationary + laterally offset → parked roadside
                    # Threshold tuned for Vietnam: cars pass within ~1.0 m of parked objects
                    stationary_lat_threshold = 0.8   # [m] keep objects within 0.8 m of centre
                    if abs(yrel) > stationary_lat_threshold:
                        log.debug("Filtered stationary roadside object: "
                                  "obj_id=%s type=%d status=%d dx=%.1f yrel=%.1f",
                                  obj_id, obj_type_int, motion_status_int, dx, yrel)
                        continue

                # ── Unknown motion status in adjacent lane → likely parked/roadside
                # Radar may classify parked cars as UNKNOWN instead of STATIONARY.
                # If the object is also in an adjacent lane and offset, it's roadside.
                if motion_status_int == MSTATUS_UNKNOWN and lane_assign_int in ADJACENT_LANES:
                    if abs(yrel) > 1.2:
                        log.debug("Filtered unknown-status obj in adjacent lane: "
                                  "obj_id=%s type=%d lane=%d dx=%.1f yrel=%.1f",
                                  obj_id, obj_type_int, lane_assign_int, dx, yrel)
                        continue

                # ── Far lateral object filter ──────────────────────────────────
                # Objects > 2.5 m from lane centre are definitely not in our lane
                # regardless of type/status — parked cars, guardrails, road signs.
                if abs(yrel) > 2.5:
                    log.debug("Filtered far lateral object: obj_id=%s type=%d dx=%.1f yrel=%.1f",
                              obj_id, obj_type_int, dx, yrel)
                    continue

                # Create or update track
                # Use obj_id as the key to maintain track consistency
                if obj_id not in self.obj_id_to_track:
                    # New track
                    track_id = self.track_id
                    self.track_id += 1
                    self.obj_id_to_track[obj_id] = track_id

                    if track_id not in self.pts:
                        self.pts[track_id] = structs.RadarData.RadarPoint()
                        self.pts[track_id].trackId = track_id
                else:
                    # Existing track
                    track_id = self.obj_id_to_track[obj_id]
                    if track_id not in self.pts:
                        self.pts[track_id] = structs.RadarData.RadarPoint()
                        self.pts[track_id].trackId = track_id

                # ── Velocity ───────────────────────────────────────────────
                # Prefer CAN Rel_Vx; fall back to Δd/Δt if missing.
                vrel_from_can = vrel is not None
                if vrel_from_can:
                    vrel = float(vrel)
                    if abs(vrel) < VREL_DEADBAND:
                        vrel = 0.0   # static-object calibration noise
                else:
                    # Estimate from distance change — but ONLY if previous
                    # frame had a CAN-based vRel (prevents feedback loops
                    # where estimated vRel feeds back into estimated aRel).
                    prev = self.prev_data.get(track_id)
                    if prev is not None and prev[3]:   # prev.from_can
                        prev_dRel, prev_time = prev[1], prev[2]
                        if current_time_nanos > prev_time > 0 and prev_dRel is not None:
                            dt = (current_time_nanos - prev_time) / 1e9
                            if 0.001 < dt < 1.0:
                                vrel = -(dx - prev_dRel) / dt
                                if abs(vrel) > MAX_VREL:
                                    log.debug("Clipped estimated vRel %.1f → 0  obj_id=%s", vrel, obj_id)
                                    vrel = 0.0
                            else:
                                vrel = 0.0
                        else:
                            vrel = 0.0
                    else:
                        vrel = 0.0

                # ── Acceleration (Δv / Δt) ────────────────────────────────
                aRel = float('nan')
                prev = self.prev_data.get(track_id)
                if prev is not None:
                    prev_vRel, _, prev_time, _ = prev
                    if (current_time_nanos > prev_time > 0
                            and prev_vRel is not None and not math.isnan(prev_vRel)):
                        dt = (current_time_nanos - prev_time) / 1e9
                        if 0.001 < dt < 1.0:
                            a = (vrel - prev_vRel) / dt
                            if abs(a) <= MAX_AREL:
                                aRel = a
                            # else: leave nan — downstream Kalman ignores nan

                # Store for next frame: (vRel, dRel, time, from_can)
                self.prev_data[track_id] = (vrel, dx, current_time_nanos, vrel_from_can)

                # ── Emit point (sanitise nan / inf before writing) ───────
                pt = self.pts[track_id]
                pt.dRel = float(dx)
                pt.yRel = float(yrel)
                pt.vRel = float(vrel) if math.isfinite(vrel) else 0.0
                pt.aRel = float(aRel) if math.isfinite(aRel) else 0.0
                # Negate vy_rel: DBC Rel_Vy uses ISO convention (positive=LEFT),
                # openpilot uses positive=RIGHT (same as yRel negation on line 265).
                pt.yvRel = float(-vy_rel) if (vy_rel is not None and math.isfinite(vy_rel)) else 0.0
                pt.measured = True

                pt.motionStatus = int(motion_status) if motion_status is not None else MSTATUS_INVALID
                pt.motionOrientation = int(motion_orient) if motion_orient is not None else ORIENT_INVALID
                pt.laneAssignment = int(lane_assign) if lane_assign is not None else LANE_UNKNOWN

                seen_obj_ids.add(obj_id)

        # ── Remove stale tracks ───────────────────────────────────────
        stale_track_ids = []
        for oid, tid in list(self.obj_id_to_track.items()):
            if oid not in seen_obj_ids:
                stale_track_ids.append(tid)
                del self.obj_id_to_track[oid]
                self.valid_cnt.pop(oid, None)

        for tid in stale_track_ids:
            self.pts.pop(tid, None)
            self.prev_data.pop(tid, None)

        ret.points = list(self.pts.values())
        return ret

