"""CAR_STATE (0x501) decode helpers — Gate → SOC chassis feedback.

Wire-identical to gate-panda gate-soc2/soc2/car_state.py.

  byte 0-3:  v_ego_mps            sint32 LE, 0.001 m/s
  byte 4-7:  steering_angle_deg   sint32 LE, 0.001 deg
  byte 8:    standstill
  byte 9:    brake_pressed   # pedal >0.5% OR ESC_Braking_effort (OEM 0xF1 / 0x20D)
  byte 10:   abs_active
  byte 11:   esc_active
  byte 12-13: steering_torque_nm  sint16 LE, 0.01 Nm
  byte 14-15: crc16 LE over [0..13]
"""

from __future__ import annotations

import struct

from openpilot.selfdrive.gate.gate_crc import crc16_ccitt_false

ID_CAR_STATE = 0x501
CAR_STATE_DLC = 16
CAR_STATE_STALE_S = 0.1
TARGET_RESOLUTION = 0.001
TORQUE_RESOLUTION = 0.01


def _quantize_001(value: float) -> int:
  scaled = value / TARGET_RESOLUTION
  return int(scaled + 0.5) if scaled >= 0 else int(scaled - 0.5)


def _quantize_torque(value: float) -> int:
  scaled = value / TORQUE_RESOLUTION
  return int(scaled + 0.5) if scaled >= 0 else int(scaled - 0.5)


def decode_car_state(data: bytes) -> dict | None:
  """Decode CAR_STATE 0x501 (16 B). Returns None if length/CRC invalid."""
  if len(data) != CAR_STATE_DLC:
    return None
  if crc16_ccitt_false(data[:14]) != struct.unpack_from("<H", data, 14)[0]:
    return None

  v_ego = struct.unpack_from("<i", data, 0)[0] * TARGET_RESOLUTION
  steer = struct.unpack_from("<i", data, 4)[0] * TARGET_RESOLUTION
  return {
    "v_ego_mps": v_ego,
    "vehicle_speed_kph": v_ego * 3.6,
    "steering_angle_deg": steer,
    "steering_torque_nm": struct.unpack_from("<h", data, 12)[0] * TORQUE_RESOLUTION,
    "standstill": bool(data[8]),
    "brake_pressed": bool(data[9]),
    "abs_active": bool(data[10]),
    "esc_active": bool(data[11]),
  }


def encode_car_state(
  *,
  v_ego_mps: float | None = None,
  vehicle_speed_kph: float | None = None,
  steering_angle_deg: float = 0.0,
  steering_torque_nm: float = 0.0,
  standstill: bool = False,
  brake_pressed: bool = False,
  abs_active: bool = False,
  esc_active: bool = False,
) -> bytes:
  """Build a valid CAR_STATE frame (for tests / bench inject)."""
  if v_ego_mps is None:
    v_ego_mps = 0.0 if vehicle_speed_kph is None else vehicle_speed_kph / 3.6
  p = bytearray(CAR_STATE_DLC)
  struct.pack_into("<i", p, 0, _quantize_001(v_ego_mps))
  struct.pack_into("<i", p, 4, _quantize_001(steering_angle_deg))
  p[8] = 1 if standstill else 0
  p[9] = 1 if brake_pressed else 0
  p[10] = 1 if abs_active else 0
  p[11] = 1 if esc_active else 0
  struct.pack_into("<h", p, 12, _quantize_torque(steering_torque_nm))
  crc = crc16_ccitt_false(bytes(p[:14]))
  struct.pack_into("<H", p, 14, crc)
  return bytes(p)


def format_car_state(info: dict | None) -> str:
  if info is None:
    return "CAR_STATE invalid"
  return (
    f"v_ego={info['v_ego_mps']:.3f} m/s ({info['vehicle_speed_kph']:.2f} kph) "
    f"steer={info['steering_angle_deg']:+.3f} deg "
    f"tq={info['steering_torque_nm']:+.2f} Nm "
    f"standstill={int(info['standstill'])} brake={int(info['brake_pressed'])}"
  )
