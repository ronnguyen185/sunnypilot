"""Gate Protocol v2 frame encoder (wire-identical to soc1/src/gate_encode.c)."""

from __future__ import annotations

import math
import struct

from openpilot.selfdrive.gate.gate_crc import crc16_ccitt_false

# Physical → raw: all active units use 0.001 / LSB (degree, Nm, m/s^2, m/s).
TARGET_RESOLUTION = 0.001

# CAN IDs
ID_SOURCE_HEALTH = 0x400
ID_CMD_LAT = 0x410
ID_CMD_LON = 0x411

# Request types (CarControl / soc1 enums)
LAT_REQ_NONE = 0
LAT_REQ_ANGLE = 1
LAT_REQ_TORQUE = 2

LON_REQ_NONE = 0
LON_REQ_ACCEL = 1
LON_REQ_SPEED = 2

# Target units
LAT_UNIT_NONE = 0
LAT_UNIT_DEGREE = 1
LAT_UNIT_NEWTON_METER = 2

LON_UNIT_NONE = 0
LON_UNIT_MPS2 = 1
LON_UNIT_MPS = 2

CMD_LEN = 12


def _round_i32(physical: float, resolution: float = TARGET_RESOLUTION) -> int:
  x = physical / resolution
  return int(math.floor(x + 0.5)) if x >= 0 else int(math.ceil(x - 0.5))


def encode_steer_deg(steer_deg: float) -> int:
  """Quantize degrees at 0.001 deg/LSB."""
  return _round_i32(steer_deg)


def encode_accel_mps2(accel_mps2: float) -> int:
  """Quantize m/s^2 at 0.001 m/s^2/LSB."""
  return _round_i32(accel_mps2)


def build_source_health(counter: int, available: bool) -> bytes:
  frame = bytearray(4)
  frame[0] = counter & 0xFF
  frame[1] = 0x01 if available else 0x00
  crc = crc16_ccitt_false(bytes(frame[:2]))
  struct.pack_into("<H", frame, 2, crc)
  return bytes(frame)


def build_cmd_lat(
  counter: int,
  active: bool,
  req_type: int,
  target_unit: int,
  target: int,
) -> bytes:
  """Build CMD_LAT (12 B). Inactive must use type/unit/target = 0."""
  frame = bytearray(CMD_LEN)
  frame[0] = counter & 0xFF
  frame[1] = 0x01 if active else 0x00
  frame[2] = req_type & 0xFF
  frame[3] = target_unit & 0xFF
  struct.pack_into("<i", frame, 4, int(target))
  frame[8] = 0
  frame[9] = 0
  crc = crc16_ccitt_false(bytes(frame[:10]))
  struct.pack_into("<H", frame, 10, crc)
  return bytes(frame)


def build_cmd_lon(
  counter: int,
  active: bool,
  req_type: int,
  target_unit: int,
  target: int,
) -> bytes:
  """Build CMD_LON (12 B). Inactive must use type/unit/target = 0."""
  frame = bytearray(CMD_LEN)
  frame[0] = counter & 0xFF
  frame[1] = 0x01 if active else 0x00
  frame[2] = req_type & 0xFF
  frame[3] = target_unit & 0xFF
  struct.pack_into("<i", frame, 4, int(target))
  frame[8] = 0
  frame[9] = 0
  crc = crc16_ccitt_false(bytes(frame[:10]))
  struct.pack_into("<H", frame, 10, crc)
  return bytes(frame)
