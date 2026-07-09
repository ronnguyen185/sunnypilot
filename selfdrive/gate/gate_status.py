"""GATE_STATUS (0x500) decode helpers (wire-identical to gate-panda gate-soc2/soc2/gate_status.py)."""

from __future__ import annotations

import struct

from openpilot.selfdrive.gate.gate_crc import crc16_ccitt_false

ID_GATE_STATUS = 0x500

# Firmware health_level_t: HEALTH_GREEN=0, HEALTH_YELLOW=1, HEALTH_RED=2
HEALTH_NAMES = {
  0: "GREEN",
  1: "YELLOW",
  2: "RED",
}

GATE_STATE_NAMES = {
  0: "PRIMARY_ACTIVE",
  1: "SECONDARY_ACTIVE",
  2: "MRM_ACTIVE",
  3: "ISOLATED",
}

SOURCE_NAMES = {
  0: "NONE",
  1: "PRIMARY",
  2: "SECONDARY",
  3: "INTERNAL_MRM",
}


def decode_gate_status(data: bytes) -> dict:
  """Decode GATE_STATUS 0x500 (16 B, ages big-endian per gate_build_gate_status)."""
  if len(data) < 16:
    return {"raw": data.hex(), "note": "frame too short"}

  expected = struct.unpack_from("<H", data, 14)[0]
  if crc16_ccitt_false(bytes(data[:14])) != expected:
    return {"raw": data.hex(), "note": "CRC mismatch"}

  gs, src, pri, sec = data[0], data[1], data[2], data[3]
  return {
    "gate_state": GATE_STATE_NAMES.get(gs, f"unknown({gs})"),
    "active_source": SOURCE_NAMES.get(src, f"unknown({src})"),
    "pri_health": HEALTH_NAMES.get(pri, f"unknown({pri})"),
    "sec_health": HEALTH_NAMES.get(sec, f"unknown({sec})"),
    "gate_state_raw": gs,
    "active_source_raw": src,
    "pri_health_raw": pri,
    "sec_health_raw": sec,
    "pri_hb_age_ms": struct.unpack_from(">H", data, 4)[0],
    "sec_hb_age_ms": struct.unpack_from(">H", data, 6)[0],
    "pri_cmd_age_ms": data[8],
    "sec_cmd_age_ms": data[9],
    "switch_reason": data[10],
    "safety_flags": data[11],
    "uptime_s": data[12] | (data[13] << 8),
    "raw": data.hex(),
  }


def format_gate_status(info: dict) -> str:
  """One-line summary matching gate-panda verify_gate.py / emulate_soc2_c3x --listen."""
  if "note" in info:
    return f"GATE_STATUS invalid: {info['note']} raw={info.get('raw', '')}"
  return (
    f"state={info['gate_state']} source={info['active_source']} "
    f"pri={info['pri_health']} sec={info['sec_health']} "
    f"sec_hb_age={info['sec_hb_age_ms']}ms sec_cmd_age={info['sec_cmd_age_ms']}ms "
    f"uptime={info['uptime_s']}s"
  )
