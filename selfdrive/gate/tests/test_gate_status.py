"""GATE_STATUS decode tests (parity with gate-panda gate-soc2/soc2/test_gate_status.py)."""

from openpilot.selfdrive.gate.gate_crc import crc16_ccitt_false
from openpilot.selfdrive.gate.gate_status import decode_gate_status, format_gate_status


def _status_frame(gs: int, src: int, pri: int, sec: int) -> bytes:
  p = bytearray(16)
  p[0], p[1], p[2], p[3] = gs, src, pri, sec
  crc = crc16_ccitt_false(bytes(p[:14]))
  p[14] = crc & 0xFF
  p[15] = (crc >> 8) & 0xFF
  return bytes(p)


class TestDecodeGateStatus:
  def test_short_frame(self):
    info = decode_gate_status(b"\x01\x02")
    assert "note" in info

  def test_green_sec_health(self):
    info = decode_gate_status(_status_frame(1, 2, 2, 0))
    assert info["gate_state"] == "SECONDARY_ACTIVE"
    assert info["pri_health"] == "RED"
    assert info["sec_health"] == "GREEN"

  def test_unknown_health_level(self):
    info = decode_gate_status(_status_frame(0, 0, 0, 9))
    assert info["sec_health"] == "unknown(9)"

  def test_format_line(self):
    info = decode_gate_status(_status_frame(1, 2, 2, 0))
    line = format_gate_status(info)
    assert "SECONDARY_ACTIVE" in line
    assert "sec=GREEN" in line
