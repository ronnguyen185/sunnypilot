"""CAR_STATE decode/encode tests (parity with gate-panda gate-soc2/soc2/test_car_state.py)."""

from openpilot.selfdrive.gate.car_state import decode_car_state, encode_car_state, format_car_state
from openpilot.selfdrive.gate.gate_crc import crc16_ccitt_false


class TestDecodeCarState:
  def test_bad_length(self):
    assert decode_car_state(b"\x00" * 8) is None

  def test_bad_crc(self):
    frame = encode_car_state(v_ego_mps=10.0)
    bad = bytearray(frame)
    bad[14] ^= 0xFF
    assert decode_car_state(bytes(bad)) is None

  def test_example_10mps_neg_steer(self):
    frame = encode_car_state(v_ego_mps=10.0, steering_angle_deg=-12.5)
    assert frame[0:4] == (10000).to_bytes(4, "little", signed=True)
    assert frame[4:8] == (-12500).to_bytes(4, "little", signed=True)
    info = decode_car_state(frame)
    assert info is not None
    assert abs(info["v_ego_mps"] - 10.0) < 1e-6
    assert abs(info["steering_angle_deg"] - (-12.5)) < 1e-6

  def test_flags_and_format(self):
    frame = encode_car_state(v_ego_mps=0.0, standstill=True, brake_pressed=True)
    info = decode_car_state(frame)
    assert info is not None
    assert info["standstill"] is True
    line = format_car_state(info)
    assert "standstill=1" in line

  def test_crc_matches_ccitt_false(self):
    frame = encode_car_state(v_ego_mps=1.23, steering_angle_deg=4.56)
    assert crc16_ccitt_false(frame[:14]) == int.from_bytes(frame[14:16], "little")
