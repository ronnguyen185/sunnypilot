"""Unit tests matching soc1/tests/test_encode.cpp for protocol v2."""

import struct

from openpilot.selfdrive.gate.gate_crc import crc16_ccitt_false
from openpilot.selfdrive.gate.gate_encode import (
  CMD_LEN,
  LAT_REQ_ANGLE,
  LAT_REQ_NONE,
  LAT_UNIT_DEGREE,
  LAT_UNIT_NONE,
  LON_REQ_ACCEL,
  LON_REQ_NONE,
  LON_UNIT_MPS2,
  LON_UNIT_NONE,
  build_cmd_lat,
  build_cmd_lon,
  build_source_health,
  encode_accel_mps2,
  encode_steer_deg,
)


class TestCrc:
  def test_check_value(self):
    assert crc16_ccitt_false(b"123456789") == 0x29B1

  def test_empty_data(self):
    assert crc16_ccitt_false(None) == 0xFFFF

  def test_single_zero_byte(self):
    assert crc16_ccitt_false(b"\x00") == 0xE1F0

  def test_two_zero_bytes(self):
    assert crc16_ccitt_false(b"\x00\x00") == 0x1D0F

  def test_health_payload_match(self):
    payload = bytes([0x05, 0x01])
    crc = crc16_ccitt_false(payload)
    le = struct.pack("<H", crc)
    assert struct.unpack("<H", le)[0] == crc


class TestHealth:
  def test_counter_and_available(self):
    frame = build_source_health(5, True)
    assert frame[0] == 5
    assert frame[1] == 0x01

  def test_available_false(self):
    frame = build_source_health(0, False)
    assert frame[1] == 0x00

  def test_reserved_bits_zero(self):
    frame = build_source_health(1, True)
    assert frame[1] & 0xFE == 0x00

  def test_crc_is_correct(self):
    frame = build_source_health(5, True)
    expected = crc16_ccitt_false(frame[:2])
    stored = struct.unpack("<H", frame[2:4])[0]
    assert stored == expected


class TestScaling:
  def test_steer_001_deg(self):
    assert encode_steer_deg(15.0) == 15000
    assert encode_steer_deg(150.0) == 150000

  def test_accel_001_mps2(self):
    assert encode_accel_mps2(0.05) == 50
    assert encode_accel_mps2(-1.5) == -1500


class TestLat:
  def test_steer_angle_active(self):
    frame = build_cmd_lat(1, True, LAT_REQ_ANGLE, LAT_UNIT_DEGREE, 15000)
    assert len(frame) == CMD_LEN
    assert frame[0] == 1
    assert frame[1] == 0x01
    assert frame[2] == LAT_REQ_ANGLE
    assert frame[3] == LAT_UNIT_DEGREE
    assert struct.unpack_from("<i", frame, 4)[0] == 15000
    assert frame[8] == 0 and frame[9] == 0
    expected = crc16_ccitt_false(frame[:10])
    assert struct.unpack_from("<H", frame, 10)[0] == expected

  def test_inactive(self):
    frame = build_cmd_lat(0, False, LAT_REQ_NONE, LAT_UNIT_NONE, 0)
    assert frame[1] == 0x00
    assert frame[2] == 0
    assert frame[3] == 0
    assert struct.unpack_from("<i", frame, 4)[0] == 0

  def test_negative_target(self):
    frame = build_cmd_lat(1, True, LAT_REQ_ANGLE, LAT_UNIT_DEGREE, -1234)
    assert struct.unpack_from("<i", frame, 4)[0] == -1234


class TestLon:
  def test_accel_active(self):
    frame = build_cmd_lon(1, True, LON_REQ_ACCEL, LON_UNIT_MPS2, 50)
    assert len(frame) == CMD_LEN
    assert frame[2] == LON_REQ_ACCEL
    assert frame[3] == LON_UNIT_MPS2
    assert struct.unpack_from("<i", frame, 4)[0] == 50
    expected = crc16_ccitt_false(frame[:10])
    assert struct.unpack_from("<H", frame, 10)[0] == expected

  def test_inactive(self):
    frame = build_cmd_lon(0, False, LON_REQ_NONE, LON_UNIT_NONE, 0)
    assert frame[1] == 0x00
    assert frame[2] == 0
    assert frame[3] == 0

  def test_negative_accel(self):
    frame = build_cmd_lon(1, True, LON_REQ_ACCEL, LON_UNIT_MPS2, -500)
    assert struct.unpack_from("<i", frame, 4)[0] == -500
