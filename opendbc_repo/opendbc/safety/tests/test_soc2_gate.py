#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py


GATE_BUS = 2
VINFAST_STEER_TX = 0x37A


class TestSoc2Gate(common.PandaSafetyTest):
  TX_MSGS = [[0x400, GATE_BUS], [0x410, GATE_BUS], [0x411, GATE_BUS]]
  FWD_BUS_LOOKUP = {}

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.soc2Gate, GATE_BUS)
    self.safety.init_tests()

  def test_gate_tx_allowed(self):
    # Protocol v2: CMD_LAT/LON are 12 bytes
    self.assertTrue(self._tx(common.make_msg(GATE_BUS, 0x400, 4)))
    self.assertTrue(self._tx(common.make_msg(GATE_BUS, 0x410, 12)))
    self.assertTrue(self._tx(common.make_msg(GATE_BUS, 0x411, 12)))

  def test_gate_tx_wrong_len_rejected(self):
    self.assertFalse(self._tx(common.make_msg(GATE_BUS, 0x400, 8)))
    self.assertFalse(self._tx(common.make_msg(GATE_BUS, 0x410, 8)))
    self.assertFalse(self._tx(common.make_msg(GATE_BUS, 0x410, 4)))

  def test_gate_tx_wrong_bus_rejected(self):
    self.assertFalse(self._tx(common.make_msg(0, 0x400, 4)))
    self.assertFalse(self._tx(common.make_msg(0, 0x410, 12)))
    self.assertFalse(self._tx(common.make_msg(0, 0x411, 12)))

  def test_oem_actuation_blocked_on_chassis_bus(self):
    self.assertFalse(self._tx(common.make_msg(0, VINFAST_STEER_TX, 8)))
    self.assertFalse(self._tx(common.make_msg(0, 0x131, 8)))
    self.assertFalse(self._tx(common.make_msg(2, VINFAST_STEER_TX, 8)))

  def test_controls_allowed_without_rx(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(common.make_msg(0, 0x17E, 8)))
    self.assertTrue(self.safety.get_controls_allowed())


class TestSoc2GateBusZero(common.PandaSafetyTest):
  GATE_BUS = 0
  TX_MSGS = [[0x400, 0], [0x410, 0], [0x411, 0]]
  FWD_BUS_LOOKUP = {}

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.soc2Gate, self.GATE_BUS)
    self.safety.init_tests()

  def test_gate_tx_allowed_on_bus_zero(self):
    b = self.GATE_BUS
    self.assertTrue(self._tx(common.make_msg(b, 0x400, 4)))
    self.assertTrue(self._tx(common.make_msg(b, 0x410, 12)))
    self.assertTrue(self._tx(common.make_msg(b, 0x411, 12)))

  def test_gate_tx_rejected_on_bus_two(self):
    self.assertFalse(self._tx(common.make_msg(2, 0x400, 4)))


if __name__ == "__main__":
  unittest.main()
