from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.vinfast.values import CANBUS
from opendbc.car.vinfast.vinfastcan import create_acc_control


def _make_cp(openpilot_longitudinal_control: bool = True):
  cp = structs.CarParams()
  cp.openpilotLongitudinalControl = openpilot_longitudinal_control
  return cp


def _decode_acc_status(msg, bus):
  parser = CANParser("vinfast_vf8_chassis_can", [("ADAS_ACC_Status", 0)], bus)
  parser.update([0, [msg]])
  return parser.vl["ADAS_ACC_Status"]


class TestVinfastAccControl:
  def test_accel_clipping_and_standstill_req_at_stop(self):
    cp = _make_cp(True)
    packer = CANPacker("vinfast_vf8_chassis_can")

    # Over-limit accel should clip to +6.0, and standstill request must be 1 at standstill.
    msg = create_acc_control(
      packer, cp, frame=0, accel=9.0, long_active=True,
      standstill=True, acc_popup_feed=0, reengage_active=False,
    )
    vals = _decode_acc_status(msg, CANBUS.chassis)

    assert vals["ADAS_ACC_AccelDecel_Cmd"] == 6.0
    assert vals["ADAS_ACC_StandstillReq"] == 1

  def test_standstill_req_clears_when_not_stopped(self):
    cp = _make_cp(True)
    packer = CANPacker("vinfast_vf8_chassis_can")

    msg = create_acc_control(
      packer, cp, frame=1, accel=0.5, long_active=True,
      standstill=False, acc_popup_feed=0, reengage_active=False,
    )
    vals = _decode_acc_status(msg, CANBUS.chassis)

    assert vals["ADAS_ACC_StandstillReq"] == 0

  def test_standstill_req_stays_one_during_reengage_when_stopped(self):
    cp = _make_cp(True)
    packer = CANPacker("vinfast_vf8_chassis_can")

    # Even with popup/reengage flags, standstill request should remain 1 if vehicle is stopped.
    msg = create_acc_control(
      packer, cp, frame=2, accel=0.6, long_active=True,
      standstill=True, acc_popup_feed=3, reengage_active=True,
    )
    vals = _decode_acc_status(msg, CANBUS.chassis)

    assert vals["ADAS_ACC_StandstillReq"] == 1

  def test_long_inactive_forces_zero_accel_and_req_zero(self):
    cp = _make_cp(True)
    packer = CANPacker("vinfast_vf8_chassis_can")

    msg = create_acc_control(
      packer, cp, frame=3, accel=2.0, long_active=False,
      standstill=True, acc_popup_feed=3, reengage_active=True,
    )
    vals = _decode_acc_status(msg, CANBUS.chassis)

    assert vals["ADAS_ACC_AccelDecel_Cmd"] == 0.0
    assert vals["ADAS_ACC_StandstillReq"] == 0
