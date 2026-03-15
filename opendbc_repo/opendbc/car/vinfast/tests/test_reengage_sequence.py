from types import SimpleNamespace

from cereal import car
from opendbc.car import Bus
from opendbc.car.vinfast.carcontroller import CarController


class DummyActuators:
  def __init__(self, steering_angle_deg=0.0, accel=-2.0,
               long_state=car.CarControl.Actuators.LongControlState.starting):
    self.steeringAngleDeg = steering_angle_deg
    self.accel = accel
    self.longControlState = long_state

  def as_builder(self):
    return DummyActuators(self.steeringAngleDeg, self.accel, self.longControlState)


def make_cc(long_active=True, lat_active=True, accel=-2.0):
  return SimpleNamespace(
    actuators=DummyActuators(accel=accel),
    latActive=lat_active,
    longActive=long_active,
    enabled=True,
    cruiseControl=SimpleNamespace(cancel=False),
  )


def make_cc_sp():
  # Road clear so re-engage branch uses launch accel path.
  return SimpleNamespace(
    leadOne=SimpleNamespace(status=False, dRel=999.0, vRel=0.0),
    leadTwo=SimpleNamespace(status=False, dRel=999.0, vRel=0.0),
  )


def make_cs(v_ego=0.0, standstill=True, acc_popup_feed=0):
  return SimpleNamespace(
    out=SimpleNamespace(
      vEgoRaw=v_ego,
      steeringAngleDeg=0.0,
      vEgo=v_ego,
      standstill=standstill,
    ),
    acc_popup_feed=acc_popup_feed,
  )


def test_multiple_reengage_stop_go_retriggers_launch_boost(monkeypatch):
  sent_acc_cmds: list[tuple[float, bool, bool, int]] = []

  def fake_create_steering_control(*_args, **_kwargs):
    return (0, b"", 0)

  def fake_create_idb_control(*_args, **_kwargs):
    return (0, b"", 0)

  def fake_create_acc_control(_packer, _cp, _frame, accel, _long_active, standstill,
                              acc_popup_feed=0, reengage_active=False):
    sent_acc_cmds.append((float(accel), bool(standstill), bool(reengage_active), int(acc_popup_feed)))
    return (0, b"", 0)

  monkeypatch.setattr("opendbc.car.vinfast.vinfastcan.create_steering_control", fake_create_steering_control)
  monkeypatch.setattr("opendbc.car.vinfast.vinfastcan.create_idb_control", fake_create_idb_control)
  monkeypatch.setattr("opendbc.car.vinfast.vinfastcan.create_acc_control", fake_create_acc_control)

  cp = SimpleNamespace(carFingerprint="VINFAST_VF8", openpilotLongitudinalControl=True)
  dbc_names = {Bus.chassis: "vinfast_vf8_chassis_can", Bus.cam: "vinfast_vf8_chassis_can"}
  controller = CarController(dbc_names, cp, SimpleNamespace())
  cc_sp = make_cc_sp()

  # 1) Initial popup re-engage at standstill -> launch kick expected.
  controller.frame = 0
  controller.update(make_cc(long_active=True), cc_sp, make_cs(v_ego=0.0, standstill=True, acc_popup_feed=3), now_nanos=0)
  accel_1, standstill_1, reengage_1, popup_1 = sent_acc_cmds[-1]
  assert standstill_1 is True and popup_1 == 3
  assert reengage_1 is True
  assert accel_1 >= 0.59

  # 2) Move off (same re-engage session, no popup) -> should still have kick.
  controller.frame = 0
  controller.update(make_cc(long_active=True), cc_sp, make_cs(v_ego=0.6, standstill=False, acc_popup_feed=0), now_nanos=0)
  accel_2, standstill_2, reengage_2, popup_2 = sent_acc_cmds[-1]
  assert standstill_2 is False and popup_2 == 0
  assert reengage_2 is True
  assert accel_2 >= 0.59

  # 3) Stop again while still re-engaged.
  controller.frame = 0
  controller.update(make_cc(long_active=True), cc_sp, make_cs(v_ego=0.0, standstill=True, acc_popup_feed=0), now_nanos=0)
  accel_3, standstill_3, reengage_3, _ = sent_acc_cmds[-1]
  assert standstill_3 is True and reengage_3 is True
  assert accel_3 >= 0.59

  # 4) Second move-off without new popup -> boost should retrigger from standstill transition.
  controller.frame = 0
  controller.update(make_cc(long_active=True), cc_sp, make_cs(v_ego=0.7, standstill=False, acc_popup_feed=0), now_nanos=0)
  accel_4, standstill_4, reengage_4, popup_4 = sent_acc_cmds[-1]
  assert standstill_4 is False and popup_4 == 0
  assert reengage_4 is True
  assert accel_4 >= 0.59
