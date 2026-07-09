from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase


class CarState(CarStateBase):
  """Minimal carState for gate-only SOC2 (no OEM CAN parsers)."""

  def get_can_parsers(self, CP, CP_SP):
    return {}

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # Bench / gate-link: card only needs a valid carState for soc2d health when
    # GateSOC2BenchNoChassis=0. With bench mode, health.py skips carState anyway.
    ret.canValid = True
    ret.standstill = True
    ret.gearShifter = structs.CarState.GearShifter.drive
    ret.cruiseState.enabled = False
    ret.cruiseState.available = True

    return ret, ret_sp
