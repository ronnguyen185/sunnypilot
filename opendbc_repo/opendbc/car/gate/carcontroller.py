from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  """No OEM CAN actuation — soc2d streams gate protocol frames instead."""

  def update(self, CC, CC_SP, CS, now_nanos):
    return CC.actuators.as_builder(), []
