import math
from cereal import car
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.vinfast import vinfastcan
from opendbc.car.vinfast.values import CarControllerParams, CANBUS

VisualAlert = structs.CarControl.HUDControl.VisualAlert

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.apply_angle_last = 0.0
    # Both chassis and camera buses use the same DBC, so one packer is sufficient
    self.packer = CANPacker(dbc_names[Bus.chassis])
    # Note: Body CAN bus not accessible on comma3x, only Chassis and Camera buses
    self.frame = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    # VF8 uses angle-based steering control
    # Get desired steering angle from actuators (in degrees)
    desired_angle = CC.actuators.steeringAngleDeg

    # Apply angle rate limiting using standard function
    # This function automatically sets angle to current steering angle when latActive is False
    # Rate limiting is speed-dependent and matches vinfast.h safety code limits
    apply_angle = apply_std_steer_angle_limits(
      desired_angle,
      self.apply_angle_last,
      CS.out.vEgoRaw,
      CS.out.steeringAngleDeg,
      CC.latActive,
      CarControllerParams.ANGLE_LIMITS
    )

    # Store for next iteration (rate limiting)
    self.apply_angle_last = apply_angle

    # Send steering control at 100Hz (every frame at 100Hz loop)
    can_sends.append(vinfastcan.create_steering_control(
      self.packer,
      self.CP,
      self.frame,
      apply_angle,
      CC.latActive
    ))

    # # Send LKA control at 20Hz (on camera bus)
    # if self.frame % 5 == 0:
    #   can_sends.append(vinfastcan.create_lka_control(
    #     self.packer,
    #     self.CP,
    #     self.frame // 5,
    #     CC.latActive
    #   ))

    # Longitudinal control at 50Hz
    if self.frame % 2 == 0:
      if self.CP.openpilotLongitudinalControl:
        # OpenPilot longitudinal: send acceleration commands (on chassis bus, bus 2)
        # Message will be visible on bus 130 (receipt of bus 2)
        accel_cmd = CC.actuators.accel if CC.longActive else 0.0
        # Clip acceleration to car's limits
        accel_cmd = max(CarControllerParams.ACCEL_MIN, min(CarControllerParams.ACCEL_MAX, accel_cmd))
        can_sends.append(vinfastcan.create_acc_control(
            self.packer,
            self.CP,
            self.frame // 2,
            accel_cmd,
            CC.longActive,
            CS.out.standstill
        ))
      else:
        # Stock longitudinal: enable/disable ACC, car handles speed control
        # When cancel is requested, disable ACC immediately
        # Otherwise, enable ACC when openpilot is engaged
        # Note: When longActive is False, car sends ADAS_ACC_Status on camera bus
        # which is read by carstate to determine cruise control state
        if CC.cruiseControl.cancel:
          # Cancel ACC - disable it
          acc_enabled = False
        else:
          # Enable ACC when openpilot is engaged
          acc_enabled = CC.enabled

        accel_cmd = 0.0  # Don't send acceleration commands - car's stock ACC handles speed control
        can_sends.append(vinfastcan.create_acc_control(
            self.packer,
            self.CP,
            self.frame // 2,
            accel_cmd,
            acc_enabled,
            CS.out.standstill
        ))

      # Send drive off request at 50Hz when model indicates car should start from standstill
      # Drive off activates when:
      # - OpenPilot is engaged (CC.enabled)
      # - Car is at standstill (CS.out.standstill)
      # - Long control state is "starting" (model says not shouldStop = time to go)
      drive_off_request = 0
      if (CC.enabled and
          CS.out.standstill and
          CC.actuators.longControlState == car.CarControl.Actuators.LongControlState.starting):
        drive_off_request = 1

      can_sends.append(vinfastcan.create_idb_control(
        self.packer,
        self.frame // 2,
        drive_off_request,
        bus=CANBUS.chassis  # ADAS_IDB is on Chassis bus (bus 2)
      ))

    new_actuators = CC.actuators.as_builder()
    # For angle control, steeringAngleDeg is already set by actuators
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1

    return new_actuators, can_sends
