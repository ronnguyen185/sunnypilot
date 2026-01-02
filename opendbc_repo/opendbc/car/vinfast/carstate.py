import math
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.vinfast.values import DBC


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.can_parsers = []

  def get_can_parsers(self, CP, CP_SP):
    ret = {}

    # Chassis bus (steering, brakes, etc.)
    # CANParser expects: list of (message_name, frequency) tuples
    messages_chassis = [
      ("SAS_Sensor", 50),
      ("EPS_ADAS_Steering_Trq", 50),
      ("IDB_STATUS", 50),
      ("IDB_AVL_RPM_WHL_FRONT", 50),
      ("IDB_AVL_RPM_WHL_REAR", 50),
      ("EPS_SteeringHoldState", 50),
      ("EPS_Advanced", 50),  # EPS_Advanced for EPSAngRespSts (steering fault detection)
      ("YSS_YawRate", 50),  # Yaw rate sensor from YSS (Yaw Stability System)
    ]

    # Camera/SCAM bus (bus 0) - ADAS_ACC_Status is sent by car when longActive is False
    # When longActive is True, openpilot sends this message, so we don't need to read it
    messages_cam = [
      ("ADAS_ACC_Status", 50),
    ]

    # Use physical bus numbers
    from opendbc.car.vinfast.values import CANBUS
    ret[Bus.chassis] = CANParser(DBC[CP.carFingerprint][Bus.chassis], messages_chassis, CANBUS.chassis)
    ret[Bus.cam] = CANParser(DBC[CP.carFingerprint][Bus.cam], messages_cam, CANBUS.cam)

    # Note: Info and Body CAN buses are not accessible on comma3x
    # Only Chassis and Camera buses are available
    # Gear, doors, and other body signals are not accessible

    return ret

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp_chassis = can_parsers[Bus.chassis]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # Wheel speeds (convert from rad/s to m/s)
    # TODO: verify the conversion factor
    if "IDB_AVL_RPM_WHL_FRONT" in cp_chassis.vl and "IDB_AVL_RPM_WHL_REAR" in cp_chassis.vl:
      fl_wheel_speed = cp_chassis.vl["IDB_AVL_RPM_WHL_FRONT"].get("AVL_RPM_WHL_FLH", 0) * 0.3
      fr_wheel_speed = cp_chassis.vl["IDB_AVL_RPM_WHL_FRONT"].get("AVL_RPM_WHL_FRH", 0) * 0.3
      rl_wheel_speed = cp_chassis.vl["IDB_AVL_RPM_WHL_REAR"].get("AVL_RPM_WHL_RLH", 0) * 0.3
      rr_wheel_speed = cp_chassis.vl["IDB_AVL_RPM_WHL_REAR"].get("AVL_RPM_WHL_RRH", 0) * 0.3
      self.parse_wheel_speeds(ret, fl_wheel_speed, fr_wheel_speed, rl_wheel_speed, rr_wheel_speed)
    else:
      # Default to zero wheel speeds if messages not available
      self.parse_wheel_speeds(ret, 0, 0, 0, 0)

    # Vehicle speed
    if "IDB_STATUS" in cp_chassis.vl:
      vehicle_speed_kph = cp_chassis.vl["IDB_STATUS"].get("VehicleSpd", 0)
      ret.vEgo = vehicle_speed_kph * CV.KPH_TO_MS
      ret.vEgoRaw = ret.vEgo
      ret.standstill = cp_chassis.vl["IDB_STATUS"].get("ESC_VehicleStandstill", 0) == 1
    else:
      ret.vEgo = 0.0
      ret.vEgoRaw = 0.0
      ret.standstill = True

    # Steering
    if "SAS_Sensor" in cp_chassis.vl:
      ret.steeringAngleDeg = cp_chassis.vl["SAS_Sensor"].get("SAS_SteerWheelAngle", 0)
      ret.steeringRateDeg = cp_chassis.vl["SAS_Sensor"].get("SAS_SteerWhlRotSpd", 0)
    else:
      ret.steeringAngleDeg = 0.0
      ret.steeringRateDeg = 0.0

    if "EPS_ADAS_Steering_Trq" in cp_chassis.vl:
      ret.steeringTorque = cp_chassis.vl["EPS_ADAS_Steering_Trq"].get("EPS_SteeringDriverTorque", 0)
      ret.steeringTorqueEps = cp_chassis.vl["EPS_ADAS_Steering_Trq"].get("EPS_SteeringEMTorque", 0)
    else:
      ret.steeringTorque = 0.0
      ret.steeringTorqueEps = 0.0

    ret.steeringPressed = abs(ret.steeringTorque) > 50  # TODO: calibrate threshold

    # Gear - not accessible on comma3x (Body CAN not connected)
    # TODO: Implement proper gear detection from CAN messages
    # For now, fake gear position as always Drive (D)
    ret.gearShifter = structs.CarState.GearShifter.drive

    # Doors - not accessible on comma3x (Body CAN not connected)
    # Set to False as fallback
    ret.doorOpen = False

    # Cruise control
    # ADAS_ACC_Status on camera bus (bus 0):
    # - When longActive is False: car sends this message, read it to determine cruise state
    # - When longActive is True: openpilot sends this message, ignore car's message
    # Read from camera bus to get cruise control status from car
    # ADAS_ACC_Main_Mode: 0 = disabled, 1 = enabled/available
    # ADAS_ACC_Mode: 0-7, where 4 = active, 2 = inactive, 7 = fault
    if "ADAS_ACC_Status" in cp_cam.vl:
      acc_status = cp_cam.vl["ADAS_ACC_Status"]
      acc_mode = acc_status.get("ADAS_ACC_Mode", 0)
      acc_main_mode = acc_status.get("ADAS_ACC_Main_Mode", 0)
      # Main_Mode = 1 means ACC system is enabled/ready
      # Mode = 4 means ACC is actively controlling speed
      # Mode = 2 means ACC is inactive (standby)
      # Mode = 7 means ACC fault
      # Available: ACC is ready to be used (not in fault mode, message is present)
      # Enabled: ACC is actively controlling speed (Mode = 4)
      ret.cruiseState.available = acc_mode != 7  # Not in fault mode
      ret.cruiseState.enabled = acc_mode == 4  # Mode 4 = active, Mode 2 = inactive, mode 7 = fault
    else:
      # Default if message not available
      ret.cruiseState.available = False
      ret.cruiseState.enabled = False
    ret.cruiseState.standstill = ret.standstill

    # EPS status - check EPSAngRespSts in EPS_Advanced message
    # EPSAngRespSts: 0 = not ready/error, 1 = ready, 2 = active, 3 = fault
    # Should be ready (1) before latActive, and active (2) when latActive
    if "EPS_Advanced" in cp_chassis.vl:
      eps_angle_resp_sts = cp_chassis.vl["EPS_Advanced"].get("EPSAngRespSts", 0)
      # Set fault if not ready (0) or fault (3)
      # Ready (1) and active (2) are both OK
      ret.steerFaultTemporary = eps_angle_resp_sts == 0 or eps_angle_resp_sts == 3
    else:
      # Message not received - don't set fault (EPS might not send it until we start controlling)
      ret.steerFaultTemporary = False

    ret.steerFaultPermanent = False

    # Yaw rate from YSS (Yaw Stability System)
    # YSS_YAW_RATE is in deg/s, convert to rad/s for openpilot
    # Signal range: [-163.83, 163.83] deg/s, scaling: 0.005 deg/s per bit, offset: -163.83
    if "YSS_YawRate" in cp_chassis.vl:
      yaw_rate_deg_s = cp_chassis.vl["YSS_YawRate"].get("YSS_YAW_RATE", 0.0)
      yaw_rate_qual = cp_chassis.vl["YSS_YawRate"].get("YSS_YAW_RATE_QUAL", 0)
      # Convert from deg/s to rad/s
      # Only use yaw rate if quality is valid (quality > 0 indicates valid data)
      if yaw_rate_qual > 0:
        ret.yawRate = yaw_rate_deg_s * CV.DEG_TO_RAD
      else:
        ret.yawRate = 0.0
    else:
      ret.yawRate = 0.0

    # TODO: Add more signals as needed (brake, gas, seatbelt, etc.)

    return ret, ret_sp
