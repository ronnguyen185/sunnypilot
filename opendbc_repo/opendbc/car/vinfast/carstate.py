import math
from opendbc.can import CANParser
from opendbc.car import Bus, structs, create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.vinfast.values import DBC

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.can_parsers = []
    # Track previous button states for detecting transitions
    self.prev_speed_button_upper = 0
    self.prev_speed_button_lower = 0
    # Track last valid cruise speed to preserve it when tag speed is invalid
    self.last_valid_cruise_speed = 0.0

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

    # Info CAN bus (turn signals, blind spot monitor, etc.)
    # Accessible via second panda or if info CAN is available
    if Bus.body in DBC[CP.carFingerprint]:
      messages_info = [
        ("BCM_LIGHT", 10),      # Turn indicator status (BCM_TurnIndicatorSts)
        ("BCM_SwitchSts", 10),  # Turn indicator switch (BCM_TurnIndicator)
        ("ADAS_BSD", 10),       # Blind spot detection (ADAS_BSD)
        ("ADAS_CMP_ACC", 20),   # ADAS ACC status (ADAS_ACC_TagSpeed, ADAS_ACC_TimeGapSet)
      ]
      ret[Bus.body] = CANParser(DBC[CP.carFingerprint][Bus.body], messages_info, CANBUS.info)

    return ret

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp_chassis = can_parsers[Bus.chassis]
    cp_cam = can_parsers[Bus.cam]
    cp_info = can_parsers.get(Bus.body)  # Info CAN bus (may not be available)

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

    # Set speed and distance gap from ADAS_CMP_ACC on info CAN bus (bus 6)
    # Always read these values regardless of openpilotLongitudinalControl status
    # ADAS_ACC_TagSpeed: Set speed in km/h (0-255, 0xFF = invalid)
    # ADAS_ACC_TimeGapSet: Time gap setting (0 = no gap, 1-4 = distance levels 1-4)
    # ADAS_ACC_SpdSet_upper_Feed: Speed increase button (0 = not pressed, 1 = pressed)
    # ADAS_ACC_SpdSet_lower_Feed: Speed decrease button (0 = not pressed, 1 = pressed)
    button_events = []
    if cp_info is not None and "ADAS_CMP_ACC" in cp_info.vl:
      adas_acc = cp_info.vl["ADAS_CMP_ACC"]
      # Get set speed from ADAS_ACC_TagSpeed (in km/h)
      # This is used by openpilot longitudinal when pcmCruiseSpeed is True
      # Only update cruiseState.speed when we have a valid tag speed
      tag_speed_kmh = adas_acc.get("ADAS_ACC_TagSpeed", 0)
      if tag_speed_kmh != 255:
        # Valid value (including 0): update cruise speed
        # 0 means cruise is off, > 0 means valid set speed
        if tag_speed_kmh > 0:
          # Valid speed: convert from km/h to m/s and set cruise speed
          # Clamp to openpilot's maximum (145 km/h) to allow speeds above car's limit
          from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
          tag_speed_kmh_clamped = min(tag_speed_kmh, V_CRUISE_MAX)
          cruise_speed_ms = tag_speed_kmh_clamped * CV.KPH_TO_MS
          ret.cruiseState.speed = cruise_speed_ms
          ret.cruiseState.speedCluster = cruise_speed_ms
          # Store as last valid speed
          self.last_valid_cruise_speed = cruise_speed_ms
        else:
          # tag_speed == 0: cruise is off, but preserve last valid speed for UI display
          # Set to 0 only if we've never seen a valid speed
          if self.last_valid_cruise_speed > 0:
            ret.cruiseState.speed = self.last_valid_cruise_speed
            ret.cruiseState.speedCluster = self.last_valid_cruise_speed
          else:
            ret.cruiseState.speed = 0.0
            ret.cruiseState.speedCluster = 0.0
      else:
        # If tag_speed is 255 (invalid/unset), use last valid speed to preserve UI display
        if self.last_valid_cruise_speed > 0:
          ret.cruiseState.speed = self.last_valid_cruise_speed
          ret.cruiseState.speedCluster = self.last_valid_cruise_speed
        # If no last valid speed, don't update (preserves previous value or defaults to 0)

      # Get time gap setting and map to distance bars
      # ADAS_ACC_TimeGapSet: 0 = no gap, 1-4 = distance levels 1-4
      # leadDistanceBars: 1 = closest, 3 = farthest (some ports use 2-4)
      time_gap = adas_acc.get("ADAS_ACC_TimeGapSet", 0)
      if time_gap >= 1 and time_gap <= 4:
        # Map time gap levels 1-4 to distance bars 1-4
        # Level 1 = closest (1 bar), Level 4 = farthest (4 bars)
        ret_sp.leadDistanceBars = time_gap
      else:
        # No gap or invalid, don't set leadDistanceBars (keep default)
        pass

      # Parse speed set buttons from car
      # ADAS_ACC_SpdSet_upper_Feed: 0 = not pressed, 1 = pressed (speed increase)
      # ADAS_ACC_SpdSet_lower_Feed: 0 = not pressed, 1 = pressed (speed decrease)
      # Detect transitions from 0 to 1 (button press) to create button events
      speed_button_upper = adas_acc.get("ADAS_ACC_SpdSet_upper_Feed", 0)
      speed_button_lower = adas_acc.get("ADAS_ACC_SpdSet_lower_Feed", 0)

      # Create button events when buttons transition from 0 to 1 (pressed)
      # Use create_button_events helper: (current, previous, mapping_dict)
      # Mapping: 1 = button pressed -> ButtonType.accelCruise or ButtonType.decelCruise
      button_events.extend(create_button_events(
        speed_button_upper, self.prev_speed_button_upper,
        {1: ButtonType.accelCruise}
      ))
      button_events.extend(create_button_events(
        speed_button_lower, self.prev_speed_button_lower,
        {1: ButtonType.decelCruise}
      ))

      # Store current state for next iteration
      self.prev_speed_button_upper = speed_button_upper
      self.prev_speed_button_lower = speed_button_lower

    # Set button events (empty list if no buttons pressed or info CAN not available)
    ret.buttonEvents = button_events

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

    # Turn signals and blind spot monitor from Info CAN bus
    # BCM_TurnIndicatorSts: 0 = OFF, 1 = LEFT, 2 = RIGHT, 3 = INVALID
    # BCM_TurnIndicator: 0 = OFF, 1 = LEFT, 2 = RIGHT, 3 = INVALID
    # ADAS_BSD_IndLeft/IndRight: 0 = off, 1 = on (warning stage 1), 2 = flashing (warning stage 2), 3 = sensor fault
    if cp_info is not None:
      # Turn signals - prefer BCM_TurnIndicatorSts (status) over BCM_TurnIndicator (switch)
      if "BCM_LIGHT" in cp_info.vl:
        turn_indicator_sts = cp_info.vl["BCM_LIGHT"].get("BCM_TurnIndicatorSts", 0)
        ret.leftBlinker = turn_indicator_sts == 1  # LEFT
        ret.rightBlinker = turn_indicator_sts == 2  # RIGHT
      elif "BCM_SwitchSts" in cp_info.vl:
        turn_indicator_switch = cp_info.vl["BCM_SwitchSts"].get("BCM_TurnIndicator", 0)
        ret.leftBlinker = turn_indicator_switch == 1  # LEFT
        ret.rightBlinker = turn_indicator_switch == 2  # RIGHT
      else:
        ret.leftBlinker = False
        ret.rightBlinker = False

      # Blind spot monitor
      # ADAS_BSD_IndLeft/IndRight: 1 = on (warning), 2 = flashing (strong warning), 0/3 = off/fault
      if "ADAS_BSD" in cp_info.vl:
        bsd_ind_left = cp_info.vl["ADAS_BSD"].get("ADAS_BSD_IndLeft", 0)
        bsd_ind_right = cp_info.vl["ADAS_BSD"].get("ADAS_BSD_IndRight", 0)
        # Left blind spot: indicator is on (1) or flashing (2)
        ret.leftBlindspot = bsd_ind_left == 1 or bsd_ind_left == 2
        # Right blind spot: indicator is on (1) or flashing (2)
        ret.rightBlindspot = bsd_ind_right == 1 or bsd_ind_right == 2
      else:
        ret.leftBlindspot = False
        ret.rightBlindspot = False
    else:
      # Info CAN bus not available - set defaults
      ret.leftBlinker = False
      ret.rightBlinker = False
      ret.leftBlindspot = False
      ret.rightBlindspot = False

    # TODO: Add more signals as needed (brake, gas, seatbelt, etc.)

    return ret, ret_sp
