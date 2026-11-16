import math
from opendbc.can import CANParser
from opendbc.car import Bus, structs, create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.vinfast.values import CAR, CarControllerParams, DBC, is_vf6_safety_platform

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.can_parsers = []
    # Track previous button states for detecting transitions
    self.prev_speed_button_upper = 0
    self.prev_speed_button_lower = 0
    # Track last valid speed limit
    self.last_valid_speed_limit = 0.0
    # Track last valid time gap setting (1-4) for fallback when info CAN is unavailable
    self.last_valid_time_gap = 3  # Default to level 3 (medium distance)
    # ACC popup feed from ADAS_CMP_ACC message
    self.acc_popup_feed = 0
    # Latest IDB_Info_Flag from chassis (used for hybrid long-control bridge)
    self.idb_info_flag = {}
    # Stock SCAM ACC on cam bus (hybrid mode mirror — same as joystick working --vinfast)
    self.scam_acc_mode = None
    self.scam_accel_cmd = 0.0
    # Sticky steeringPressed: engage fast, release only after sustained low torque
    self.steering_pressed_latched = False
    self.steering_pressed = False
    self.steering_press_cnt = 0
    self.steering_release_cnt = 0

  def get_can_parsers(self, CP, CP_SP):
    ret = {}

    # Chassis bus (steering, brakes, etc.)
    # CANParser expects: list of (message_name, frequency) tuples
    messages_chassis = [
      ("SAS_Sensor", 50),
      ("EPS_ADAS_Steering_Trq", 50),
      ("IDB_STATUS", 50),
      ("IDB_Info_Flag", 50),
      ("IDB_AVL_RPM_WHL_FRONT", 50),
      ("IDB_AVL_RPM_WHL_REAR", 50),
      ("EPS_SteeringHoldState", 50),
      ("EPS_Advanced", 50),  # EPS_Advanced for EPSAngRespSts (steering fault detection)
      ("YSS_YawRate", 50),  # Yaw rate sensor from YSS (Yaw Stability System)
    ]
    if not is_vf6_safety_platform(CP.carFingerprint):
      messages_chassis.append(("BCM_CLAMP_STAT", 10))

    # Camera/SCAM bus (bus 2) - ADAS_ACC_Status is sent by car when longActive is False
    # When longActive is True, openpilot sends this message, so we don't need to read it
    messages_cam = [
      ("ADAS_ACC_Status", 50),
    ]

    # Use physical bus numbers
    from opendbc.car.vinfast.values import CANBUS
    ret[Bus.chassis] = CANParser(DBC[CP.carFingerprint][Bus.chassis], messages_chassis, CANBUS.chassis)
    ret[Bus.cam] = CANParser(DBC[CP.carFingerprint][Bus.cam], messages_cam, CANBUS.cam)

    # InfoCAN bus (bus 5): turn signals, BSM, cruise set-speed buttons, TSR
    if Bus.body in DBC[CP.carFingerprint]:
      messages_info = [
        ("BCM_LIGHT", 10),      # Turn indicator status (BCM_TurnIndicatorSts)
        ("BCM_SwitchSts", 10),  # Turn indicator switch (BCM_TurnIndicator)
        ("BCM_CLAMP_STAT", 10), # Brake light switch (STAT_BLS) on VF6 info bus
        ("ADAS_BSD", 10),       # Blind spot detection (ADAS_BSD)
        ("ADAS_CMP_ACC", 20),   # ADAS ACC status (ADAS_ACC_TagSpeed, ADAS_ACC_TimeGapSet)
        ("ADAS_TSR_STATUS", 10), # Traffic Sign Recognition status
        ("ADAS_Speed_Display", 10), # ISA speed limit display
      ]
      ret[Bus.body] = CANParser(DBC[CP.carFingerprint][Bus.body], messages_info, CANBUS.info_bus(CP.carFingerprint))

    return ret

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp_chassis = can_parsers[Bus.chassis]
    cp_cam = can_parsers[Bus.cam]
    cp_info = can_parsers.get(Bus.body)  # InfoCAN bus (may not be available)

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

    # Sticky driver takeover detection:
    # - torque hysteresis (noise ~1 Nm) + OEM EPS_Drive_Intervention
    # - fast engage / slow release so steerOverride does not ping-pong mid-turn
    # - when EPS is angle-active, raise press threshold with |steerRate| so
    #   torsion-bar reaction during OP large turns is not treated as driver input
    eps_angle_active = False
    if "EPS_Advanced" in cp_chassis.vl:
      # EPSAngRespSts: 2 = active (lat control engaged)
      eps_angle_active = cp_chassis.vl["EPS_Advanced"].get("EPSAngRespSts", 0) == 2

    press_nm = CarControllerParams.STEER_DRIVER_PRESS_NM
    if eps_angle_active:
      press_nm = min(
        CarControllerParams.STEER_PRESS_NM_MAX,
        press_nm + CarControllerParams.STEER_PRESS_RATE_GAIN * abs(ret.steeringRateDeg),
      )

    torque_abs = abs(ret.steeringTorque)
    if torque_abs > press_nm:
      self.steering_pressed_latched = True
    elif torque_abs < CarControllerParams.STEER_DRIVER_RELEASE_NM:
      self.steering_pressed_latched = False

    eps_drive_intervention = False
    if "EPS_SteeringHoldState" in cp_chassis.vl:
      # Only trust OEM intervention when torque also clears the rate-aware press
      # threshold — the flag alone can follow torsion-bar spikes on OP turns.
      eps_drive_intervention = (
        cp_chassis.vl["EPS_SteeringHoldState"].get("EPS_Drive_Intervention", 0) == 1
        and torque_abs > press_nm
      )

    raw_pressed = self.steering_pressed_latched or eps_drive_intervention
    if raw_pressed:
      self.steering_release_cnt = 0
      self.steering_press_cnt = min(
        self.steering_press_cnt + 1, CarControllerParams.STEER_PRESSED_MIN_COUNT
      )
      if self.steering_press_cnt >= CarControllerParams.STEER_PRESSED_MIN_COUNT:
        self.steering_pressed = True
    else:
      self.steering_press_cnt = 0
      self.steering_release_cnt = min(
        self.steering_release_cnt + 1, CarControllerParams.STEER_RELEASE_MIN_COUNT
      )
      if self.steering_release_cnt >= CarControllerParams.STEER_RELEASE_MIN_COUNT:
        self.steering_pressed = False
    ret.steeringPressed = self.steering_pressed

    # Gear - not accessible on comma3x (Body CAN not connected)
    # TODO: Implement proper gear detection from CAN messages
    # For now, fake gear position as always Drive (D)
    ret.gearShifter = structs.CarState.GearShifter.drive

    # Doors - not accessible on comma3x (Body CAN not connected)
    # Set to False as fallback
    ret.doorOpen = False

    # Cruise control
    # ADAS_ACC_Status on camera bus (bus 2):
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

    # Set speed / distance gap / cruise buttons from ADAS_CMP_ACC on InfoCAN (bus 5)
    # ADAS_ACC_TagSpeed: Set speed in km/h (0-255, 0xFF = invalid)
    # ADAS_ACC_TimeGapSet: Time gap setting (0 = no gap, 1-4 = distance levels 1-4)
    # ADAS_ACC_SpdSet_upper_Feed: Speed increase button (0 = not pressed, 1 = pressed)
    # ADAS_ACC_SpdSet_lower_Feed: Speed decrease button (0 = not pressed, 1 = pressed)
    button_events = []
    if cp_info is not None and "ADAS_CMP_ACC" in cp_info.vl:
      adas_acc = cp_info.vl["ADAS_CMP_ACC"]

      # SpdSet feed from ADAS_CMP_ACC (MFS_Control_Button 0x3F7 is not on VF6 comma3x info bus)
      speed_button_upper = int(adas_acc.get("ADAS_ACC_SpdSet_upper_Feed", 0) or 0)
      speed_button_lower = int(adas_acc.get("ADAS_ACC_SpdSet_lower_Feed", 0) or 0)
      button_events.extend(create_button_events(
        speed_button_upper, self.prev_speed_button_upper,
        {1: ButtonType.accelCruise},
      ))
      button_events.extend(create_button_events(
        speed_button_lower, self.prev_speed_button_lower,
        {1: ButtonType.decelCruise},
      ))
      self.prev_speed_button_upper = speed_button_upper
      self.prev_speed_button_lower = speed_button_lower

      # Set speed from raw ADAS_ACC_TagSpeed (km/h); 0 = off, 255 = invalid
      tag_speed_kmh = int(adas_acc.get("ADAS_ACC_TagSpeed", 0) or 0)
      if tag_speed_kmh != 255:
        if tag_speed_kmh > 0:
          from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
          tag_speed_kmh = min(tag_speed_kmh, V_CRUISE_MAX)
          cruise_speed_ms = tag_speed_kmh * CV.KPH_TO_MS
          ret.cruiseState.speed = cruise_speed_ms
          ret.cruiseState.speedCluster = cruise_speed_ms
        else:
          ret.cruiseState.speed = 0.0
          ret.cruiseState.speedCluster = 0.0

      # ACC popup feed from ADAS_CMP_ACC message
      # ADAS_ACC_PopUp_Feed: 3 = "Press gas pedal to re-engage the function"
      self.acc_popup_feed = adas_acc.get("ADAS_ACC_PopUp_Feed", 0)
      
      # Get time gap setting and map to distance bars
      # ADAS_ACC_TimeGapSet: 0 = no gap, 1-4 = distance levels 1-4
      # leadDistanceBars: 1 = closest, 4 = farthest
      time_gap = adas_acc.get("ADAS_ACC_TimeGapSet", 0)
      if time_gap >= 1 and time_gap <= 4:
        # Map time gap levels 1-4 to distance bars 1-4
        # Level 1 = closest (1 bar), Level 4 = farthest (4 bars)
        ret_sp.leadDistanceBars = time_gap
        self.last_valid_time_gap = time_gap  # Store for fallback
      else:
        # No gap or invalid, use last valid time gap if available
        if self.last_valid_time_gap >= 1 and self.last_valid_time_gap <= 4:
          ret_sp.leadDistanceBars = self.last_valid_time_gap
        # Otherwise, don't set leadDistanceBars (keep default)

    else:
      # ADAS_CMP_ACC not available - set acc_popup_feed to 0
      self.acc_popup_feed = 0
      # InfoCAN bus not available - use last valid time gap if available
      if self.last_valid_time_gap >= 1 and self.last_valid_time_gap <= 4:
        ret_sp.leadDistanceBars = self.last_valid_time_gap

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

    # Turn signals + BSM from InfoCAN (bus 5)
    # BCM_TurnIndicatorSts: 0=OFF, 1=LEFT, 2=RIGHT; ADAS_BSD_IndLeft/IndRight: 0/3=off, 1=on, 2=flash
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
      # InfoCAN bus not available - set defaults
      ret.leftBlinker = False
      ret.rightBlinker = False
      ret.leftBlindspot = False
      ret.rightBlindspot = False

    # Speed limit from TSR (Traffic Sign Recognition)
    # ADAS_TSR_Typ1: 1 = "Maximum Speed Limit", 2 = "End of speed limit sign"
    # ADAS_TSR_Typ1_value: 0 = "No value", 1-31 = speeds 5-155 km/h (incrementing by 5)
    # Speed calculation: speed_kmh = value * 5 (for value 1-31)
    if cp_info is not None and "ADAS_TSR_STATUS" in cp_info.vl:
      tsr_status = cp_info.vl["ADAS_TSR_STATUS"]
      tsr_typ1 = tsr_status.get("ADAS_TSR_Typ1", 0)
      tsr_typ1_value = tsr_status.get("ADAS_TSR_Typ1_value", 0)

      # Check if we have a valid speed limit sign (Typ1 = 1 = Maximum Speed Limit)
      # and a valid speed value (1-31)
      if tsr_typ1 == 1 and 1 <= tsr_typ1_value <= 31:
        # Convert value to km/h: 1 = 5 km/h, 2 = 10 km/h, ..., 31 = 155 km/h
        speed_limit_kmh = tsr_typ1_value * 5
        speed_limit_ms = speed_limit_kmh * CV.KPH_TO_MS
        ret_sp.speedLimit = speed_limit_ms
        self.last_valid_speed_limit = speed_limit_ms
      elif tsr_typ1 == 2:
        # End of speed limit sign - clear speed limit
        ret_sp.speedLimit = 0.0
        self.last_valid_speed_limit = 0.0
      else:
        # No valid speed limit detected - clear speed limit to allow updates
        # Don't use last valid speed limit to prevent it from staying forever
        ret_sp.speedLimit = 0.0
        self.last_valid_speed_limit = 0.0
    else:
      # TSR message not available - clear speed limit to allow updates
      # Don't use last valid speed limit to prevent it from staying forever
      ret_sp.speedLimit = 0.0
      self.last_valid_speed_limit = 0.0

    if "IDB_Info_Flag" in cp_chassis.vl:
      self.idb_info_flag = dict(cp_chassis.vl["IDB_Info_Flag"])

    if "ADAS_ACC_Status" in cp_cam.vl:
      cam_acc = cp_cam.vl["ADAS_ACC_Status"]
      mode = cam_acc.get("ADAS_ACC_Mode")
      if mode is not None:
        self.scam_acc_mode = int(mode)
      cmd = cam_acc.get("ADAS_ACC_AccelDecel_Cmd", 0)
      if cmd is not None:
        self.scam_accel_cmd = float(cmd)

    if "BCM_CLAMP_STAT" in cp_chassis.vl:
      ret.brakePressed = cp_chassis.vl["BCM_CLAMP_STAT"].get("STAT_BLS", 0) == 1
    elif cp_info is not None and "BCM_CLAMP_STAT" in cp_info.vl:
      ret.brakePressed = cp_info.vl["BCM_CLAMP_STAT"].get("STAT_BLS", 0) == 1
    else:
      ret.brakePressed = False

    return ret, ret_sp

