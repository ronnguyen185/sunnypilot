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
    # Initialize controller params with car-specific limits
    self.params = CarControllerParams(CP)

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    # VF8 uses angle-based steering control
    # Get desired steering angle from actuators (in degrees)
    desired_angle = CC.actuators.steeringAngleDeg

    # Check if desired angle exceeds limit - if so, don't send angle command
    angle_limit = self.params.ANGLE_LIMITS.STEER_ANGLE_MAX
    angle_exceeds_limit = abs(desired_angle) > angle_limit if CC.latActive else False

    # Apply angle rate limiting using standard function
    # This function automatically sets angle to current steering angle when latActive is False
    # Rate limiting is speed-dependent and matches vinfast.h safety code limits
    apply_angle = apply_std_steer_angle_limits(
      desired_angle,
      self.apply_angle_last,
      CS.out.vEgoRaw,
      CS.out.steeringAngleDeg,
      CC.latActive,
      self.params.ANGLE_LIMITS
    )

    # Store for next iteration (rate limiting)
    self.apply_angle_last = apply_angle

    # Send steering control at 100Hz (every frame at 100Hz loop)
    # When angle exceeds limit, don't send angle command (set lat_active to False)
    effective_lat_active = CC.latActive and not angle_exceeds_limit
    can_sends.append(vinfastcan.create_steering_control(
      self.packer,
      self.CP,
      self.frame,
      apply_angle,
      effective_lat_active,
      angle_limit
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
        # OpenPilot longitudinal: send acceleration commands on chassis bus (bus 2)
        accel_cmd = CC.actuators.accel if CC.longActive else 0.0
        
        # Get acc_popup_feed for standstill request logic
        acc_popup_feed = getattr(CS, 'acc_popup_feed', 0)
        
        # When re-engaging (acc_popup_feed == 3), enforce our own acceleration logic
        # Only follow our acceleration until closest lead distance < 2m, then brake
        if acc_popup_feed == 3 and CC.longActive:
          # Get distance to closest lead vehicle (check both leadOne and leadTwo)
          lead_one_distance = CC_SP.leadOne.dRel if CC_SP.leadOne.status else float('inf')
          lead_two_distance = CC_SP.leadTwo.dRel if CC_SP.leadTwo.status else float('inf')
          # Find the closest lead vehicle
          lead_distance = min(lead_one_distance, lead_two_distance)
          
          if lead_distance < 2.0:
            # Lead vehicle too close: brake to maintain safe distance
            accel_cmd = -2.0
          else:
            # Re-engaging: enforce our own positive acceleration (ignore model completely)
            # Use minimum 0.2 m/s² to ensure car can move forward
            accel_cmd = 0.2
        # When starting from standstill (normal case), ensure accel is at least 0 to allow acceleration
        # Negative accel (like -2) prevents the car from accelerating from stop
        # BUT: Don't boost if model predicts stopping (longControlState == stopping) or if accel is negative
        # The model may want to hold negative accel to maintain stopping distance
        elif (CS.out.standstill and 
              CC.actuators.longControlState == car.CarControl.Actuators.LongControlState.starting and
              CC.actuators.longControlState != car.CarControl.Actuators.LongControlState.stopping and
              accel_cmd >= 0.0):
          # Starting from stop: ensure minimum positive accel to allow acceleration
          # Only boost if model is NOT predicting stopping and accel is not negative
          accel_cmd = max(0.0, accel_cmd)
        
        # Clip acceleration to car's limits
        accel_cmd = max(self.params.ACCEL_MIN, min(self.params.ACCEL_MAX, accel_cmd))
        can_sends.append(vinfastcan.create_acc_control(
            self.packer,
            self.CP,
            self.frame // 2,
            accel_cmd,
            CC.longActive,
            CS.out.standstill,
            acc_popup_feed
        ))
      else:
        # Stock longitudinal: enable/disable ACC, car handles speed control
        # When cancel is requested, disable ACC immediately
        # Otherwise, enable ACC when openpilot is engaged
        # Note: When longActive is False, car sends ADAS_ACC_Status on SCAM bus (bus 0)
        # which is read by carstate to determine cruise control state
        if CC.cruiseControl.cancel:
          # Cancel ACC - disable it
          acc_enabled = False
        else:
          # Enable ACC when openpilot is engaged
          acc_enabled = CC.enabled

        accel_cmd = 0.0  # Don't send acceleration commands - car's stock ACC handles speed control
        # Get acc_popup_feed for standstill request logic
        acc_popup_feed = getattr(CS, 'acc_popup_feed', 0)
        can_sends.append(vinfastcan.create_acc_control(
            self.packer,
            self.CP,
            self.frame // 2,
            accel_cmd,
            acc_enabled,
            CS.out.standstill,
            acc_popup_feed
        ))

      # Send IDB control message at 50Hz (always send to keep message alive)
      # Drive off request activates when:
      # - OpenPilot is engaged (CC.enabled)
      # - Car is at standstill (CS.out.standstill)
      # - Long control state is "starting" (model says not shouldStop = time to go)
      # OR when ACC popup feed = 3 (re-engage request - "Press gas pedal to re-engage the function")
      drive_off_request = 0
      acc_popup_feed = getattr(CS, 'acc_popup_feed', 0)
      
      # Debug logging (every 2 seconds at 50Hz) - only log when acc_popup_feed is 3
      if acc_popup_feed == 3:
        if self.frame % 100 == 0:
          from openpilot.common.swaglog import cloudlog
          cloudlog.info(f"IDB: acc_popup_feed={acc_popup_feed}, enabled={CC.enabled}, standstill={CS.out.standstill}, longState={CC.actuators.longControlState}, drive_off_request={drive_off_request}")
      
      # When ACC popup feed = 3, send IDB drive off request (re-engage) - requires CC.enabled
      if CC.enabled and acc_popup_feed == 3:
        drive_off_request = 1
        # Debug: log when drive off is triggered (every frame when triggered)
        from openpilot.common.swaglog import cloudlog
        cloudlog.info(f"IDB: TRIGGERED! acc_popup_feed={acc_popup_feed}, enabled={CC.enabled}, drive_off_request={drive_off_request}")
      # Normal drive off from standstill case
      elif (CC.enabled and
            CS.out.standstill and
            CC.actuators.longControlState == car.CarControl.Actuators.LongControlState.starting):
        drive_off_request = 1

      # Always send IDB message at 50Hz to keep it alive (even when drive_off_request = 0)
      # Send directly on chassis bus (bus 2) - safety code blocks IDB from SCAM->Chassis to prevent conflicts
      # but openpilot sends its own IDB messages directly on chassis bus
      can_sends.append(vinfastcan.create_idb_control(
        self.packer,
        self.frame // 2,
        drive_off_request,
        bus=CANBUS.chassis  # ADAS_IDB sent directly on Chassis bus (bus 2)
      ))

    new_actuators = CC.actuators.as_builder()
    # For angle control, steeringAngleDeg is already set by actuators
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1

    return new_actuators, can_sends
