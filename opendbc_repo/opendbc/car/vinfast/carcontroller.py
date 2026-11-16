import math
import numpy as np
from cereal import car
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import AngleSteeringLimits, apply_std_steer_angle_limits
from opendbc.car.vinfast import vinfastcan
from opendbc.car.vinfast.values import CAR, CarControllerParams, CANBUS, is_vf6_safety_platform

VisualAlert = structs.CarControl.HUDControl.VisualAlert

# Re-engage lead classes (internal classification for TTC behavior tuning)
LEAD_CLASS_NONE = 0
LEAD_CLASS_RECEDING = 1
LEAD_CLASS_STEADY = 2
LEAD_CLASS_CLOSING = 3
LEAD_CLASS_CRITICAL = 4


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
    # Reengage state tracking (latched on popup feed)
    self.reengage_active = False
    self.reengage_accel_state = 0.2
    self.reengage_lead_class = LEAD_CLASS_NONE
    self.reengage_boost_frames = 0
    self.reengage_standstill_armed = False
    self.reengage_popup_prev = False
    self.prev_long_engaged = False
    self.prev_scam_acc_mode = None
    self.final_accel_cmd = 0.0  # Track final acceleration command (what we send via CAN)

  def _steer_angle_max(self, v_ego: float) -> float:
    lookup = getattr(self.params, 'ANGLE_MAX_LOOKUP', None)
    if lookup is not None:
      return float(np.interp(v_ego, lookup[0], lookup[1]))
    return self.params.ANGLE_LIMITS.STEER_ANGLE_MAX

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    # VF8 uses angle-based steering control
    # Get desired steering angle from actuators (in degrees)
    desired_angle = CC.actuators.steeringAngleDeg

    angle_limit = self._steer_angle_max(CS.out.vEgoRaw)
    # Clip to speed-dependent max (VF6) or static STEER_ANGLE_MAX (VF8/VF9), then rate-limit.
    # Do not drop lat_active when planner requests past limit — that sent AOLReq=0 and released EPS.
    if CC.latActive:
      desired_angle = float(np.clip(desired_angle, -angle_limit, angle_limit))

    # VF6: pass speed-dependent max into rate limiter (OEM current table, not static 180°)
    angle_limits = self.params.ANGLE_LIMITS
    if getattr(self.params, 'ANGLE_MAX_LOOKUP', None) is not None:
      angle_limits = AngleSteeringLimits(
        angle_limit,
        self.params.ANGLE_LIMITS.ANGLE_RATE_LIMIT_UP,
        self.params.ANGLE_LIMITS.ANGLE_RATE_LIMIT_DOWN,
      )

    apply_angle = apply_std_steer_angle_limits(
      desired_angle,
      self.apply_angle_last,
      CS.out.vEgoRaw,
      CS.out.steeringAngleDeg,
      CC.latActive,
      angle_limits,
    )
    apply_angle = float(np.clip(apply_angle, -angle_limit, angle_limit))

    # Store for next iteration (rate limiting)
    self.apply_angle_last = apply_angle

    # Send steering control at 100Hz (every frame at 100Hz loop)
    effective_lat_active = CC.latActive
    can_sends.append(vinfastcan.create_steering_control(
      self.packer,
      self.CP,
      self.frame,
      apply_angle,
      effective_lat_active
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
      vf6_combined_long = is_vf6_safety_platform(self.CP.carFingerprint)
      brake_pressed = getattr(CS.out, 'brakePressed', False) if vf6_combined_long else False
      scam_acc_mode = getattr(CS, 'scam_acc_mode', None) if vf6_combined_long else None
      scam_accel_cmd = getattr(CS, 'scam_accel_cmd', 0.0) if vf6_combined_long else 0.0
      # Hybrid: Standby→Active edge on SCAM (working.py --vinfast)
      transition_flag = (
        vf6_combined_long and self.prev_scam_acc_mode == 3 and scam_acc_mode == 4
      )
      long_engaged = (
        CC.longActive and not brake_pressed and scam_acc_mode == 4
        if vf6_combined_long else CC.longActive
      )

      if self.CP.openpilotLongitudinalControl:
        # OpenPilot longitudinal: send acceleration commands on chassis bus
        # Get acc_popup_feed for standstill request logic FIRST
        acc_popup_feed = getattr(CS, 'acc_popup_feed', 0)
        
        # Get distance to closest lead vehicle (check both leadOne and leadTwo)
        lead_one_distance = CC_SP.leadOne.dRel if CC_SP.leadOne.status else float('inf')
        lead_two_distance = CC_SP.leadTwo.dRel if CC_SP.leadTwo.status else float('inf')
        lead_distance = min(lead_one_distance, lead_two_distance)

        # Reengage latch:
        # - If popup feed == 3 while longActive => latch reengage_active True
        # - Stays active until: road clear, lead moving away, or speed-based handoff
        popup_reengage_req = (acc_popup_feed == 3) and CC.longActive
        popup_reengage_trigger = popup_reengage_req and (not self.reengage_popup_prev)
        was_reengage_active = self.reengage_active
        if popup_reengage_req:
          self.reengage_active = True
        # Latch a short launch boost window whenever re-engage becomes active, so
        # every re-engage event gets the same instant 0.6 m/s^2 kick.
        if popup_reengage_trigger or (self.reengage_active and (not was_reengage_active)):
          self.reengage_boost_frames = 60  # ~1.2s at 50Hz
        if self.reengage_active and (not CC.longActive):
          # Disengaged longitudinal: clear latch
          self.reengage_active = False
          self.reengage_boost_frames = 0
          self.reengage_standstill_armed = False

        # ── Lead analysis ──────────────────────────────────────────────
        lead_vRel = 0.0
        closing_speed = 0.0
        lead_ttc = float('inf')
        if lead_distance != float('inf'):
          lead_obj = None
          if CC_SP.leadOne.status and (lead_one_distance <= lead_two_distance):
            lead_obj = CC_SP.leadOne
          elif CC_SP.leadTwo.status:
            lead_obj = CC_SP.leadTwo
          if lead_obj is not None:
            lead_vRel = float(getattr(lead_obj, 'vRel', 0.0))
            closing_speed = max(0.0, -lead_vRel)
            if closing_speed > 0.1:
              lead_ttc = lead_distance / closing_speed

        road_clear = (lead_distance == float('inf'))
        lead_moving_away = (not road_clear) and (lead_vRel > 0.5)
        is_creeping = (not road_clear) and (lead_distance < 10.0) and (not lead_moving_away)
        is_fast_resume = road_clear or lead_moving_away
        REENGAGE_HANDOFF_SPEED = (5.0 / 3.6) if is_creeping else ((10.0 / 3.6) if is_fast_resume else (6.0 / 3.6))

        # ── Re-engage exit condition ──────────────────────────────────
        # Only hand off to model once ego speed is high enough for the
        # model to produce meaningful acceleration on its own.
        # Use a lower threshold in creeping traffic to keep re-engage
        # conservative while still guaranteeing handoff.
        if self.reengage_active and (not popup_reengage_trigger) and CS.out.vEgo > REENGAGE_HANDOFF_SPEED:
          self.reengage_active = False
          self.reengage_standstill_armed = False
        
        # Capture model's acceleration BEFORE our override for logging/debugging
        model_accel_before_override = CC.actuators.accel if CC.longActive else 0.0
        
        # ── Re-engage acceleration override ──────────────────────────────
        # Creeping (close/slow lead that isn't moving away): conservative
        # accel + speed cap at 5 km/h.
        # Not creeping (road clear or lead far/moving away): stronger accel
        # + speed cap at 6 km/h before handoff.
        if self.reengage_active and CC.longActive:
          # Re-arm launch boost whenever we are back at standstill in re-engage.
          # Then retrigger boost on the next standstill -> move-off transition.
          if CS.out.standstill:
            self.reengage_standstill_armed = True
          elif self.reengage_standstill_armed and CS.out.vEgo < 1.5:
            self.reengage_boost_frames = 60  # ~1.2s at 50Hz
            self.reengage_standstill_armed = False

          # Launch boost is capped at 0.6 m/s^2 for all re-engage starts.
          REENGAGE_LAUNCH_ACCEL = 0.6
          launch_boost_active = (CS.out.standstill or (CS.out.vEgo < 0.3) or
                                 (self.reengage_boost_frames > 0))
          launch_accel = REENGAGE_LAUNCH_ACCEL
          # Keep creep-mode braking softer to reduce harsh stop-go oscillations.
          REENGAGE_STOP_ACCEL = -1.0 if is_creeping else -1.5

          if is_creeping:
            resume_accel = launch_accel if launch_boost_active else 0.3
          else:
            resume_accel = launch_accel if launch_boost_active else 0.5
          
          # Distance guardrail: never accelerate hard when the lead is near.
          dist_full_brake = 4.5
          dist_resume = 8.0 if is_creeping else 6.0
          if road_clear:
            accel_dist = resume_accel
          elif lead_distance <= dist_full_brake:
            accel_dist = REENGAGE_STOP_ACCEL
          elif lead_distance >= dist_resume:
            accel_dist = resume_accel
          else:
            u_dist = (lead_distance - dist_full_brake) / max(0.1, dist_resume - dist_full_brake)
            accel_dist = REENGAGE_STOP_ACCEL + u_dist * (resume_accel - REENGAGE_STOP_ACCEL)

          # TTC shaping bands (requested):
          # ttc < 0.9s       -> strong brake
          # 0.9..2.0s        -> proportional decel
          # 2.0..4.0s        -> near-zero / gentle
          # > 4.0s           -> allow resume accel
          TTC_HARD_BRAKE = 0.9
          TTC_DECEL_END = 2.0
          TTC_GENTLE_END = 4.0
          TTC_STRONG_BRAKE = -1.2 if is_creeping else -2.0
          TTC_DECEL_AT_2S = -0.20 if is_creeping else -0.35
          TTC_GENTLE_AT_4S = 0.15 if is_creeping else 0.20

          # Internal lead class for readability/debug tuning.
          if road_clear:
            self.reengage_lead_class = LEAD_CLASS_NONE
          elif lead_moving_away:
            self.reengage_lead_class = LEAD_CLASS_RECEDING
          elif closing_speed <= 0.1:
            self.reengage_lead_class = LEAD_CLASS_STEADY
          elif lead_ttc < TTC_HARD_BRAKE:
            self.reengage_lead_class = LEAD_CLASS_CRITICAL
          else:
            self.reengage_lead_class = LEAD_CLASS_CLOSING

          # TTC-based accel target by band.
          if road_clear or not math.isfinite(lead_ttc):
            accel_ttc = resume_accel
          elif lead_ttc < TTC_HARD_BRAKE:
            accel_ttc = TTC_STRONG_BRAKE
          elif lead_ttc <= TTC_DECEL_END:
            u_ttc = (lead_ttc - TTC_HARD_BRAKE) / max(0.1, (TTC_DECEL_END - TTC_HARD_BRAKE))
            accel_ttc = TTC_STRONG_BRAKE + u_ttc * (TTC_DECEL_AT_2S - TTC_STRONG_BRAKE)
          elif lead_ttc <= TTC_GENTLE_END:
            u_ttc = (lead_ttc - TTC_DECEL_END) / max(0.1, (TTC_GENTLE_END - TTC_DECEL_END))
            accel_ttc = TTC_DECEL_AT_2S + u_ttc * (TTC_GENTLE_AT_4S - TTC_DECEL_AT_2S)
          else:
            accel_ttc = resume_accel

          # Conservative fusion: apply whichever asks for less acceleration.
          accel_cmd = min(accel_dist, accel_ttc)
          if lead_moving_away and lead_distance > 6.0:
            accel_cmd = max(accel_cmd, 0.4 if CS.out.standstill else 0.2)

          if accel_cmd > 0.0:
            REENGAGE_MAX_SPEED_MS = (5.0 / 3.6) if is_creeping else ((10.0 / 3.6) if is_fast_resume else (6.0 / 3.6))
            REENGAGE_TAPER_START = REENGAGE_MAX_SPEED_MS * 0.7
            if CS.out.vEgo >= REENGAGE_MAX_SPEED_MS:
              accel_cmd = 0.0
            elif CS.out.vEgo >= REENGAGE_TAPER_START:
              taper = (REENGAGE_MAX_SPEED_MS - CS.out.vEgo) / (REENGAGE_MAX_SPEED_MS - REENGAGE_TAPER_START)
              accel_cmd = accel_cmd * max(0.0, taper)

          self.reengage_accel_state = accel_cmd
          if self.reengage_boost_frames > 0:
            self.reengage_boost_frames -= 1
        else:
          # Not in reengage mode: use model's acceleration
          accel_cmd = CC.actuators.accel if CC.longActive else 0.0
          self.reengage_lead_class = LEAD_CLASS_NONE
          self.reengage_boost_frames = 0
          self.reengage_standstill_armed = False
        
        # When starting from standstill (normal case, not in reengage), ensure accel is at least 0 to allow acceleration
        # Negative accel (like -2) prevents the car from accelerating from stop
        # BUT: Don't boost if model predicts stopping (longControlState == stopping) or if accel is negative
        # The model may want to hold negative accel to maintain stopping distance
        # Only apply this logic when NOT in reengage mode (reengage has its own logic)
        # CRITICAL: Check self.reengage_active, not just acc_popup_feed, because acc_popup_feed turns off quickly
        if (not self.reengage_active and
            acc_popup_feed != 3 and
            CS.out.standstill and 
            CC.actuators.longControlState == car.CarControl.Actuators.LongControlState.starting and
            CC.actuators.longControlState != car.CarControl.Actuators.LongControlState.stopping and
            accel_cmd >= 0.0):
          # Starting from stop: ensure minimum positive accel to allow acceleration
          # Only boost if model is NOT predicting stopping and accel is not negative
          accel_cmd = max(0.0, accel_cmd)
        
        # Clip acceleration to car's limits
        # CRITICAL: During reengage, ensure our override is NOT modified by any other logic
        # The model's acceleration should be completely ignored during reengage
        if self.reengage_active and CC.longActive:
          # During reengage, our accel_cmd is already set correctly above
          # Just clip it to car limits, but don't let any other logic modify it
          # This ensures the model's CC.actuators.accel (-2.0) is completely ignored
          accel_cmd = max(self.params.ACCEL_MIN, min(self.params.ACCEL_MAX, accel_cmd))
          # Verify our override is being used (should be positive when lead is far, not -2.0)
          if accel_cmd != self.reengage_accel_state:
            # Something modified our override - restore it
            accel_cmd = max(self.params.ACCEL_MIN, min(self.params.ACCEL_MAX, self.reengage_accel_state))
        else:
          # Normal mode: clip to car limits
          accel_cmd = max(self.params.ACCEL_MIN, min(self.params.ACCEL_MAX, accel_cmd))
        # Store final acceleration command for actuators override
        self.final_accel_cmd = accel_cmd

        # CRITICAL: Use our accel_cmd (not CC.actuators.accel) when sending CAN message
        acc_kwargs = {}
        if vf6_combined_long:
          acc_kwargs = dict(
            brake_pressed=brake_pressed,
            transition_flag=transition_flag,
            scam_acc_mode=scam_acc_mode,
          )
        can_sends.append(vinfastcan.create_acc_control(
            self.packer,
            self.CP,
            self.frame // 2,
            accel_cmd,
            CC.longActive,
            CS.out.standstill,
            acc_popup_feed,
            self.reengage_active,
            **acc_kwargs,
        ))

        if vf6_combined_long:
          idb_info = getattr(CS, 'idb_info_flag', None)
          if idb_info:
            can_sends.append(vinfastcan.create_idb_info_flag(
              self.packer,
              self.frame // 2,
              idb_info,
              scam_acc_mode=scam_acc_mode,
              scam_accel_cmd=scam_accel_cmd,
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
            acc_popup_feed,
            self.reengage_active  # Pass reengage_active to ensure standstill_req is unset during reengage
        ))

      # Send IDB control message at 50Hz (always send to keep message alive)
      drive_off_request = 0
      acc_popup_feed = getattr(CS, 'acc_popup_feed', 0)

      if self.CP.openpilotLongitudinalControl:
        if CC.enabled and acc_popup_feed == 3:
          drive_off_request = 1
        elif vf6_combined_long:
          accel_for_drive = self.final_accel_cmd if CC.longActive else 0.0
          if transition_flag:
            drive_off_request = 0
          elif long_engaged and accel_for_drive > 0:
            drive_off_request = 1
        elif (CC.enabled and
              CS.out.standstill and
              CC.actuators.longControlState == car.CarControl.Actuators.LongControlState.starting):
          drive_off_request = 1
      else:
        if CC.enabled and acc_popup_feed == 3:
          drive_off_request = 1
        elif (CC.enabled and
              CS.out.standstill and
              CC.actuators.longControlState == car.CarControl.Actuators.LongControlState.starting):
          drive_off_request = 1

      # ADAS_IDB on chassis @ 50Hz (vinfastVf6 blocks SCAM ADAS_IDB cam→chassis)
      can_sends.append(vinfastcan.create_idb_control(
        self.packer,
        self.frame // 2,
        drive_off_request,
        bus=CANBUS.chassis,
      ))
      self.reengage_popup_prev = (acc_popup_feed == 3)
      if self.CP.openpilotLongitudinalControl and vf6_combined_long:
        self.prev_long_engaged = long_engaged
        if scam_acc_mode is not None:
          self.prev_scam_acc_mode = scam_acc_mode
      else:
        self.prev_long_engaged = False
        self.prev_scam_acc_mode = None

    new_actuators = CC.actuators.as_builder()
    # For angle control, steeringAngleDeg is already set by actuators
    new_actuators.steeringAngleDeg = apply_angle
    
    # During reengage, override actuators.accel to match our override
    # This ensures the actuators message reflects what we're actually sending to the car
    # and prevents any downstream code from using the model's acceleration
    if self.reengage_active and CC.longActive and self.CP.openpilotLongitudinalControl:
      # Override actuators.accel with our reengage acceleration (always kept current)
      # This ensures the actuators message matches what we're actually sending via CAN
      # and prevents downstream code from seeing the model's conflicting acceleration
      new_actuators.accel = self.reengage_accel_state
      # Also override longControlState to "starting" to prevent model's "stopping" state from interfering
      # This is informational only - the actual CAN command uses our accel_cmd, but this prevents
      # any downstream code from seeing the model's "stopping" state and potentially using it
      new_actuators.longControlState = car.CarControl.Actuators.LongControlState.starting
    new_actuators.reengageStatus = self.reengage_active
    
    self.frame += 1

    return new_actuators, can_sends