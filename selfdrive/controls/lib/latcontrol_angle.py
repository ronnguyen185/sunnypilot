import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

# TODO This is speed dependent
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees
# Single switch for A/B test:
# - True  -> VinFast PI controller ON
# - False -> Standard OP angle controller (feedforward only)
VINFAST_PI_ENABLED = True

# PI gains for VinFast angle control feedback (no derivative term to reduce oscillation)
# These correct for the gap between desired and actual steering angle
# Tuned to minimize steering error while reducing oscillation around center (±5 deg)
# ENV note:
# - This file is the "PI enabled" baseline for VinFast.
# - If you later add an environment toggle, use a name like VINFAST_ANGLE_PI=1
#   to explicitly enable this PI path at runtime.
VINFAST_ANGLE_KP_BP = [0.0, 3.5, 5.56, 8.33, 11.11, 55.56]  # m/s breakpoints
VINFAST_ANGLE_KP_VF8 = [0.6, 0.7, 0.9, 1.0, 1.1, 1.3]  # Proportional gain - VF8
VINFAST_ANGLE_KP_VF9 = [0.8, 0.9, 0.95, 1.0, 1.1, 1.3]  # Proportional gain - VF9
VINFAST_ANGLE_KI_VF8 = 0.05  # Integral gain - eliminates steady-state error
VINFAST_ANGLE_KI_VF9 = 0.035  # Lower I for VF9 to reduce center oscillation
VINFAST_ANGLE_KD = 0.0   # Derivative gain - set to 0 for PI controller (reduces oscillation)
# Damping for high-angle oscillation
HIGH_ANGLE_START_DEG = 50.0
HIGH_ANGLE_END_DEG = 470.0
HIGH_ANGLE_KP_SCALE_MIN = 0.45
MAX_PI_CORR_RATE_DEG_PER_CYCLE = 0.35
# Extra VF9 stabilization around center/small angle (anti-hunting)
VF9_SMALL_ANGLE_DES_DEG = 12.0
VF9_SMALL_ANGLE_ERR_DEADBAND_DEG = 0.30
VF9_SMALL_ANGLE_MAX_CORR_RATE_DEG_PER_CYCLE = 0.12
VF9_SMALL_ANGLE_I_BLEED = 0.94
VF9_SMALL_CORR_ZERO_DEG = 0.06
VF9_SMALL_CENTER_DECAY = 0.90


class LatControlAngle(LatControl):
  def __init__(self, CP, CP_SP, CI):
    super().__init__(CP, CP_SP, CI)
    self.sat_check_min_speed = 5.
    self.use_steer_limited_by_safety = CP.brand == "tesla"
    # Increase saturation threshold for VinFast brand
    self.steer_angle_saturation_threshold = 20.0 if CP.brand == "vinfast" else STEER_ANGLE_SATURATION_THRESHOLD

    # Add PI controller for VinFast to close the gap between desired and actual steering
    if CP.brand == "vinfast" and VINFAST_PI_ENABLED:
      # Store car fingerprint for speed-dependent KP adjustment
      self.vinfast_car_fingerprint = getattr(CP, "carFingerprint", "")
      
      # Model-specific tuning with speed-dependent KP for VinFast models
      if self.vinfast_car_fingerprint == "VINFAST_VF8":
        vinfast_kp = (VINFAST_ANGLE_KP_BP, VINFAST_ANGLE_KP_VF8)
        vinfast_ki = VINFAST_ANGLE_KI_VF8
      elif self.vinfast_car_fingerprint == "VINFAST_VF9":
        vinfast_kp = (VINFAST_ANGLE_KP_BP, VINFAST_ANGLE_KP_VF9)
        vinfast_ki = VINFAST_ANGLE_KI_VF9
      else:
        vinfast_kp = (VINFAST_ANGLE_KP_BP, VINFAST_ANGLE_KP_VF8)
        vinfast_ki = VINFAST_ANGLE_KI_VF8

      # PI outputs angle correction in degrees
      # Limit to prevent overcorrection while allowing strong feedback
      max_angle_correction = 15.0  # Reduced limit to reduce oscillation around center
      self.pid = PIDController(
        k_p=vinfast_kp,
        k_i=vinfast_ki,
        k_d=VINFAST_ANGLE_KD,  # Set to 0 for PI controller
        k_f=0.0,  # No feedforward in PID - handled separately
        pos_limit=max_angle_correction,
        neg_limit=-max_angle_correction,
        rate=100
      )
      # Track previous error for integrator unwinding logic
      self.prev_angle_error = 0.0
      self.prev_angle_correction = 0.0
    else:
      self.pid = None
      self.prev_angle_error = 0.0
      self.prev_angle_correction = 0.0
      self.vinfast_car_fingerprint = None

  def reset(self):
    """Reset the PI controller when control is disabled"""
    if self.pid is not None:
      self.pid.reset()
    self.prev_angle_error = 0.0
    self.prev_angle_correction = 0.0
    super().reset()

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited):
    angle_log = log.ControlsState.LateralAngleState.new_message()

    if not active:
      angle_log.active = False
      angle_steers_des = float(CS.steeringAngleDeg)
      if self.pid is not None:
        self.pid.reset()
    else:
      angle_log.active = True
      # Calculate desired angle from curvature (feedforward)
      angle_steers_des_ff = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
      angle_steers_des_ff += params.angleOffsetDeg

      # For VinFast: add PI feedback to close the gap (no derivative term to reduce oscillation)
      if self.pid is not None:
        # Calculate error between desired and actual steering angle
        angle_error = angle_steers_des_ff - CS.steeringAngleDeg
        is_vf9 = self.vinfast_car_fingerprint == "VINFAST_VF9"

        # VF9 center stabilization: suppress tiny corrective chatter around straight driving.
        if is_vf9 and abs(angle_steers_des_ff) < VF9_SMALL_ANGLE_DES_DEG:
          if abs(angle_error) < VF9_SMALL_ANGLE_ERR_DEADBAND_DEG:
            angle_error = 0.0
          # Bleed integral slightly in the tiny-error region to avoid sign-flip hunting.
          if hasattr(self.pid, 'i') and abs(angle_error) < (VF9_SMALL_ANGLE_ERR_DEADBAND_DEG * 1.5):
            self.pid.i *= VF9_SMALL_ANGLE_I_BLEED

        # Help integrator unwind faster when error changes sign (returning to center)
        # When error crosses zero, reduce integrator to speed up return to center
        # This reduces oscillation around center (±5 deg)
        if self.prev_angle_error * angle_error < 0:
          # Error changed sign (crossed zero) - reduce integrator to help unwind faster
          if hasattr(self.pid, 'i'):
            # Reduce integrator by 60% when error crosses zero to speed up return
            self.pid.i *= 0.4

        self.prev_angle_error = angle_error

        # Freeze integrator if steering is limited or driver is steering
        freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5

        # PI correction (in degrees) - no derivative term, error_rate=0 for PI controller
        raw_angle_correction = self.pid.update(
          error=angle_error,
          error_rate=0.0,  # No derivative term for PI controller
          speed=CS.vEgo,
          feedforward=0.0,  # Feedforward is handled separately
          freeze_integrator=freeze_integrator
        )

        # Reduce feedback aggressiveness as |desired angle| grows to prevent
        # high-angle oscillation while keeping near-center response.
        abs_des_ff = abs(angle_steers_des_ff)
        if abs_des_ff <= HIGH_ANGLE_START_DEG:
          kp_scale = 1.0
        elif abs_des_ff >= HIGH_ANGLE_END_DEG:
          kp_scale = HIGH_ANGLE_KP_SCALE_MIN
        else:
          t = (abs_des_ff - HIGH_ANGLE_START_DEG) / (HIGH_ANGLE_END_DEG - HIGH_ANGLE_START_DEG)
          kp_scale = 1.0 - t * (1.0 - HIGH_ANGLE_KP_SCALE_MIN)

        scaled_correction = raw_angle_correction * kp_scale
        if is_vf9 and abs_des_ff < VF9_SMALL_ANGLE_DES_DEG:
          # Near center: zero tiny corrections to suppress left-right dither.
          if abs(scaled_correction) < VF9_SMALL_CORR_ZERO_DEG and abs(angle_error) < (VF9_SMALL_ANGLE_ERR_DEADBAND_DEG * 1.2):
            scaled_correction = 0.0

        # Rate-limit correction change to suppress chatter at large angles.
        corr_rate_limit = MAX_PI_CORR_RATE_DEG_PER_CYCLE
        if is_vf9 and abs_des_ff < VF9_SMALL_ANGLE_DES_DEG:
          corr_rate_limit = min(corr_rate_limit, VF9_SMALL_ANGLE_MAX_CORR_RATE_DEG_PER_CYCLE)
        delta_corr = scaled_correction - self.prev_angle_correction
        delta_corr = max(-corr_rate_limit, min(corr_rate_limit, delta_corr))
        angle_correction = self.prev_angle_correction + delta_corr
        if is_vf9 and abs_des_ff < VF9_SMALL_ANGLE_DES_DEG and abs(angle_error) < (VF9_SMALL_ANGLE_ERR_DEADBAND_DEG * 1.2):
          # Bleed residual correction near center so old correction does not keep
          # exciting oscillation after the path straightens.
          angle_correction *= VF9_SMALL_CENTER_DECAY
        self.prev_angle_correction = angle_correction

        # Final desired angle = feedforward + PI correction
        angle_steers_des = float(angle_steers_des_ff + angle_correction)
      else:
        # Original behavior for other cars
        angle_steers_des = float(angle_steers_des_ff)

    if self.use_steer_limited_by_safety:
      # these cars' carcontrollers calculate max lateral accel and jerk, so we can rely on carOutput for saturation
      angle_control_saturated = steer_limited_by_safety
    else:
      # for cars which use a method of limiting torque such as a torque signal (Nissan and Toyota)
      # or relying on EPS (Ford Q3), carOutput does not capture maxing out torque  # TODO: this can be improved
      angle_control_saturated = abs(angle_steers_des - CS.steeringAngleDeg) > self.steer_angle_saturation_threshold
    angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, False, curvature_limited))
    angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    angle_log.steeringAngleDesiredDeg = float(angle_steers_des)
    return 0, float(angle_steers_des), angle_log
