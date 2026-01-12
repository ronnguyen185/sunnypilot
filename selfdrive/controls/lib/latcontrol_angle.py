import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

# TODO This is speed dependent
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees

# PI gains for VinFast angle control feedback (no derivative term to reduce oscillation)
# These correct for the gap between desired and actual steering angle
# Tuned to minimize steering error while reducing oscillation around center (±5 deg)
VINFAST_ANGLE_KP = 0.8 # Proportional gain - strong feedback
VINFAST_ANGLE_KI = 0.05  # Integral gain - eliminates steady-state error
VINFAST_ANGLE_KD = 0.0   # Derivative gain - set to 0 for PI controller (reduces oscillation)


class LatControlAngle(LatControl):
  def __init__(self, CP, CP_SP, CI):
    super().__init__(CP, CP_SP, CI)
    self.sat_check_min_speed = 5.
    self.use_steer_limited_by_safety = CP.brand == "tesla"
    # Increase saturation threshold for VinFast brand
    self.steer_angle_saturation_threshold = 20.0 if CP.brand == "vinfast" else STEER_ANGLE_SATURATION_THRESHOLD

      # Add PI controller for VinFast to close the gap between desired and actual steering
    if CP.brand == "vinfast":
      # PI outputs angle correction in degrees
      # Limit to prevent overcorrection while allowing strong feedback
      max_angle_correction = 8.0  # Reduced limit to reduce oscillation around center
      self.pid = PIDController(
        k_p=VINFAST_ANGLE_KP,
        k_i=VINFAST_ANGLE_KI,
        k_d=VINFAST_ANGLE_KD,  # Set to 0 for PI controller
        k_f=0.0,  # No feedforward in PID - handled separately
        pos_limit=max_angle_correction,
        neg_limit=-max_angle_correction,
        rate=100
      )
      # Track previous error for integrator unwinding logic
      self.prev_angle_error = 0.0
    else:
      self.pid = None
      self.prev_angle_error = 0.0

  def reset(self):
    """Reset the PI controller when control is disabled"""
    if self.pid is not None:
      self.pid.reset()
    self.prev_angle_error = 0.0
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
        angle_correction = self.pid.update(
          error=angle_error,
          error_rate=0.0,  # No derivative term for PI controller
          speed=CS.vEgo,
          feedforward=0.0,  # Feedforward is handled separately
          freeze_integrator=freeze_integrator
        )

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
