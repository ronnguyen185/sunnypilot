import math
import numpy as np

from cereal import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.cruise_ext import VCruiseHelperSP


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 80  # Default, can be adjusted via buttons
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper(VCruiseHelperSP):
  def __init__(self, CP, CP_SP):
    VCruiseHelperSP.__init__(self, CP, CP_SP)
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}
    self.params = Params()
    # Track if we're in adjustment mode for initial experimental speed
    self.adjusting_vmax_init = False
    self.vmax_init_adjust_timer = 0
    # Store adjustable initial experimental mode speed (defaults to 105)
    self.v_initial_experimental_mode = V_CRUISE_INITIAL_EXPERIMENTAL_MODE

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph

    # Allow adjusting VMaxInitialExperimentalMode when cruise is not initialized
    self.update_vmax_init_experimental(CS, enabled, is_metric)

    self.get_minimum_set_speed(is_metric)

    if CS.cruiseState.available:
      _enabled = self.update_enabled_state(CS, enabled)
      if not self.CP.pcmCruise or (not self.CP_SP.pcmCruiseSpeed and _enabled):
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, _enabled, is_metric)
        self.update_speed_limit_assist_v_cruise_non_pcm()
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self.update_button_timers(CS, enabled)
      else:
        # When pcmCruiseSpeed is enabled, use car's speed but allow button adjustments
        # Start with car's speed as base
        car_speed_kph = CS.cruiseState.speed * CV.MS_TO_KPH if CS.cruiseState.speed > 0 else V_CRUISE_UNSET

        # Process button events to allow user adjustments
        self.update_button_timers(CS, enabled)
        button_adjustment = 0.0

        if enabled and CS.buttonEvents:
          long_press = False
          button_type = None
          v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

          for b in CS.buttonEvents:
            if b.type.raw in self.button_timers and not b.pressed:
              if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
                continue  # end long press
              button_type = b.type.raw
              break
          else:
            for k, timer in self.button_timers.items():
              if timer and timer % CRUISE_LONG_PRESS == 0:
                button_type = k
                long_press = True
                break

          if button_type is not None:
            # Don't adjust speed when pressing resume to exit standstill
            cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
            if not (button_type == ButtonType.accelCruise and cruise_standstill):
              # Don't adjust speed if we've enabled since the button was depressed
              if self.button_change_states[button_type]["enabled"]:
                # Speed Limit Assist check
                if not self.update_speed_limit_assist_pre_active_confirmed(button_type):
                  long_press, v_cruise_delta = VCruiseHelperSP.update_v_cruise_delta(self, long_press, v_cruise_delta)
                  button_adjustment = v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

        # Apply button adjustment if any
        if button_adjustment != 0.0 and car_speed_kph != V_CRUISE_UNSET:
          # Adjust from car's speed
          self.v_cruise_kph = car_speed_kph + button_adjustment
          self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), self.v_cruise_min, V_CRUISE_MAX)
        elif car_speed_kph != V_CRUISE_UNSET:
          # No button press, use car's speed
          self.v_cruise_kph = car_speed_kph
        else:
          # Car speed is invalid
          if CS.cruiseState.speed == 0:
            self.v_cruise_kph = V_CRUISE_UNSET
          elif CS.cruiseState.speed == -1:
            self.v_cruise_kph = -1
          else:
            self.v_cruise_kph = V_CRUISE_UNSET

        # Set cluster speed (for UI display)
        if self.v_cruise_kph != V_CRUISE_UNSET and self.v_cruise_kph != -1:
          self.v_cruise_cluster_kph = self.v_cruise_kph
        elif CS.cruiseState.speedCluster > 0:
          # Fallback to car's cluster speed if available
          self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        elif CS.cruiseState.speed == 0:
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_cluster_kph = -1
        else:
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    # Allow setting v max even when not engaged
    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Only allow speed changes when gear is in Drive (D)
    if CS.gearShifter != car.CarState.GearShifter.drive:
      return

    # Don't adjust speed when pressing resume to exit standstill (only when enabled)
    if enabled:
      cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Allow setting v max even when not enabled - remove the enabled check
    # When enabled, still check button state to prevent issues with rising edge enables
    if enabled and not self.button_change_states[button_type]["enabled"]:
      return

    # Speed Limit Assist for Non PCM long cars.
    # True: Disallow set speed changes when user confirmed the target set speed during preActive state
    # False: Allow set speed changes as SLA is not requesting user confirmation
    if self.update_speed_limit_assist_pre_active_confirmed(button_type):
      return

    long_press, v_cruise_delta = VCruiseHelperSP.update_v_cruise_delta(self, long_press, v_cruise_delta)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), self.v_cruise_min, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def get_v_initial_experimental_mode(self) -> int:
    """Get the initial experimental mode speed, which can be adjusted via buttons."""
    return self.v_initial_experimental_mode

  def update_vmax_init_experimental(self, CS, enabled, is_metric) -> bool:
    """
    Allow adjusting VMaxInitialExperimentalMode using buttons when cruise is not initialized.
    Returns True if adjustment was made.
    """
    # Only allow adjustment when cruise is not initialized and openpilot is enabled
    if self.v_cruise_initialized or not enabled:
      self.adjusting_vmax_init = False
      self.vmax_init_adjust_timer = 0
      return False

    # Check for button presses to enter adjustment mode or adjust value
    button_pressed = False
    button_type = None
    long_press = False

    for b in CS.buttonEvents:
      if b.type in (ButtonType.accelCruise, ButtonType.decelCruise):
        if b.pressed:
          # Enter adjustment mode on button press
          self.adjusting_vmax_init = True
          self.vmax_init_adjust_timer = 30  # Keep in adjustment mode for 3 seconds (30 frames at 10Hz)
          button_pressed = True
        elif not b.pressed and self.adjusting_vmax_init:
          # Adjust value on button release
          button_type = b.type
          button_pressed = True
          break

    # Decrement timer
    if self.vmax_init_adjust_timer > 0:
      self.vmax_init_adjust_timer -= 1
    else:
      self.adjusting_vmax_init = False

    # Adjust value if button was released
    if button_type is not None and self.adjusting_vmax_init:
      current_vmax = self.get_v_initial_experimental_mode()
      v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

      # Check for long press (adjust by 5)
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          long_press = True
          v_cruise_delta *= 5
          break

      if button_type == ButtonType.accelCruise:
        new_vmax = current_vmax + v_cruise_delta
      else:  # ButtonType.decelCruise
        new_vmax = current_vmax - v_cruise_delta

      # Clip to valid range
      new_vmax = int(round(np.clip(new_vmax, V_CRUISE_MIN, V_CRUISE_MAX)))

      # Store adjusted value (persists for this session)
      self.v_initial_experimental_mode = new_vmax
      return True

    return False

  def initialize_v_cruise(self, CS, experimental_mode: bool, dynamic_experimental_control: bool) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial_experimental_mode = experimental_mode and not dynamic_experimental_control
    # Use adjustable initial experimental mode speed
    initial = self.get_v_initial_experimental_mode() if initial_experimental_mode else V_CRUISE_INITIAL

    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      # Prefer last cruise speed if valid, otherwise use initial value
      if self.v_cruise_kph_last > 0 and V_CRUISE_MIN <= self.v_cruise_kph_last <= V_CRUISE_MAX:
        self.v_cruise_kph = int(round(self.v_cruise_kph_last))
      else:
        self.v_cruise_kph = int(round(np.clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph
