from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.vinfast.carcontroller import CarController
from opendbc.car.vinfast.carstate import CarState
from opendbc.car.vinfast.radar_interface import RadarInterface
from opendbc.car.vinfast.values import CAR, CANBUS
from openpilot.common.params import Params


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface  # Re-enabled for diagnostics

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "vinfast"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.vinfast)]

    # Angle-based lateral control setup (VF8 uses angle controller)
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.05  # Reduced from 0.1 to minimize lag and improve error correction response
    # Lower delay reduces lag between desired and actual angle, allowing PID to respond faster
    ret.steerLimitTimer = 1.0
    # Note: lateralTuning is not initialized for angle-based control
    # LatControlAngle doesn't use lateralTuning parameters
    # Adjust centerToFront to improve steering response
    # Similar vehicles use 0.44-0.45, but VinFast may need higher value for better response
    # Higher centerToFront = more front weight bias = more responsive steering
    ret.centerToFront = ret.wheelbase * 0.4  # Increased from 0.42 to improve response

    # Longitudinal control setup
    # Default to stock longitudinal (car's ACC) - user can enable openpilot longitudinal via alpha_long
    ret.radarUnavailable = False  # Disable radar interface (set to False to enable)
    ret.alphaLongitudinalAvailable = True  # Enable experimental longitudinal control toggle

    # Check for temporary override to use stock longitudinal even when alpha_long is enabled
    # This allows switching to stock ACC without disabling AlphaLongitudinalEnabled
    # Use try/except since this parameter is not in params_keys.h (temporary override)
    params = Params()
    try:
      use_stock_long = params.get("VinFastUseStockLongitudinal") == b"1"
    except Exception:
      use_stock_long = False

    # Set openpilotLongitudinalControl based on alpha_long, but allow override to stock
    ret.openpilotLongitudinalControl = alpha_long and not use_stock_long
    ret.pcmCruise = not ret.openpilotLongitudinalControl  # True = use stock ACC, False = openpilot controls acceleration

    # Set safety parameter flag for longitudinal control only if actually using openpilot longitudinal
    # Bit 0: openpilot longitudinal control enabled
    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= 1  # Set bit 0 to indicate longitudinal control
    ret.startingState = True
    ret.vEgoStarting = 0.1
    ret.startAccel = 1.0
    ret.longitudinalActuatorDelay = 0.5

    # Car specific configuration
    ret.dashcamOnly = False  # TODO: set to True if not fully tested

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    """Sunnypilot-specific parameters for VinFast."""
    # For now, use default sunnypilot parameters
    # Add VinFast-specific sunnypilot flags or parameters here if needed in the future
    return ret

  @staticmethod
  def init(CP: structs.CarParams, CP_SP: structs.CarParamsSP, can_recv, can_send):
    """
    Initialize car interface.
    Configures CAN-FD for bus 1 (radar) after a 10 second delay.
    This is called after pandad sets safety mode, so we wait 10 seconds
    then try once to configure CAN-FD, then stop.
    """
    import time
    import threading

    def configure_bus1():
      """Configure bus 1 for CAN-FD. Returns True if successful."""
      try:
        from panda import Panda
        panda = Panda()

        # Enable bus 1 for radar (required for CAN-FD to work)
        panda.set_can_enable(CANBUS.radar, True)

        # Set CAN-FD speeds explicitly (like vinfast_set_safety.py does)
        panda.set_can_speed_kbps(CANBUS.radar, 500)  # Nominal: 500 kbps
        panda.set_can_data_speed_kbps(CANBUS.radar, 2000)  # Data: 2000 kbps (enables CAN-FD)

        # Disable auto-detection to use explicit speeds
        panda.set_canfd_auto(CANBUS.radar, False)

        del panda
        return True
      except Exception:
        return False

    def delayed_config_thread():
      """Wait 10 seconds after safety mode is set, then try to configure CAN-FD once."""
      # Wait 10 seconds for pandad to finish setting safety mode
      time.sleep(10.0)

      # Try to configure CAN-FD once
      try:
        configure_bus1()
      except Exception:
        pass

      # Thread exits after one attempt - no background checking

    # Start thread that waits 10 seconds then tries once
    config_thread = threading.Thread(target=delayed_config_thread, daemon=True, name="VinFastBus1Config")
    config_thread.start()

