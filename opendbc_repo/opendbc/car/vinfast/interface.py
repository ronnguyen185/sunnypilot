from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.vinfast.carcontroller import CarController
from opendbc.car.vinfast.carstate import CarState
from opendbc.car.vinfast.radar_interface import RadarInterface
from opendbc.car.vinfast.values import CAR, CANBUS, is_vf6_safety_platform
from openpilot.common.params import Params


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface  # Re-enabled for diagnostics

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "vinfast"
    safety_model = (structs.CarParams.SafetyModel.vinfastVf6
                    if is_vf6_safety_platform(candidate)
                    else structs.CarParams.SafetyModel.vinfast)
    ret.safetyConfigs = [get_safety_config(safety_model)]

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
    # VF6: enable radar bus 1 and InfoCAN bus 5 in hardware; no radar DBC yet.
    ret.radarUnavailable = is_vf6_safety_platform(candidate)
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

    # Enable blind spot monitoring - VinFast VF8 has BSM via ADAS_BSD message on info CAN bus
    # This allows the UI to enable AutoLaneChangeBsmDelay option
    ret.enableBsm = True

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    """Sunnypilot-specific parameters for VinFast."""
    # Enable pcmCruiseSpeed so that openpilot longitudinal uses the car's set speed from ADAS_CMP_ACC
    # This allows openpilot to react to speed changes made via the car's cruise control buttons
    ret.pcmCruiseSpeed = True
    return ret

  @staticmethod
  def init(CP: structs.CarParams, CP_SP: structs.CarParamsSP, can_recv, can_send):
    """
    Initialize car interface.
    Configures CAN-FD for bus 1 (radar) after a 10 second delay.
    This is called after pandad sets safety mode, so we wait 10 seconds
    then try once to configure CAN-FD, then stop.
    Also sets default parameters for VinFast.
    """
    from openpilot.common.params import Params
    import time
    import threading

    # Set default parameters for VinFast if not already set
    params = Params()
    # Enable AutoLaneChangeBsmDelay by default for VinFast (has blind spot monitoring)
    if params.get("AutoLaneChangeBsmDelay") is None:
      params.put_bool("AutoLaneChangeBsmDelay", True)
    # Default VinFast auto lane change to 2-second delayed nudgeless mode.
    # AutoLaneChangeTimer enum: TWO_SECONDS = 4.
    if params.get("AutoLaneChangeTimer") is None:
      params.put("AutoLaneChangeTimer", "4")
    # VF6/VF7 Plus: default alpha longitudinal + experimental mode on first boot
    if is_vf6_safety_platform(CP.carFingerprint):
      if params.get("AlphaLongitudinalEnabled") is None:
        params.put_bool("AlphaLongitudinalEnabled", True)
      if params.get("ExperimentalMode") is None:
        params.put_bool("ExperimentalMode", True)

    def _open_pandas():
      from panda import Panda

      pandas = []
      seen = set()
      for serial in Panda.list() or []:
        try:
          p = Panda(serial=serial)
          pandas.append(p)
          seen.add(p.get_usb_serial())
        except Exception:
          continue
      # SPI/internal last-resort if list() missed it
      try:
        for serial in (Panda.spi_list() or []):
          if serial in seen:
            continue
          pandas.append(Panda(serial=serial))
      except Exception:
        pass
      pandas.sort(key=lambda x: (not getattr(x, "is_internal", lambda: False)(), x.get_usb_serial() or ""))
      return pandas

    def configure_buses():
      """Enable radar bus 1 (CAN-FD) and VF6 InfoCAN bus 5. Returns True if any panda was configured."""
      try:
        pandas = _open_pandas()
        if not pandas:
          return False

        ok = False
        for i, panda in enumerate(pandas):
          try:
            if i == 0:
              # First panda: radar on physical bus 1 (logical bus 1)
              panda.set_can_enable(CANBUS.radar, True)
              panda.set_can_speed_kbps(CANBUS.radar, 500)
              panda.set_can_data_speed_kbps(CANBUS.radar, 2000)
              panda.set_canfd_auto(CANBUS.radar, False)
              ok = True
            else:
              # Second panda: InfoCAN on physical bus 1 (logical bus 5)
              if is_vf6_safety_platform(CP.carFingerprint):
                panda.set_can_enable(1, True)
                panda.set_can_speed_kbps(1, 500)
                ok = True
          finally:
            try:
              panda.close()
            except Exception:
              pass
        return ok
      except Exception:
        return False

    def delayed_config_thread():
      """Wait 10 seconds after safety mode is set, then configure buses once."""
      time.sleep(10.0)
      try:
        configure_buses()
      except Exception:
        pass

    config_thread = threading.Thread(target=delayed_config_thread, daemon=True, name="VinFastBusConfig")
    config_thread.start()

