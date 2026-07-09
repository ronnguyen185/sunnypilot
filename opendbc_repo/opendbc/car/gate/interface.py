import threading
import time

from opendbc.car import get_safety_config, structs
from opendbc.car.gate.carcontroller import CarController
from opendbc.car.gate.carstate import CarState
from opendbc.car.interfaces import CarInterfaceBase
from openpilot.common.params import Params
from openpilot.selfdrive.gate.gate_params import gate_soc2_bus


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "gate"
    params = Params()
    gate_bus = gate_soc2_bus(params)
    # soc2Gate (@36) is already in the released cereal/panda safety; gateLink (@37)
    # would need a rebuild. Same whitelist (0x400/0x410/0x411 on GateSOC2Bus).
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.soc2Gate, gate_bus)]

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.05
    ret.steerLimitTimer = 1.0
    ret.centerToFront = ret.wheelbase * 0.4
    # Desk / gate-link: vEgo=0 and standstill=True; without this latActive stays false.
    ret.steerAtStandstill = True

    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long
    ret.pcmCruise = not ret.openpilotLongitudinalControl

    ret.startingState = True
    ret.vEgoStarting = 0.1
    ret.startAccel = 1.0
    ret.longitudinalActuatorDelay = 0.5
    ret.dashcamOnly = False

    return ret

  @staticmethod
  def init(CP: structs.CarParams, CP_SP: structs.CarParamsSP, can_recv, can_send):
    """Configure CAN-FD on the gate SOC2 bus after pandad sets safety mode."""
    params = Params()

    def configure_gate_bus() -> bool:
      gate_bus = gate_soc2_bus(params)
      try:
        from panda import Panda

        panda = None
        try:
          spi_list = Panda.spi_list()
          if spi_list:
            panda = Panda(serial=spi_list[0])
            if not panda.spi:
              panda.close()
              panda = None
        except Exception:
          panda = None

        if panda is None:
          try:
            panda = Panda()
            if not panda.spi:
              panda.close()
              panda = None
          except Exception:
            panda = None

        if panda is None or not panda.spi:
          return False

        panda.set_can_enable(gate_bus, True)
        panda.set_can_speed_kbps(gate_bus, 500)
        panda.set_can_data_speed_kbps(gate_bus, 2000)
        panda.set_canfd_auto(gate_bus, True)
        panda.close()
        return True
      except Exception:
        return False

    def delayed_config_thread():
      time.sleep(10.0)
      try:
        configure_gate_bus()
      except Exception:
        pass

    threading.Thread(target=delayed_config_thread, daemon=True, name="GateBusConfig").start()
