from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

# VF6 OEM steering limits — "Current" column from EPS table (deg / deg/s @ km/h).
# Rates are deg/step @ 100 Hz (e.g. 1.8 = 180 deg/s @ 0 km/h, 0.4 = 40 deg/s @ 100+ km/h).
VF6_STEER_SPEED_BP_MS = [0., 5.556, 11.111, 16.667, 22.222, 27.778, 33.333, 41.667]  # 0–150 km/h
VF6_STEER_ANGLE_MAX_V_DEG = [180., 120., 90., 70., 50., 40., 40., 40.]
VF6_STEER_RATE_V_DEG_PER_STEP = [1.8, 1.2, 0.9, 0.7, 0.5, 0.4, 0.4, 0.4]  # 180–40 deg/s


class CarControllerParams:
  ACCEL_MIN = -3.5  # m/s
  ACCEL_MAX = 2.0   # m/s

  # Driver torque hysteresis (Nm): press above ON, release below OFF (noise ~1 Nm).
  STEER_DRIVER_PRESS_NM = 2.0
  STEER_DRIVER_RELEASE_NM = 0.8
  STEER_PRESSED_MIN_COUNT = 5    # ~50 ms @ 100 Hz
  STEER_RELEASE_MIN_COUNT = 50   # ~500 ms @ 100 Hz
  STEER_PRESS_RATE_GAIN = 0.02   # Nm per deg/s
  STEER_PRESS_NM_MAX = 4.0

  STEER_DELTA_UP = 3
  STEER_DELTA_DOWN = 7
  STEER_DRIVER_ALLOWANCE = 50
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1
  STEER_THRESHOLD = 150
  STEER_STEP = 1  # 100 Hz

  def __init__(self, CP):
    if is_vf6_safety_platform(CP.carFingerprint):
      # VF6/VF7 Plus: speed-dependent angle (180° @0 → 40° @100+) and rate (180 → 40 deg/s)
      steer_max = 180.0
      self.STEER_MAX = int(steer_max)
      self.ANGLE_LIMITS = AngleSteeringLimits(
        steer_max,
        (VF6_STEER_SPEED_BP_MS, VF6_STEER_RATE_V_DEG_PER_STEP),
        (VF6_STEER_SPEED_BP_MS, VF6_STEER_RATE_V_DEG_PER_STEP),
      )
      self.ANGLE_MAX_LOOKUP = (VF6_STEER_SPEED_BP_MS, VF6_STEER_ANGLE_MAX_V_DEG)
    elif CP.carFingerprint == CAR.VINFAST_VF9.name:
      # Keep the local VF9 150° cap already tuned on this C3X
      self.STEER_MAX = 150
      self.ANGLE_LIMITS = AngleSteeringLimits(
        150.0,
        ([0., 20., 40.], [7., 5., 3.]),
        ([0., 20., 40.], [7., 5., 3.]),
      )
    else:
      # VF8 EPS accepts up to ~470 degrees
      self.STEER_MAX = 470
      self.ANGLE_LIMITS = AngleSteeringLimits(
        470.0,
        ([0., 20., 40.], [7., 5., 3.]),
        ([0., 20., 40.], [7., 5., 3.]),
      )


class VinFastFlags(IntFlag):
  pass


@dataclass
class VinFastCarDocs(CarDocs):
  package: str = "All"


@dataclass
class VinFastPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.chassis: "vinfast_vf8_chassis_can",
    Bus.cam: "vinfast_vf8_chassis_can",
    Bus.radar: "vinfast_vf8_mrr_scam",
    Bus.body: "vinfast_vf8_info_can",
  })


class CAR(Platforms):
  VINFAST_VF8 = VinFastPlatformConfig(
    [VinFastCarDocs("VinFast VF8 2023-24", "All", car_parts=CarParts.common([CarHarness.custom]))],
    CarSpecs(mass=2600, wheelbase=2.95, steerRatio=13.3, tireStiffnessFactor=0.82),
  )
  VINFAST_VF9 = VinFastPlatformConfig(
    [VinFastCarDocs("VinFast VF9 2023-24", "All", car_parts=CarParts.common([CarHarness.custom]))],
    CarSpecs(mass=2928, wheelbase=3.15, steerRatio=15.0, tireStiffnessFactor=0.85),
  )
  VINFAST_VF6 = VinFastPlatformConfig(
    [VinFastCarDocs("VinFast VF6 2024-25", "All", car_parts=CarParts.common([CarHarness.custom]))],
    CarSpecs(mass=2050, wheelbase=2.73, steerRatio=15.5, tireStiffnessFactor=0.82),
    dbc_dict={
      Bus.chassis: "vinfast_vf6_chassis_can",
      Bus.cam: "vinfast_vf6_chassis_can",
      Bus.body: "vinfast_vf6_info_can",
    },
  )
  VINFAST_VF7_PLUS = VinFastPlatformConfig(
    [VinFastCarDocs("VinFast VF7 Plus 2024-26", "All", car_parts=CarParts.common([CarHarness.custom]))],
    CarSpecs(mass=2205, wheelbase=2.84, steerRatio=15.5, tireStiffnessFactor=0.82),
    dbc_dict={
      Bus.chassis: "vinfast_vf6_chassis_can",
      Bus.cam: "vinfast_vf6_chassis_can",
      Bus.body: "vinfast_vf6_info_can",
    },
  )

VF6_SAFETY_PLATFORMS = (CAR.VINFAST_VF6, CAR.VINFAST_VF7_PLUS)

def is_vf6_safety_platform(candidate) -> bool:
  if isinstance(candidate, str):
    return candidate in {p.name for p in VF6_SAFETY_PLATFORMS}
  return candidate in VF6_SAFETY_PLATFORMS


class CANBUS:
  # First panda: 0 chassis, 1 radar (CAN-FD), 2 SCAM
  # Second panda: 4 / 5 / 6. VF6 InfoCAN is physical bus 1 → logical bus 5.
  chassis = 0
  cam = 2
  radar = 1
  info = 4      # VF8/VF9 InfoCAN
  info_vf6 = 5  # VF6 InfoCAN

  @staticmethod
  def info_bus(car_fingerprint: str) -> int:
    if is_vf6_safety_platform(car_fingerprint):
      return CANBUS.info_vf6
    return CANBUS.info

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
