from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  ACCEL_MIN = -3.5  # m/s
  ACCEL_MAX = 2.0   # m/s

  STEER_MAX = 180
  STEER_DELTA_UP = 3
  STEER_DELTA_DOWN = 7
  STEER_DRIVER_ALLOWANCE = 50
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1
  STEER_THRESHOLD = 150
  STEER_STEP = 1  # 100 Hz

  # Angle rate limits matching vinfast.h safety code (speed-dependent)
  # Speed breakpoints: 0, 20, 40 m/s
  # Rate limits: up [7, 5, 3] deg/step, down [7, 5, 3] deg/step
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    180.0,  # STEER_ANGLE_MAX (degrees) - set to ±180 degrees
    ([0., 20., 40.], [7., 5., 3.]),   # ANGLE_RATE_LIMIT_UP (speed breakpoints, rate limits)
    ([0., 20., 40.], [7., 5., 3.]),  # ANGLE_RATE_LIMIT_DOWN (speed breakpoints, rate limits)
  )

  def __init__(self, CP):
    pass


class VinFastFlags(IntFlag):
  # Static flags
  # Add flags as needed
  pass


@dataclass
class VinFastCarDocs(CarDocs):
  package: str = "All"


@dataclass
class VinFastPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.chassis: "vinfast_vf8_chassis_can",
    Bus.cam: "vinfast_vf8_chassis_can",  # Camera bus uses same DBC (SCAM messages)
    Bus.radar: "vinfast_vf8_mrr_scam",  # Radar bus (CAN-FD) - MRR with SCAM integration
    Bus.body: "vinfast_vf8_info_can",  # Info CAN bus (turn signals, blind spot monitor, etc.)
  })


class CAR(Platforms):
  VINFAST_VF8 = VinFastPlatformConfig(
    [VinFastCarDocs("VinFast VF8 2023-24", "All", car_parts=CarParts.common([CarHarness.custom]))],
    CarSpecs(mass=2600, wheelbase=2.95, steerRatio=15.3, tireStiffnessFactor=0.82),  # Reduced from 17.0 to match Santa Fe's steerRatio (16.55) for better curvature response
  )
  VINFAST_VF9 = VinFastPlatformConfig(
    [VinFastCarDocs("VinFast VF9 2023-24", "All", car_parts=CarParts.common([CarHarness.custom]))],
    CarSpecs(mass=2928, wheelbase=3.15, steerRatio=15.0, tireStiffnessFactor=0.85),
  )


class CANBUS:
  # Note: Due to wiring, bus assignments are reversed:
  # - Bus 2 = Chassis bus (physical chassis CAN) on first panda
  # - Bus 0 = SCAM bus (camera/SCAM CAN) on first panda
  # - Bus 1 = Radar bus (CAN-FD) on first panda
  # - Info CAN bus: Second panda connected via USB, physical bus 2
  #   If pandad applies bus_offset=4 to second panda, messages appear on bus 6 (2+4)
  #   Adjust this value based on actual bus number in aggregated CAN stream
  chassis = 2  # Bus 2 is chassis bus (due to wiring) on first panda
  cam = 0      # Bus 0 is SCAM/camera bus (due to wiring) on first panda
  radar = 1    # Bus 1 is radar bus (CAN-FD) on first panda
  info = 6     # Info CAN bus: bus 2 on second panda (USB) -> bus 6 if bus_offset=4 is applied by pandad

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

