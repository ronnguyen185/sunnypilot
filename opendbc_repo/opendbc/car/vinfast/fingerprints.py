""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.vinfast.values import CAR

Ecu = CarParams.Ecu

# Basic fingerprints for VinFast VF8
# These are common messages seen on the chassis bus (bus 2)
# TODO: Collect more complete fingerprints from actual vehicle
FINGERPRINTS = {
  CAR.VINFAST_VF8: [{
    # Common VinFast messages (decimal addresses from DBC)
    274: 8,   # 0x112 - BCM_CLAMP_STAT
    305: 6,   # 0x131 - ADAS_IDB
    306: 4,   # 0x132 - ADAS_LKA
    309: 8,   # 0x135 - ADAS_IDB_APA
    382: 7,   # 0x17E - SAS_Sensor (steering angle)
    383: 8,   # 0x17F - EPS_ADAS_TOI
    525: 8,   # 0x20D - IDB_STATUS (vehicle speed, status)
    596: 8,   # 0x254 - IDB_AVL_RPM_WHL_REAR (wheel speeds)
    597: 8,   # 0x255 - IDB_AVL_RPM_WHL_FRONT (wheel speeds)
    813: 5,   # 0x32D - ADAS_ACC_Status
    890: 8,   # 0x37A - ADAS_EPS_LATE_CON (steering control)
    891: 6,   # 0x37B - EPS_ADAS_Steering_Trq
    796: 8,   # 0x31C - EPS_SteeringHoldState
  }],
}

FW_VERSIONS = {
  CAR.VINFAST_VF8: {
    # TODO: Add firmware versions when available
    # Example structure:
    # (Ecu.eps, 0x730, None): [
    #   b'FW_VERSION_STRING\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    # ],
  },
}

