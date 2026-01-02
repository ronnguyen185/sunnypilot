from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.vinfast.values import CarControllerParams, CANBUS

# CRC-8 lookup table for VinFast checksum (from /data/card implementation)
VINFAST_CRC8_TABLE = [
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF,
    0x9C, 0x81, 0xA6, 0xBB, 0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E,
    0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76, 0x87, 0x9A, 0xBD, 0xA0,
    0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85,
    0xD6, 0xCB, 0xEC, 0xF1, 0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40,
    0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8, 0xDE, 0xC3, 0xE4, 0xF9,
    0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B,
    0x08, 0x15, 0x32, 0x2F, 0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A,
    0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2, 0x26, 0x3B, 0x1C, 0x01,
    0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24,
    0x77, 0x6A, 0x4D, 0x50, 0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2,
    0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A, 0x6C, 0x71, 0x56, 0x4B,
    0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA,
    0xA9, 0xB4, 0x93, 0x8E, 0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB,
    0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43, 0xB2, 0xAF, 0x88, 0x95,
    0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0,
    0xE3, 0xFE, 0xD9, 0xC4
]


def vinfast_checksum(data: bytes) -> int:
    """
    Calculate VinFast CRC-8 checksum.
    Checksum is calculated on bytes 1 to end (excluding first byte).
    Initial value: 0xFF, Final XOR: 0xFF.
    """
    crc = 0xFF
    for byte in data[1:]:
        crc = VINFAST_CRC8_TABLE[crc ^ byte]
    crc ^= 0xFF
    return crc


def _apply_checksum(packer: CANPacker, msg_name: str, bus: int,
                    checksum_field: str, values: dict) -> tuple[int, bytes, int]:
    """
    Helper that packs a CAN message, computes VinFast checksum, updates the checksum
    field, then returns the final CAN message tuple (address, data, bus).
    """
    msg = packer.make_can_msg(msg_name, bus, values)
    checksum = vinfast_checksum(msg[1])
    values[checksum_field] = checksum
    return packer.make_can_msg(msg_name, bus, values)


def create_steering_control(packer, CP, frame, apply_angle, lat_active):
    """
    Create steering control message (ADAS_EPS_LATE_CON)
    Message ID: 0x37A (890 decimal)
    VF8 uses angle-based steering control
    """
    alive = frame % 15
    values = {
        "CHKSM_ADAS_EPS_LATE_CON": 0,  # Will be calculated
        "ALV_ADAS_EPS_LATE_CON": alive,
        "ADAS_EPS_StrWhe_TOLAct": 0,
        "ADAS_EPS_StrWhe_AOLAct": 1 if lat_active else 0,
        "ADAS_EPS_AOLReq": apply_angle if lat_active else 0.0,
        "ADAS_EPS_Torq_Fact_Req": 0.5,
        "SECCAN_ADAS_EPS_LATE_CON": 0,  # TODO: Implement SECCAN if needed
    }

    return _apply_checksum(packer, "ADAS_EPS_LATE_CON", CANBUS.chassis, "CHKSM_ADAS_EPS_LATE_CON", values)


def create_acc_control(packer, CP, frame, accel, long_active, standstill):
    """
    Create ACC control message (ADAS_ACC_Status)
    Message ID: 0x32D (813 decimal)
    """
    alive = frame % 15

    # Clip acceleration within allowed range (-6 to 6 m/s^2 based on DBC)
    accel_cmd = max(-6.0, min(6.0, accel)) if long_active else 0.0

    values = {
        "CRC_ACC_STATUS": 0,  # will be calculated
        "Alive_ACC_STATUS": alive,
        "ADAS_ACC_Information": 0,
        "ADAS_ACC_Main_Mode": 1 if long_active else 0,
        "ADAS_ACC_AccelDecel_Cmd": accel_cmd,
        "ADAS_ACC_StandstillReq": 1 if (standstill and long_active) else 0,
        "ADAS_ACC_Mode": 4 if long_active else 2,
        "ADAS_ACC_IDB_DecCmdAct": 1 if accel_cmd < 0 else 0,
    }

    # Send on chassis bus (bus 2) for longitudinal control
    # When openpilot controls longitudinal, the message needs to be on chassis bus
    # to be visible on bus 130 (receipt of bus 2)
    # When using stock longitudinal, send on camera bus (bus 0) to match car's behavior
    bus = CANBUS.chassis if CP.openpilotLongitudinalControl else CANBUS.cam
    return _apply_checksum(packer, "ADAS_ACC_Status", bus, "CRC_ACC_STATUS", values)


def create_lka_control(packer, CP, frame, lat_active):
    """
    Create LKA control message (ADAS_LKA)
    Message ID: 0x132 (306 decimal)
    """
    # LSS activation: 0=off, 1=standby, 2=active, 3=error
    lss_activation = 2 if lat_active else 0

    # Haptic warning: 0=off, 1=on
    hap_warning = 0

    # Alive counter: 0-14 (4 bits)
    alive = frame % 15

    # Initial values (checksum will be calculated after encoding)
    values = {
        "ADAS_LKA_Checksum": 0,  # Will be calculated
        "ADAS_LKA_Alive": alive,
        "LSS_Activation": lss_activation,
        "SECCAN_ADAS_LKA": 0,  # TODO: Implement SECCAN if needed
        "ADAS_HapWarning": hap_warning,
    }

    return _apply_checksum(packer, "ADAS_LKA", CANBUS.chassis, "ADAS_LKA_Checksum", values)


def create_idb_control(packer, frame, drive_off_request, bus=0):
    """
    Create ADAS_IDB message (0x131) used for drive-off requests.
    """
    values = {
        "ADAS_IDB_Checksum": 0,
        "ADAS_IDB_Alive": frame % 15,
        "SECCAN_ADAS_IDB": 0,
        "ADAS_ACC_IDB_DriveOff": drive_off_request,
    }
    return _apply_checksum(packer, "ADAS_IDB", bus, "ADAS_IDB_Checksum", values)


def create_mfs_control_button(packer, frame, cruise_on_off=1, up_control=1, bus=0):
    """
    Create MFS_Control_Button message on the Info CAN.
    """
    values = {
        "CHSKM_MFS_Control_Button": 0,
        "ALV_MFS_Control_Button": frame % 15,
        "MFS_CruiseOn_Off": cruise_on_off,
        "MFS_UpControl": up_control,
    }
    return _apply_checksum(
        packer,
        "MFS_Control_Button",
        bus,
        "CHSKM_MFS_Control_Button",
        values,
    )


def create_adas_bcm_status(packer, frame, adas_status=1, headlight_request=0,
                           indicator_request=0, foldmirror_request=0, seccan_val=1, bus=0):
    """
    Create ADAS_BCM_Status (0x20F) on the body CAN.
    """
    values = {
        "ADAS_BCM_Status": adas_status,
        "ADAS_BCM_HeadlightReq": headlight_request,
        "ADAS_BCM_IndicatorlightReq": indicator_request,
        "ADAS_BCM_FoldMirrorReq": foldmirror_request,
        "SECCAN_ADAS_BCM_Sts": seccan_val,
    }
    return packer.make_can_msg("ADAS_BCM_Status", bus, values)


def create_bcm_central_lock(packer, frame, stat_cdl_led=0, door_unlock_fl=0, atws=0,
                            door_lock_fl=0, central_lock_sw_sts=0, bus=0):
    """
    Create BCM_STAT_CENTRAL_LOCK message with checksum.
    """
    values = {
        "CHKSM_BCM_CENTRAL_LOCK": 0,
        "ALV_BCM_CENTRAL_LOCK": frame % 15,
        "STAT_CDL_LED": stat_cdl_led,
        "STAT_DoorUnlockFL": door_unlock_fl,
        "STAT_ATWS": atws,
        "STAT_DoorLockFL": door_lock_fl,
        "BCM_CentralLockSwSts": central_lock_sw_sts,
    }
    return _apply_checksum(
        packer,
        "BCM_STAT_CENTRAL_LOCK",
        bus,
        "CHKSM_BCM_CENTRAL_LOCK",
        values,
    )


def create_doorfl_lock_command(packer, counter=0, bus=0):
    """
    Create DoorFLCommand message (no checksum).
    """
    values = {
        "NDoorFLVehicleLocked": 1,
        "NDoorFLVehicleMoving": 1,
        "NDoorFLOpenInvRequested": 0,
        "NDoorFLOpenRequested": 1,
        "NDoorFLChildLocked": 0,
        "NDoorFLVehicleCrash": 0,
        "NDoorFLCommandCounter": counter,
        "NDoorFLCloseInvRequested": 0,
        "NDoorFLCloseRequested": 1,
    }
    return packer.make_can_msg("DoorFLCommand", bus, values)

