from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.vinfast.values import CAR, CarControllerParams, CANBUS, is_vf6_safety_platform


def _vf6_combined_long(CP) -> bool:
  return is_vf6_safety_platform(CP.carFingerprint)

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

    if _vf6_combined_long(CP):
      torque_factor = 0.6
    else:
      # VF8/VF9: adaptive torque factor based on angle request
      abs_angle = abs(apply_angle) if lat_active else 0.0
      max_angle_for_full_torque = 300.0
      min_torque_factor = 0.5
      max_torque_factor = 0.6
      if abs_angle >= max_angle_for_full_torque:
        torque_factor = max_torque_factor
      else:
        torque_factor = min_torque_factor + (max_torque_factor - min_torque_factor) * (abs_angle / max_angle_for_full_torque)
    
    values = {
        "CHKSM_ADAS_EPS_LATE_CON": 0,  # Will be calculated
        "ALV_ADAS_EPS_LATE_CON": alive,
        "ADAS_EPS_StrWhe_TOLAct": 0,
        "ADAS_EPS_StrWhe_AOLAct": 1 if lat_active else 0,
        "ADAS_EPS_AOLReq": apply_angle if lat_active else 0.0,
        "ADAS_EPS_Torq_Fact_Req": torque_factor,
        "SECCAN_ADAS_EPS_LATE_CON": 0,  # TODO: Implement SECCAN if needed
    }

    return _apply_checksum(packer, "ADAS_EPS_LATE_CON", CANBUS.chassis, "CHKSM_ADAS_EPS_LATE_CON", values)


def create_acc_control(packer, CP, frame, accel, long_active, standstill, acc_popup_feed=0,
                       reengage_active=False, brake_pressed=False, transition_flag=False,
                       scam_acc_mode=None):
    """
    Create ACC control message (ADAS_ACC_Status)
    Message ID: 0x32D (813 decimal)

    VF6 openpilot long: combined joystick hybrid stop-and-go timing.
    VF8/VF9 openpilot long: original standstill-aligned ACC control.
    """
    alive = frame % 15
    op_long = CP.openpilotLongitudinalControl

    if op_long and _vf6_combined_long(CP):
      # Hybrid mirror: follow SCAM acc_mode; send planner accel only in mode 4 (working.py)
      ready_from_scam = (scam_acc_mode == 4)
      if not ready_from_scam:
        acc_mode = 3
      elif brake_pressed:
        acc_mode = 2
        transition_flag = False
      else:
        acc_mode = scam_acc_mode if scam_acc_mode is not None else 3

      if acc_mode == 4 and long_active:
        accel_cmd = max(-6.0, min(6.0, accel))
      else:
        accel_cmd = 0.0

      if transition_flag:
        standstill_req = 1
      elif acc_mode == 4 and accel_cmd > 0:
        standstill_req = 0
      elif reengage_active and long_active and standstill:
        standstill_req = 1
      else:
        standstill_req = 0

      if acc_mode == 4:
        if transition_flag:
          dec_cmd_act = 1
        elif accel_cmd < 0:
          dec_cmd_act = 1
        else:
          dec_cmd_act = 0
      else:
        dec_cmd_act = 0

      main_mode = 1 if acc_mode == 4 else 0
    elif op_long:
      accel_cmd = max(-6.0, min(6.0, accel)) if long_active else 0.0
      standstill_req = 1 if (standstill and long_active) else 0
      acc_mode = 4 if long_active else 2
      dec_cmd_act = 1 if accel_cmd < 0 else 0
      main_mode = 1 if long_active else 0
    else:
      accel_cmd = max(-6.0, min(6.0, accel)) if long_active else 0.0
      standstill_req = 1 if (standstill and long_active) else 0
      acc_mode = 4 if long_active else 2
      dec_cmd_act = 1 if accel_cmd < 0 else 0
      main_mode = 1 if long_active else 0

    values = {
        "CRC_ACC_STATUS": 0,  # will be calculated
        "Alive_ACC_STATUS": alive,
        "ADAS_ACC_Information": 0,
        "ADAS_ACC_Main_Mode": main_mode,
        "ADAS_ACC_AccelDecel_Cmd": accel_cmd,
        "ADAS_ACC_StandstillReq": standstill_req,
        "ADAS_ACC_Mode": acc_mode,
        "ADAS_ACC_IDB_DecCmdAct": dec_cmd_act,
    }
    # VF6 only: 8-byte ADAS_ACC_Status includes SECCAN at byte 7 (VF8/VF9 use 5-byte)
    if _vf6_combined_long(CP):
        values["SECCAN_ADAS_ACC_Status"] = 0

    bus = CANBUS.chassis if op_long else CANBUS.cam
    return _apply_checksum(packer, "ADAS_ACC_Status", bus, "CRC_ACC_STATUS", values)


def create_idb_info_flag(packer, frame, idb_info: dict, scam_acc_mode=None, scam_accel_cmd=0.0):
    """
    Create modified IDB_Info_Flag for SCAM bus (hybrid long-control bridge).

    Forwards IDB status from chassis; a/d flags follow stock SCAM accel when SCAM
    is Active (mode 4) — same as vinfast_combined_joystick_working.py --vinfast.
    Message ID: 0x32F (815 decimal)
    """
    if scam_acc_mode == 4:
      acc_decel_flag = 1 if scam_accel_cmd <= 0 else 0
      acc_accel_flag = 1 if scam_accel_cmd > 0 else 0
    else:
      acc_decel_flag = 0
      acc_accel_flag = 0

    driver_override = idb_info.get("IDB_ACC_DriverOverride", 0)

    values = {
        "CHKSM_IDB_Info_Flag": 0,
        "ALV_IDB_Info_Flag": frame % 15,
        "IDB_ACC_Available_Flag": idb_info.get("IDB_ACC_Available_Flag", 1),
        "IDB_ACC_Deceleration_Flag": acc_decel_flag,
        "IDB_ACC_Acceleration_Flag": acc_accel_flag,
        "IDB_RAEB_Active_Flag": idb_info.get("IDB_RAEB_Active_Flag", 0),
        "IDB_RAEB_Available_Flag": idb_info.get("IDB_RAEB_Available_Flag", 0),
        "IDB_AEB_Active_Flag": idb_info.get("IDB_AEB_Active_Flag", 0),
        "IDB_AEB_Available_Flag": idb_info.get("IDB_AEB_Available_Flag", 0),
        "IDB_ACC_DriverOverride": driver_override,
        "SECCAN_IDB_Info_Flag": idb_info.get("SECCAN_IDB_Info_Flag", 0),
    }

    return _apply_checksum(packer, "IDB_Info_Flag", CANBUS.cam, "CHKSM_IDB_Info_Flag", values)


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

