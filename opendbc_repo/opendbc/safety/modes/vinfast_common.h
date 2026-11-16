#pragma once

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

// CAN message addresses (decimal from DBC)
#define VINFAST_ADAS_EPS_LATE_CON 890U  // 0x37A - Steering control from SCAM
#define VINFAST_ADAS_IDB_APA      309U  // 0x135 - ACC control from ADAS_Chassis
#define VINFAST_ADAS_LKA          306U  // 0x132 - LKA control from SCAM
#define VINFAST_ADAS_ACC_STATUS   813U  // 0x32D - ACC status from SCAM
#define VINFAST_ADAS_IDB          305U  // 0x131 - IDB control from SCAM
#define VINFAST_IDB_INFO_FLAG     815U  // 0x32F - IDB info flags (chassis + cam)
#define VINFAST_VEHICLE_DIRECTION 1036U // 0x40C
#define VINFAST_EPS_STEERING_TRQ  891U  // 0x37B
#define VINFAST_IDB_STATUS        525U  // 0x20D
#define VINFAST_SAS_SENSOR        382U  // 0x17E
#define VINFAST_BCM_CLAMP_STAT    274U  // 0x112

// Bus 0 = chassis, bus 2 = SCAM/cam (panda HW forwards 0<->2)
#define VINFAST_CHASSIS_BUS 0U
#define VINFAST_CAMERA_BUS  2U

// VF6 angle limits are enforced in Python (carcontroller); firmware stays permissive.
static const AngleSteeringLimits VINFAST_STEERING_LIMITS = {
  .max_angle = 47000,
  .angle_deg_to_can = 1.0f,
  .angle_rate_up_lookup = {
    {0.f, 20.f, 40.f},
    {7.f, 5.f, 3.f}
  },
  .angle_rate_down_lookup = {
    {0.f, 20.f, 40.f},
    {7.f, 5.f, 3.f}
  },
  .max_angle_error = 1000,
  .angle_error_min_speed = 0.0f,
  .frequency = 100U,
  .angle_is_curvature = false,
  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

static const LongitudinalLimits VINFAST_LONG_LIMITS = {
  .max_accel = 200,
  .min_accel = -350,
  .inactive_accel = 0,
};

static bool vinfast_longitudinal = false;

static uint8_t vinfast_get_counter(const CANPacket_t *msg) {
  if (msg->addr == VINFAST_ADAS_EPS_LATE_CON) {
    return (msg->data[1] >> 3) & 0xFU;
  }
  if (msg->addr == VINFAST_ADAS_LKA) {
    return (msg->data[1] >> 3) & 0xFU;
  }
  if (msg->addr == VINFAST_ADAS_ACC_STATUS) {
    return (msg->data[1] >> 3) & 0xFU;
  }
  if (msg->addr == VINFAST_IDB_STATUS) {
    return (msg->data[1] >> 3) & 0xFU;
  }
  return 0;
}

static uint32_t vinfast_get_checksum(const CANPacket_t *msg) {
  if (msg->addr == VINFAST_ADAS_EPS_LATE_CON ||
      msg->addr == VINFAST_ADAS_IDB_APA ||
      msg->addr == VINFAST_ADAS_LKA ||
      msg->addr == VINFAST_ADAS_ACC_STATUS ||
      msg->addr == VINFAST_IDB_STATUS) {
    return msg->data[0];
  }
  return 0;
}

static const uint8_t vinfast_crc8_table[256] = {
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
};

static uint32_t vinfast_compute_checksum(const CANPacket_t *msg) {
  uint8_t crc = 0xFF;
  int len = GET_LEN(msg);
  for (int i = 1; i < len; i++) {
    crc = vinfast_crc8_table[crc ^ msg->data[i]];
  }
  crc ^= 0xFF;
  return (uint32_t)crc;
}

static bool vinfast_get_quality_flag_valid(const CANPacket_t *msg) {
  UNUSED(msg);
  return true;
}

static void vinfast_rx_hook(const CANPacket_t *msg) {
  controls_allowed = true;

  if (msg->bus == VINFAST_CHASSIS_BUS) {
    if (msg->addr == VINFAST_SAS_SENSOR) {
      int angle_raw = (GET_BYTES(msg, 5, 2) >> 7) & 0xFFFFU;
      int angle_meas_new = ROUND(((float)angle_raw * 0.0238f - 780.0f) * 100.0f);
      update_sample(&angle_meas, angle_meas_new);
    }

    if (msg->addr == VINFAST_EPS_STEERING_TRQ) {
      int torque_raw = (GET_BYTES(msg, 3, 2) >> 6) & 0xFFFU;
      int torque_driver_new = ROUND(((float)torque_raw * 0.01f - 10.24f) * 100.0f);
      update_sample(&torque_driver, torque_driver_new);
    }

    if (msg->addr == VINFAST_IDB_STATUS) {
      int speed_raw = (GET_BYTES(msg, 2, 2) >> 7) & 0x1FFFU;
      float speed_kph = (float)speed_raw * 0.05625f;
      UPDATE_VEHICLE_SPEED(speed_kph * KPH_TO_MS);
      vehicle_moving = !GET_BIT(msg, 24U);
    }
  }
}

static bool vinfast_tx_hook(const CANPacket_t *msg) {
  controls_allowed = true;
  UNUSED(msg);
  return true;
}

static bool vinfast_fwd_hook_impl(int bus_num, int addr, bool block_idb_info_flag) {
  // bus_num is the SOURCE bus. true = block forwarding.
  if (bus_num == 1U) {
    return false;
  }

  if (bus_num == VINFAST_CHASSIS_BUS) {
    // VF6 hybrid: block stock IDB_Info chassis->cam when host sends patched copy on cam.
    if (block_idb_info_flag && (addr == VINFAST_IDB_INFO_FLAG) && vinfast_longitudinal) {
      return true;
    }
    return false;
  }

  if (bus_num == VINFAST_CAMERA_BUS) {
    // VF6 hybrid bridge: injected IDB_Info_Flag must stay on cam bus only.
    if (block_idb_info_flag && (addr == VINFAST_IDB_INFO_FLAG) && vinfast_longitudinal) {
      return true;
    }
    if (addr == VINFAST_ADAS_EPS_LATE_CON) {
      return true;
    }
    if (addr == VINFAST_ADAS_IDB && vinfast_longitudinal) {
      return true;
    }
    if (addr == VINFAST_ADAS_ACC_STATUS && vinfast_longitudinal) {
      return true;
    }
    return false;
  }

  return false;
}

static safety_config vinfast_common_init(uint16_t param) {
  const int VINFAST_PARAM_LONGITUDINAL = 1;
  vinfast_longitudinal = GET_FLAG(param, VINFAST_PARAM_LONGITUDINAL);
  controls_allowed = true;

  static const CanMsg VINFAST_TX_MSGS[] = {};
  static RxCheck vinfast_rx_checks[] = {};

  safety_config ret = BUILD_SAFETY_CFG(vinfast_rx_checks, VINFAST_TX_MSGS);
  ret.disable_forwarding = false;
  return ret;
}
