#pragma once

#include "opendbc/safety/safety_declarations.h"

// UNUSED macro - defined here to avoid including utils.h which conflicts with panda's utils.h
#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

// CAN message addresses (decimal from DBC)
#define VINFAST_ADAS_EPS_LATE_CON 890U  // 0x37A - Steering control from SCAM
#define VINFAST_ADAS_IDB_APA      309U  // 0x135 - ACC control from ADAS_Chassis
#define VINFAST_ADAS_LKA          306U  // 0x132 - LKA control from SCAM
#define VINFAST_ADAS_ACC_STATUS   813U  // 0x32D - ACC status (longitudinal control) from SCAM
#define VINFAST_ADAS_IDB          305U  // 0x131 - IDB control from SCAM
#define VINFAST_VEHICLE_DIRECTION 1036U // 0x40C - Vehicle direction message
#define VINFAST_EPS_STEERING_TRQ  891U  // 0x37B - Steering torque feedback from EPS
#define VINFAST_IDB_STATUS        525U  // 0x20D - Vehicle status from IDB_Chassis
#define VINFAST_SAS_SENSOR        382U  // 0x17E - Steering angle sensor from EPS
#define VINFAST_BCM_CLAMP_STAT    274U  // 0x112 - Brake light status from XGW_Chassis

// CAN bus assignments
// Note: Due to wiring, bus assignments are reversed:
// - Bus 2 = Chassis bus (physical chassis CAN)
// - Bus 0 = SCAM bus (camera/SCAM CAN)
#define VINFAST_CHASSIS_BUS 2U  // Bus 2 is chassis bus (due to wiring)
#define VINFAST_CAMERA_BUS  0U  // Bus 0 is SCAM/camera bus (due to wiring)

// Steering angle limits (VF8 uses angle-based control)
// MAX_EPS_ANGLE = 470 degrees (from /data/card values.h)
static const AngleSteeringLimits VINFAST_STEERING_LIMITS = {
  .max_angle = 47000,  // centi-degrees (470 degrees)
  .angle_deg_to_can = 1.0,  // 1:1 conversion (already in degrees)
  .angle_rate_up_lookup = {
    {0., 20., 40.},  // speed breakpoints (m/s)
    {7., 5., 3.}     // angle rate limits (degrees per step) - conservative for 100Hz control
  },
  .angle_rate_down_lookup = {
    {0., 20., 40.},  // speed breakpoints (m/s)
    {7., 5., 3.}     // angle rate limits (degrees per step) - conservative for 100Hz control
  },
  .max_angle_error = 1000,  // centi-degrees (10 degrees)
  .angle_error_min_speed = 0.0,  // m/s
  .frequency = 100U,  // Hz
  .angle_is_curvature = false,
  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

// Longitudinal acceleration limits (from values.py)
static const LongitudinalLimits VINFAST_LONG_LIMITS = {
  .max_accel = 200,   // 2.0 m/s² in 1/100 m/s² units
  .min_accel = -350,  // -3.5 m/s² in 1/100 m/s² units
  .inactive_accel = 0,
};

static uint8_t vinfast_get_counter(const CANPacket_t *msg) {
  // Extract counter from alive counter field
  // ALV_ADAS_EPS_LATE_CON is in bits 11-14 (4 bits, 0-14)
  if (msg->addr == VINFAST_ADAS_EPS_LATE_CON) {
    return (msg->data[1] >> 3) & 0xFU;
  }
  // ALV_ADAS_LKA is in bits 11-14
  if (msg->addr == VINFAST_ADAS_LKA) {
    return (msg->data[1] >> 3) & 0xFU;
  }
  // Alive_ACC_STATUS is in bits 11-14
  if (msg->addr == VINFAST_ADAS_ACC_STATUS) {
    return (msg->data[1] >> 3) & 0xFU;
  }
  // ALV_IDB_STATUS is in bits 11-14
  if (msg->addr == VINFAST_IDB_STATUS) {
    return (msg->data[1] >> 3) & 0xFU;
  }
  return 0;
}

static uint32_t vinfast_get_checksum(const CANPacket_t *msg) {
  // Extract checksum from message
  // CHKSM/CRC is typically in byte 0, bits 7-0
  if (msg->addr == VINFAST_ADAS_EPS_LATE_CON) {
    return msg->data[0];
  }
  if (msg->addr == VINFAST_ADAS_IDB_APA) {
    return msg->data[0];
  }
  if (msg->addr == VINFAST_ADAS_LKA) {
    return msg->data[0];
  }
  if (msg->addr == VINFAST_ADAS_ACC_STATUS) {
    return msg->data[0];  // CRC_ACC_STATUS
  }
  if (msg->addr == VINFAST_IDB_STATUS) {
    return msg->data[0];
  }
  return 0;
}

// CRC-8 lookup table for VinFast checksum (from /data/card implementation)
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
  // VinFast uses CRC-8 checksum on bytes 1 to end (excluding first byte)
  // Initial value: 0xFF, Final XOR: 0xFF
  uint8_t crc = 0xFF;
  int len = GET_LEN(msg);

  // Calculate checksum on bytes 1 to len-1 (skip first byte)
  for (int i = 1; i < len; i++) {
    crc = vinfast_crc8_table[crc ^ msg->data[i]];
  }
  crc ^= 0xFF;  // Final XOR with 0xFF

  return (uint32_t)crc;
}

static bool vinfast_get_quality_flag_valid(const CANPacket_t *msg) {
  // TODO: Implement quality flag validation if VinFast uses it
  (void)msg;
  return true;
}

static void vinfast_rx_hook(const CANPacket_t *msg) {
  // For VinFast, always keep controls_allowed true
  // This prevents main.c from clearing it when heartbeat_engaged is false
  // VinFast uses lateral control only, so controls should be allowed even if heartbeat isn't engaged
  controls_allowed = true;

  // Extract signals from received CAN messages on chassis bus (bus 2)
  if (msg->bus == VINFAST_CHASSIS_BUS) {
    // Steering angle sensor (0x17E - SAS_Sensor)
    if (msg->addr == VINFAST_SAS_SENSOR) {
      // SAS_SteerWheelAngle: bits 47-62 (16 bits), scaling 0.0238, offset -780
      // Extract from bytes 5-6 (bits 40-55, need to account for bit position 47)
      int angle_raw = (GET_BYTES(msg, 5, 2) >> 7) & 0xFFFFU;  // bits 47-62
      int angle_meas_new = to_signed(angle_raw, 16);
      // Convert to centi-degrees (angle is already in degrees with scaling 0.0238, offset -780)
      // Physical angle = (raw * 0.0238) - 780
      // For safety checks, we use raw value and convert to centi-degrees
      angle_meas_new = ROUND(((float)angle_raw * 0.0238 - 780.0) * 100.0);
      update_sample(&angle_meas, angle_meas_new);
    }

    // Steering torque feedback (0x37B - EPS_ADAS_Steering_Trq)
    if (msg->addr == VINFAST_EPS_STEERING_TRQ) {
      // EPS_SteeringDriverTorque: bits 30-41 (12 bits), scaling 0.01, offset -10.24
      // Extract from bytes 3-4, bits 30-41
      int torque_raw = (GET_BYTES(msg, 3, 2) >> 6) & 0xFFFU;  // bits 30-41
      int torque_driver_new = to_signed(torque_raw, 12);
      // Convert to Nm: (raw * 0.01) - 10.24
      // For safety, we track in 1/100 Nm units
      torque_driver_new = ROUND(((float)torque_raw * 0.01 - 10.24) * 100.0);
      update_sample(&torque_driver, torque_driver_new);
    }

    // Vehicle status (0x20D - IDB_STATUS)
    if (msg->addr == VINFAST_IDB_STATUS) {
      // VehicleSpd: bits 23-35 (13 bits), scaling 0.05625, range 0-300 km/h
      int speed_raw = (GET_BYTES(msg, 2, 2) >> 7) & 0x1FFFU;  // bits 23-35
      float speed_kph = (float)speed_raw * 0.05625;
      UPDATE_VEHICLE_SPEED(speed_kph * KPH_TO_MS);

      // ESC_VehicleStandstill: bit 24
      vehicle_moving = !GET_BIT(msg, 24U);
    }
  }

  // ACC status from SCAM bus (bus 0) - forwarded from camera
  // ADAS_ACC_Status (0x32D) arrives on bus 0 from camera, gets forwarded to bus 2
  // Note: For VinFast, we don't use pcm_cruise_check to control controls_allowed
  // because we want to allow controls even when stock ACC is not engaged
  // (we're using openpilot's lateral control with stock longitudinal)
  // The message is still parsed by carstate.py to determine cruise control state
  // but we don't use it to control controls_allowed here to prevent controlsMismatch
  // Message is handled by carstate.py, not used here for safety control
}

static bool vinfast_tx_hook(const CANPacket_t *msg) {
  // For VinFast, always keep controls_allowed true when sending messages
  // This ensures controls_allowed stays true when openpilot is engaged and sending control messages
  // The TX hook is called frequently when openpilot is active, preventing main.c from clearing it
  controls_allowed = true;

  // TX hook disabled - allow all messages to be sent
  // All forwarding logic is handled in fwd_hook only
  UNUSED(msg);
  return true;  // Always allow transmission
}

// Flag to track if openpilot longitudinal control is enabled
// Set via safety parameter flag (bit 0)
static bool vinfast_longitudinal = false;

static bool vinfast_fwd_hook(int bus_num, int addr) {
  // Forwarding logic (with correct bus assignments):
  // VINFAST_CAMERA_BUS = 0 (SCAM bus)
  // VINFAST_CHASSIS_BUS = 2 (Chassis bus)
  // Bus 1 = Radar bus (CAN-FD) - no forwarding (get_fwd_bus(1) returns -1)
  //
  // bus_num is the SOURCE bus number
  //
  // - Bus 2 (Chassis) -> Bus 0 (SCAM): Forward everything
  // - Bus 0 (SCAM) -> Bus 2 (Chassis): Block 0x37A (ADAS_EPS_LATE_CON) always, and 0x131 (ADAS_IDB) + 0x32D (ADAS_ACC_Status) when OP long is enabled
  // - Bus 1 (Radar): No forwarding needed, but allow messages to be received (return false = allow)

  // Bus 1 (Radar) - no forwarding, but allow messages to pass through
  // IMPORTANT: Bus 1 messages should NOT be forwarded, but they should be allowed to be received
  // Returning false here means "don't block" - the message will be queued but not forwarded
  if (bus_num == 1U) {  // Bus 1 = Radar bus (CAN-FD)
    return false;  // false = allow (don't block), but get_fwd_bus(1) returns -1 so no forwarding happens
  }

  // Bus 2 (Chassis) -> Bus 0 (SCAM): Forward everything
  if (bus_num == VINFAST_CHASSIS_BUS) {  // Bus 2 = Chassis bus (source)
    // Forward all messages from chassis bus to SCAM bus
    return false;  // false = allow forwarding
  }

  // Bus 0 (SCAM) -> Bus 2 (Chassis): Block EPS_LATE_CON, ADAS_IDB, and ADAS_ACC_Status when openpilot controls
  if (bus_num == VINFAST_CAMERA_BUS) {  // Bus 0 = SCAM bus (source)
    // Block 0x37A (ADAS_EPS_LATE_CON) - prevent steering manipulation from SCAM to chassis
    if (addr == VINFAST_ADAS_EPS_LATE_CON) {      // 890 (0x37A)
      return true;  // true = block forwarding
    }
    // Block 0x131 (ADAS_IDB) when openpilot longitudinal control is enabled
    // OpenPilot sends its own ADAS_IDB messages for drive-off requests, so block the car's stock messages
    // Only block when vinfast_longitudinal is true (openpilot is controlling longitudinal)
    if (addr == VINFAST_ADAS_IDB && vinfast_longitudinal) {  // 305 (0x131)
      return true;  // true = block forwarding when openpilot longitudinal is enabled
    }
    // Block 0x32D (ADAS_ACC_Status) when openpilot longitudinal control is enabled
    // When openpilot controls longitudinal, it sends its own ADAS_ACC_Status messages
    // Blocking the car's ADAS_ACC_Status from SCAM prevents conflicts
    // Always block when vinfast_longitudinal is true (openpilot is controlling longitudinal)
    if (addr == VINFAST_ADAS_ACC_STATUS && vinfast_longitudinal) {  // 813 (0x32D)
      return true;  // true = block forwarding when openpilot longitudinal is enabled
    }
    // Allow all other messages from SCAM bus to forward to chassis bus
    return false;  // false = allow forwarding
  }

  // For other buses, use default forwarding behavior
  return false;  // false = allow forwarding
}

static safety_config vinfast_init(uint16_t param) {
  // Check if openpilot longitudinal control is enabled via safety parameter flag (bit 0)
  // This flag is set by openpilot when AlphaLongitudinalEnabled is true
  const int VINFAST_PARAM_LONGITUDINAL = 1;
  vinfast_longitudinal = GET_FLAG(param, VINFAST_PARAM_LONGITUDINAL);

  // CAN-FD configuration for bus 1 is handled by Python code in interface.py
  // The Python code persistently configures bus 1 for CAN-FD (500/2000 kbps)
  // via a background thread that runs continuously

  // Enable controls for VinFast mode
  controls_allowed = true;

  // TODO: Add TX_MSGS whitelist later - for now, allow all messages
  // No TX whitelist - all messages allowed (similar to allOutput)
  static const CanMsg VINFAST_TX_MSGS[] = {
    // Empty - will be added gradually
  };

  // RX checks disabled temporarily to prevent controlsMismatch
  // TODO: Re-enable and fix checksum/counter validation once messages are verified on correct bus
  static RxCheck vinfast_rx_checks[] = {
    // Empty - no RX message validation (disables safetyRxChecksInvalid)
  };

  safety_config ret = BUILD_SAFETY_CFG(vinfast_rx_checks, VINFAST_TX_MSGS);
  ret.disable_forwarding = false;  // Explicitly enable forwarding
  return ret;
}

const safety_hooks vinfast_hooks = {
  .init = vinfast_init,
  .rx = vinfast_rx_hook,
  .tx = vinfast_tx_hook,
  .fwd = vinfast_fwd_hook,
  .get_counter = vinfast_get_counter,
  .get_checksum = vinfast_get_checksum,
  .compute_checksum = vinfast_compute_checksum,
  .get_quality_flag_valid = vinfast_get_quality_flag_valid,
};