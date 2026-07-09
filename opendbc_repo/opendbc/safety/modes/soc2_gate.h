#pragma once

#include "opendbc/safety/safety_declarations.h"

// Gate Protocol v2 command IDs (decimal)
#define SOC2_GATE_CMD_HEALTH  0x400U
#define SOC2_GATE_CMD_LAT     0x410U
#define SOC2_GATE_CMD_LON     0x411U
#define SOC2_GATE_STATUS      0x500U

// C3X internal panda bus wired to gate-panda bus 1 (SOC2)
#define SOC2_GATE_DEFAULT_BUS 2U

static uint8_t soc2_gate_tx_bus = SOC2_GATE_DEFAULT_BUS;
static CanMsg soc2_gate_tx_msgs[3];

static void soc2_gate_rx_hook(const CANPacket_t *msg) {
  UNUSED(msg);
  controls_allowed = true;
}

static bool soc2_gate_tx_hook(const CANPacket_t *msg) {
  UNUSED(msg);
  return true;
}

static bool soc2_gate_fwd_hook(int bus_num, int addr) {
  UNUSED(addr);
  if ((uint8_t)bus_num == soc2_gate_tx_bus) {
    return true;  // block forwarding from gate bus
  }
  return false;
}

static safety_config soc2_gate_init(uint16_t param) {
  // param is the C3X gate TX bus from openpilot (gate_soc2_bus): 0 bench, 2 production.
  // Bus 0 is valid; do not treat param==0 as "use default".
  soc2_gate_tx_bus = (uint8_t)(param & 0xFFU);

  // Protocol v2: SOURCE_HEALTH=4B, CMD_LAT/LON=12B CAN-FD.
  soc2_gate_tx_msgs[0] = (CanMsg){SOC2_GATE_CMD_HEALTH, soc2_gate_tx_bus, 4, .check_relay = false};
  soc2_gate_tx_msgs[1] = (CanMsg){SOC2_GATE_CMD_LAT, soc2_gate_tx_bus, 12, .check_relay = false};
  soc2_gate_tx_msgs[2] = (CanMsg){SOC2_GATE_CMD_LON, soc2_gate_tx_bus, 12, .check_relay = false};

  controls_allowed = true;
  return (safety_config){NULL, 0, soc2_gate_tx_msgs, 3, true}; // NOLINT(readability/braces)
}

const safety_hooks soc2_gate_hooks = {
  .init = soc2_gate_init,
  .rx = soc2_gate_rx_hook,
  .tx = soc2_gate_tx_hook,
  .fwd = soc2_gate_fwd_hook,
};
