#pragma once

#include "opendbc/safety/modes/vinfast_common.h"

// VF6 hybrid long-control bridge: block stock IDB_Info_Flag from chassis->SCAM
// when openpilot longitudinal is active; carcontroller sends modified copy on cam bus.
static bool vinfast_vf6_fwd_hook(int bus_num, int addr) {
  return vinfast_fwd_hook_impl(bus_num, addr, true);
}

static safety_config vinfast_vf6_init(uint16_t param) {
  safety_config ret = vinfast_common_init(param);
  // VF6 hybrid long: selective fwd_hook blocks IDB_Info_Flag chassis->cam and
  // SCAM ADAS_* cam->chassis; panda HW forwards everything else (no host bridge).
  return ret;
}

const safety_hooks vinfast_vf6_hooks = {
  .init = vinfast_vf6_init,
  .rx = vinfast_rx_hook,
  .tx = vinfast_tx_hook,
  .fwd = vinfast_vf6_fwd_hook,
  .get_counter = vinfast_get_counter,
  .get_checksum = vinfast_get_checksum,
  .compute_checksum = vinfast_compute_checksum,
  .get_quality_flag_valid = vinfast_get_quality_flag_valid,
};
