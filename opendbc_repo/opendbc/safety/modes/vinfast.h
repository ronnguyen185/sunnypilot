#pragma once

#include "opendbc/safety/modes/vinfast_common.h"

static bool vinfast_fwd_hook(int bus_num, int addr) {
  return vinfast_fwd_hook_impl(bus_num, addr, false);
}

static safety_config vinfast_init(uint16_t param) {
  return vinfast_common_init(param);
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
