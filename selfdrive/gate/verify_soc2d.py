#!/usr/bin/env python3
"""Verify soc2d gate link: TX frames + GATE_STATUS from gate-panda.

Run on the C3X after enabling SOC2 (see enable_bench_soc2.sh).

Observes:
  - CMD_LAT / CMD_LON / SOURCE_HEALTH TX rates (via cereal sendcan)
  - GATE_STATUS 0x500 RX from gate-panda (via cereal can when onroad)
  - source_available flag in SOURCE_HEALTH
  - LAT engage delay (first active LON → first active LAT)
"""

from __future__ import annotations

import argparse
import time

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.selfdrive.gate.gate_encode import (
  ID_CMD_LAT,
  ID_CMD_LON,
  ID_SOURCE_HEALTH,
)
from openpilot.selfdrive.gate.gate_params import gate_soc2_bus
from openpilot.selfdrive.gate.gate_status import ID_GATE_STATUS, decode_gate_status, format_gate_status


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument("--seconds", type=float, default=8.0, help="observation window")
  ap.add_argument("--bus", type=int, default=None, help="gate bus (default: GateSOC2Bus)")
  args = ap.parse_args()

  params = Params()
  bus = args.bus if args.bus is not None else gate_soc2_bus(params)
  print(f"observing gate bus {bus} for {args.seconds:.1f}s (sendcan + can/GATE_STATUS) ...")

  send_sock = messaging.sub_sock("sendcan", timeout=100)
  can_sock = messaging.sub_sock("can", timeout=100)

  counts = {ID_SOURCE_HEALTH: 0, ID_CMD_LAT: 0, ID_CMD_LON: 0}
  gate_status_count = 0
  last_gate_status: dict | None = None
  first_lon_active_t: float | None = None
  first_lat_active_t: float | None = None
  last_available: int | None = None

  t0 = time.monotonic()
  while time.monotonic() - t0 < args.seconds:
    for msg in messaging.drain_sock(send_sock):
      for c in msg.sendcan:
        if c.src != bus:
          continue
        if c.address not in counts:
          continue
        counts[c.address] += 1
        data = bytes(c.dat)
        now = time.monotonic()
        if c.address == ID_SOURCE_HEALTH and len(data) >= 2:
          last_available = data[1]
        elif c.address == ID_CMD_LON and len(data) >= 2 and data[1] == 0x01:
          if first_lon_active_t is None:
            first_lon_active_t = now
        elif c.address == ID_CMD_LAT and len(data) >= 2 and data[1] == 0x01:
          if first_lat_active_t is None:
            first_lat_active_t = now

    for msg in messaging.drain_sock(can_sock):
      for c in msg.can:
        if c.src != bus or c.address != ID_GATE_STATUS:
          continue
        info = decode_gate_status(bytes(c.dat))
        if "note" in info:
          continue
        gate_status_count += 1
        last_gate_status = info

  print("\nTX frame counts (bus %d, sendcan):" % bus)
  print(f"  SOURCE_HEALTH (0x400): {counts[ID_SOURCE_HEALTH]}")
  print(f"  CMD_LAT       (0x410): {counts[ID_CMD_LAT]}")
  print(f"  CMD_LON       (0x411): {counts[ID_CMD_LON]}")
  print(f"  source_available (last): {last_available}")

  print(f"\nRX GATE_STATUS (0x500, can): {gate_status_count} frames")
  if last_gate_status:
    print(f"  latest: {format_gate_status(last_gate_status)}")
  else:
    print("  (none — is gate-panda running? Is C3X wired to gate bus 1?)")
    print("  bench fallback: python3 selfdrive/gate/listen_gate_status.py")

  streaming = counts[ID_CMD_LAT] > 0 and counts[ID_CMD_LON] > 0
  if not streaming:
    print("\nFAIL: no gate command TX seen. Is soc2d running? Is GateSOC2Bus correct?")
    return 1

  if first_lon_active_t is not None and first_lat_active_t is not None:
    delay = first_lat_active_t - first_lon_active_t
    print(f"\nLAT engage delay after LON active: {delay:.2f}s")
  elif first_lon_active_t is not None:
    print("\nLON active but LAT never went active in window (lat-delay > window or LAT off).")

  ok_status = last_gate_status is not None and last_gate_status.get("sec_health_raw") == 0
  if ok_status:
    print("\nOK: soc2d streaming + gate reports sec_health=GREEN.")
  elif last_gate_status is not None:
    print(f"\nWARN: soc2d streaming but sec_health={last_gate_status.get('sec_health')} "
          f"(state={last_gate_status.get('gate_state')}).")
  else:
    print("\nOK: soc2d streaming gate TX (GATE_STATUS not seen in window).")

  return 0 if streaming else 1


if __name__ == "__main__":
  raise SystemExit(main())
