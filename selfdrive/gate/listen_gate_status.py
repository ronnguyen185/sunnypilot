#!/usr/bin/env python3
"""Listen for GATE_STATUS (0x500) on the C3X gate bus.

Mirrors gate-panda gate-soc2/soc2/verify_gate.py and emulate_soc2_c3x.py --listen,
but runs on the comma 3X (SPI panda) instead of the gate USB host.

Use when openpilot is stopped or pandad is not holding the panda exclusively.
While onroad, prefer verify_soc2d.py (subscribes to cereal ``can``).
"""

from __future__ import annotations

import argparse
import sys
import time

from openpilot.common.params import Params
from openpilot.selfdrive.gate.gate_params import gate_soc2_bus
from openpilot.selfdrive.gate.gate_status import ID_GATE_STATUS, decode_gate_status, format_gate_status


def listen_panda(bus: int, duration: float | None) -> int:
  from panda import Panda

  p = Panda(Panda.list()[0])
  try:
    print(f"Listening for GATE_STATUS 0x{ID_GATE_STATUS:03X} on C3X bus {bus} ...")
    start = time.monotonic()
    last_sec_raw: int | None = None

    while True:
      if duration is not None and (time.monotonic() - start) >= duration:
        break

      for addr, dat, rx_bus in p.can_recv():
        if addr != ID_GATE_STATUS or (rx_bus & 0x7F) != bus:
          continue

        info = decode_gate_status(bytes(dat))
        ts = time.strftime("%H:%M:%S")
        sec_raw = info.get("sec_health_raw")
        marker = " <<" if sec_raw != last_sec_raw else ""
        last_sec_raw = sec_raw
        print(f"[{ts}] bus={rx_bus} {format_gate_status(info)}{marker}")

      time.sleep(0.001)

  except KeyboardInterrupt:
    print("\nStopped.")
  finally:
    p.close()

  return 0


def parse_args() -> argparse.Namespace:
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument("--bus", type=int, default=None, help="C3X gate bus (default: GateSOC2Bus)")
  ap.add_argument("--duration", type=float, default=None, help="Stop after N seconds")
  return ap.parse_args()


def main() -> int:
  args = parse_args()
  bus = args.bus if args.bus is not None else gate_soc2_bus(Params())
  try:
    return listen_panda(bus, args.duration)
  except IndexError:
    print("No panda found on C3X.", file=sys.stderr)
    return 1
  except Exception as e:
    print(f"listen failed: {type(e).__name__}: {e}", file=sys.stderr)
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
