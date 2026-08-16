#!/usr/bin/env python3
"""Verify engage → active gate CMD on sendcan (and inactive when disengaged).

Run while onroad with soc2d (GateSOC2BenchNoChassis=0):

  python3 selfdrive/gate/check_engage_cmd.py --seconds 30

Then hit ENGAGE (and optionally brake to cancel). Prints live CMD_LAT/LON
active bits vs carControl.enabled.
"""

from __future__ import annotations

import argparse
import struct
import time

import cereal.messaging as messaging

from openpilot.selfdrive.gate.gate_encode import ID_CMD_LAT, ID_CMD_LON, ID_SOURCE_HEALTH
from openpilot.selfdrive.gate.gate_params import gate_lat_delay_s, gate_soc2_bench_no_chassis, gate_soc2_bus
from openpilot.common.params import Params


def _decode(data: bytes) -> dict | None:
  if len(data) < 8:
    return None
  return {
    "active": bool(data[1] & 0x01),
    "req": data[2],
    "unit": data[3],
    "target": struct.unpack_from("<i", data, 4)[0],
  }


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument("--seconds", type=float, default=30.0)
  ap.add_argument("--bus", type=int, default=None)
  args = ap.parse_args()

  params = Params()
  bus = args.bus if args.bus is not None else gate_soc2_bus(params)
  bench = gate_soc2_bench_no_chassis(params)
  lat_delay = gate_lat_delay_s(params, 0.05)

  print(f"bus={bus} BenchNoChassis={int(bench)} GateLatDelayS={lat_delay:.2f}s")
  if bench:
    print("FAIL: GateSOC2BenchNoChassis=1 → fixed bench stream, not engage/carControl.")
    print("  Run: ./enable_controlsd_lat.sh && restart manager")
    return 1

  sm = messaging.SubMaster(["carControl", "selfdriveState", "pandaStates", "managerState"])
  send_sock = messaging.sub_sock("sendcan", timeout=50)

  last_lat = last_lon = None
  last_avail = None
  saw_engaged = False
  saw_active_lon_while_engaged = False
  saw_active_lat_while_engaged = False
  saw_inactive_while_disengaged = False
  bad_active_while_disengaged = False

  t0 = time.monotonic()
  last_print = 0.0
  print("waiting for engage (UI ENGAGE / engage_gate.sh) ...")

  while time.monotonic() - t0 < args.seconds:
    sm.update(0)
    for msg in messaging.drain_sock(send_sock):
      for c in msg.sendcan:
        if c.src != bus:
          continue
        data = bytes(c.dat)
        if c.address == ID_SOURCE_HEALTH and len(data) >= 2:
          last_avail = data[1]
        elif c.address == ID_CMD_LAT:
          last_lat = _decode(data)
        elif c.address == ID_CMD_LON:
          last_lon = _decode(data)

    if not sm.recv_frame["carControl"]:
      continue

    cc = sm["carControl"]
    enabled = bool(cc.enabled)
    lat_active = bool(cc.latActive)
    steer = float(cc.actuators.steeringAngleDeg)
    lat_on = bool(last_lat and last_lat["active"])
    lon_on = bool(last_lon and last_lon["active"])

    if enabled:
      saw_engaged = True
      if lon_on:
        saw_active_lon_while_engaged = True
      if lat_on:
        saw_active_lat_while_engaged = True
    else:
      if last_lat is not None and last_lon is not None:
        # Hot-standby: LON may stay active @ 0 while disengaged; LAT must not.
        if not lat_on:
          saw_inactive_while_disengaged = True
        if lat_on:
          bad_active_while_disengaged = True

    now = time.monotonic()
    if now - last_print >= 0.5:
      last_print = now
      ss = sm["selfdriveState"] if sm.recv_frame["selfdriveState"] else None
      alert = repr(ss.alertText1) if ss else ""
      lat_tgt = last_lat["target"] if last_lat else None
      lon_tgt = last_lon["target"] if last_lon else None
      print(
        f"  en={int(enabled)} latAct={int(lat_active)} steer={steer:7.2f} | "
        f"CMD_LAT={int(lat_on)} tgt={lat_tgt} | "
        f"CMD_LON={int(lon_on)} tgt={lon_tgt} | "
        f"health_avail={last_avail} {alert}"
      )

  print("\n=== result ===")
  print(f"  saw_engaged:                 {saw_engaged}")
  print(f"  LON active while engaged:    {saw_active_lon_while_engaged}")
  print(f"  LAT active while engaged:    {saw_active_lat_while_engaged}  (after GateLatDelayS={lat_delay:.1f}s)")
  print(f"  inactive while disengaged:   {saw_inactive_while_disengaged}")
  print(f"  BAD active while disengaged: {bad_active_while_disengaged}")

  if not saw_engaged:
    print("FAIL: never saw carControl.enabled — engage from UI, then re-run.")
    return 1
  if bad_active_while_disengaged:
    print("FAIL: CMD_LAT active while disengaged.")
    return 1
  if not saw_active_lon_while_engaged:
    print("FAIL: engaged but CMD_LON never active (soc2d / pair policy).")
    return 1
  if lat_delay <= 0.0 and not saw_active_lat_while_engaged:
    print("FAIL: engaged but CMD_LAT never active.")
    return 1
  if lat_delay > 0.0 and not saw_active_lat_while_engaged:
    print(
      f"WARN: LON ok but LAT never active in window — hold engage > {lat_delay:.1f}s "
      f"(GateLatDelayS) or set GateLatDelayS=0."
    )
    return 0

  print("OK: engage → active gate CMD; disengage → LAT inactive (LON standby OK).")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
