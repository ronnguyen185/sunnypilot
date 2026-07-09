#!/usr/bin/env python3
"""Stand-alone openpilot SOC2 command test (no onroad / soc2d required).

Because the C3X often cannot go onroad on a gate bench, this exercises the
**openpilot gate modules alone** — same encode/emitter path soc2d uses — with
CLI parity to gate-soc2/soc2_cmd.sh::

    ./soc2_cmd.sh --lat 150 --lon 0 --lat-delay-s 3 --listen

Modes
-----
  offline (default)
      Run openpilot emitter_loop + gate_encode in-process. No panda, no manager.
      Validates protocol v2 frames and LAT engage delay.

  --tx
      Same modules, but TX on the C3X panda (requires pandad/manager STOPPED).
      Optional --listen for GATE_STATUS 0x500 RX.

  --observe
      Subscribe cereal sendcan (needs onroad + soc2d). Use later when onroad works.

Examples
--------
  # Module-only (recommended on bench while offroad):
  python3 test_soc2_cmd.py --lat 150 --lon 0 --lat-delay-s 3

  # Module + real CAN to gate (stop openpilot first):
  python3 test_soc2_cmd.py --tx --lat 150 --lon 0 --lat-delay-s 3 --duration 10 --listen

  # Wrapper:
  ./soc2_cmd_test.sh --lat 150 --lon 0 --lat-delay-s 3
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

from openpilot.selfdrive.gate.emitter_loop import EmitterConfig, EmitterState, emitter_tick
from openpilot.selfdrive.gate.gate_encode import (
  CMD_LEN,
  ID_CMD_LAT,
  ID_CMD_LON,
  ID_SOURCE_HEALTH,
  LAT_REQ_ANGLE,
  LAT_REQ_NONE,
  LAT_UNIT_DEGREE,
  LAT_UNIT_NONE,
  LON_REQ_ACCEL,
  LON_UNIT_MPS2,
  encode_accel_mps2,
  encode_steer_deg,
)
from openpilot.selfdrive.gate.gate_status import ID_GATE_STATUS, decode_gate_status, format_gate_status
from openpilot.selfdrive.gate.soc2d import bench_emitter_config

TICK_HZ = 100
SAFETY_ALLOUTPUT = 17
CAN_SEND_TIMEOUT_MS = 2000


# ---------------------------------------------------------------------------
# Shared decode / expect helpers
# ---------------------------------------------------------------------------

def _decode_cmd(data: bytes) -> dict:
  if len(data) != CMD_LEN:
    return {"ok": False, "note": f"len={len(data)} != {CMD_LEN}"}
  return {
    "ok": True,
    "counter": data[0],
    "active": data[1] & 0x01,
    "req_type": data[2],
    "unit": data[3],
    "target": struct.unpack_from("<i", data, 4)[0],
  }


def _expect_raws(lat_deg: float, lon_mps2: float) -> tuple[int, int]:
  return encode_steer_deg(lat_deg), encode_accel_mps2(lon_mps2)


def _check_frame(
  *,
  name: str,
  data: bytes,
  expect_active: bool,
  expect_type: int | None,
  expect_unit: int | None,
  expect_target: int | None,
) -> list[str]:
  errs: list[str] = []
  info = _decode_cmd(data)
  if not info.get("ok"):
    return [f"{name}: {info.get('note')}"]
  if bool(info["active"]) != expect_active:
    errs.append(f"{name}: active={info['active']} expected {int(expect_active)}")
  if expect_active:
    if expect_type is not None and info["req_type"] != expect_type:
      errs.append(f"{name}: type={info['req_type']} expected {expect_type}")
    if expect_unit is not None and info["unit"] != expect_unit:
      errs.append(f"{name}: unit={info['unit']} expected {expect_unit}")
    if expect_target is not None and info["target"] != expect_target:
      errs.append(f"{name}: target={info['target']} expected {expect_target}")
  else:
    if info["req_type"] != LAT_REQ_NONE and name.startswith("LAT"):
      errs.append(f"{name}: inactive should use REQ_NONE")
    if info["unit"] != LAT_UNIT_NONE and name.startswith("LAT"):
      errs.append(f"{name}: inactive should use UNIT_NONE")
    if info["target"] != 0:
      errs.append(f"{name}: inactive target should be 0, got {info['target']}")
  return errs


# ---------------------------------------------------------------------------
# Mode: offline (module alone)
# ---------------------------------------------------------------------------

def run_offline(
  *,
  bus: int,
  lat_deg: float,
  lon_mps2: float,
  lat_delay_s: float,
  ticks: int,
  verbose: bool,
) -> int:
  """Exercise soc2d.bench_emitter_config + emitter_tick without hardware."""
  lat_raw, lon_raw = _expect_raws(lat_deg, lon_mps2)
  cfg = bench_emitter_config(
    bus=bus, lat_deg=lat_deg, lon_mps2=lon_mps2, lat_delay_s=lat_delay_s,
  )
  state = EmitterState()

  print("=== offline: openpilot gate modules (no onroad / no panda) ===")
  print(f"  cfg: bus={cfg.bus} lat_wanted={cfg.lat_wanted} lon_wanted={cfg.lon_wanted}")
  print(f"  targets: {lat_deg}deg/{lon_mps2}m/s^2 → raw lat={lat_raw} lon={lon_raw}")
  print(f"  lat_delay_s={lat_delay_s}  ticks={ticks} (hz={TICK_HZ})")
  print(f"  source_available={cfg.source_available}")

  delay_ticks = int(lat_delay_s * TICK_HZ) if lat_delay_s > 0 else 0
  errs: list[str] = []
  counts = {ID_SOURCE_HEALTH: 0, ID_CMD_LAT: 0, ID_CMD_LON: 0}
  first_lat_active_tick: int | None = None

  for tick in range(ticks):
    sends = emitter_tick(state, cfg)
    lat = next(s for s in sends if s.can_id == ID_CMD_LAT)
    lon = next(s for s in sends if s.can_id == ID_CMD_LON)
    counts[ID_CMD_LAT] += 1
    counts[ID_CMD_LON] += 1
    for s in sends:
      if s.can_id == ID_SOURCE_HEALTH:
        counts[ID_SOURCE_HEALTH] += 1
        if len(s.data) != 4 or s.data[1] != 0x01:
          errs.append(f"tick {tick}: bad SOURCE_HEALTH {s.data.hex()}")

    expect_lat_active = tick >= delay_ticks
    errs.extend(_check_frame(
      name=f"LAT@{tick}",
      data=lat.data,
      expect_active=expect_lat_active,
      expect_type=LAT_REQ_ANGLE if expect_lat_active else LAT_REQ_NONE,
      expect_unit=LAT_UNIT_DEGREE if expect_lat_active else LAT_UNIT_NONE,
      expect_target=lat_raw if expect_lat_active else 0,
    ))
    errs.extend(_check_frame(
      name=f"LON@{tick}",
      data=lon.data,
      expect_active=True,
      expect_type=LON_REQ_ACCEL,
      expect_unit=LON_UNIT_MPS2,
      expect_target=lon_raw,
    ))

    if lat.data[1] == 0x01 and first_lat_active_tick is None:
      first_lat_active_tick = tick

    if verbose and tick in (0, max(0, delay_ticks - 1), delay_ticks, ticks - 1):
      li = _decode_cmd(lat.data)
      lo = _decode_cmd(lon.data)
      print(
        f"  tick {tick:4d}: LAT active={li['active']} tgt={li['target']}  "
        f"LON active={lo['active']} tgt={lo['target']}  len={len(lat.data)}"
      )

    if lat.bus != bus or lon.bus != bus:
      errs.append(f"tick {tick}: bus mismatch")
    if not lat.fd or not lon.fd:
      errs.append(f"tick {tick}: expected fd=True")

  print("\n=== counts ===")
  print(f"  CMD_LAT={counts[ID_CMD_LAT]}  CMD_LON={counts[ID_CMD_LON]}  "
        f"SOURCE_HEALTH={counts[ID_SOURCE_HEALTH]}")
  if first_lat_active_tick is not None:
    print(f"  first LAT active at tick {first_lat_active_tick} "
          f"({first_lat_active_tick / TICK_HZ:.2f}s), expect delay_ticks={delay_ticks}")
    if first_lat_active_tick != delay_ticks:
      errs.append(
        f"LAT arm tick {first_lat_active_tick} != expected {delay_ticks}"
      )
  elif delay_ticks > 0 and ticks <= delay_ticks:
    print("  LAT never active (window shorter than lat_delay_s — OK)")
  elif delay_ticks > 0:
    errs.append("LAT never became active after delay")

  # Dedup flood
  uniq = list(dict.fromkeys(errs))
  print()
  if uniq:
    for e in uniq[:20]:
      print(f"FAIL: {e}")
    if len(uniq) > 20:
      print(f"... and {len(uniq) - 20} more")
    return 1

  print("OK: openpilot emitter/encode match soc2_cmd v2 (offline).")
  return 0


# ---------------------------------------------------------------------------
# Mode: --tx (module + panda hardware, like soc2_cmd.sh)
# ---------------------------------------------------------------------------

def _import_panda():
  try:
    from panda import Panda
    return Panda
  except Exception:
    sys.path.insert(0, "/data/openpilot")
    from panda import Panda
    return Panda


def _pandad_running() -> bool:
  import subprocess
  try:
    r = subprocess.run(["pgrep", "-f", r"pandad"], capture_output=True, timeout=2)
    return r.returncode == 0
  except Exception:
    return False


def run_tx(
  *,
  bus: int,
  lat_deg: float,
  lon_mps2: float,
  lat_delay_s: float,
  duration: float,
  listen: bool,
) -> int:
  """Stream openpilot emitter frames on C3X panda (pandad must be stopped)."""
  if _pandad_running():
    print(
      "ERROR: pandad is running and owns the SPI panda.\n"
      "  Stop openpilot first, e.g.:  sudo systemctl stop comma\n"
      "  Or use offline mode (default) without --tx.",
      file=sys.stderr,
    )
    return 1

  Panda = _import_panda()
  if not Panda.list():
    print("ERROR: no panda found.", file=sys.stderr)
    return 1

  from opendbc.car.structs import CarParams

  lat_raw, lon_raw = _expect_raws(lat_deg, lon_mps2)
  cfg = bench_emitter_config(
    bus=bus, lat_deg=lat_deg, lon_mps2=lon_mps2, lat_delay_s=lat_delay_s,
  )
  state = EmitterState()

  p = Panda(Panda.list()[0])
  print("=== tx: openpilot emitter → C3X panda (standalone, no soc2d) ===")
  print(f"  serial={p.get_usb_serial()} fw={p.get_version()!r}")
  print(f"  bus={bus} lat={lat_deg}deg(raw={lat_raw}) lon={lon_mps2}(raw={lon_raw}) "
        f"lat_delay_s={lat_delay_s}")
  try:
    p.set_power_save(False)
    p.set_safety_mode(CarParams.SafetyModel.allOutput)
    p.set_can_enable(bus, True)
    p.set_can_speed_kbps(bus, 500)
    p.set_can_data_speed_kbps(bus, 2000)
    p.set_canfd_non_iso(bus, False)
    p.set_canfd_auto(bus, True)
    p.can_clear(0xFFFF)

    t0 = time.monotonic()
    next_tick = t0
    last_diag = t0
    sent = 0
    sample_done = False

    while True:
      now = time.monotonic()
      if duration > 0 and (now - t0) >= duration:
        break

      sends = emitter_tick(state, cfg)
      if not sample_done:
        for s in sends:
          if s.can_id == ID_CMD_LAT:
            print(f"  sample 0x410 len={len(s.data)} hex={s.data.hex()}")
          if s.can_id == ID_CMD_LON:
            print(f"  sample 0x411 len={len(s.data)} hex={s.data.hex()}")
        sample_done = True

      try:
        p.can_send_many(
          [[s.can_id, s.data, s.bus] for s in sends],
          fd=True,
          timeout=CAN_SEND_TIMEOUT_MS,
        )
        sent += 1
      except Exception as e:
        print(f"  send error: {type(e).__name__}: {e}", file=sys.stderr)
        try:
          p.can_clear(bus)
        except Exception:
          pass

      if listen:
        for addr, dat, rxb in p.can_recv():
          if (rxb & 0x7F) == bus and addr == ID_GATE_STATUS:
            info = decode_gate_status(bytes(dat))
            if "note" not in info:
              print(f"  GATE_STATUS: {format_gate_status(info)}")

      if (now - last_diag) >= 1.0:
        try:
          h = p.can_health(bus)
          print(
            f"  bus{bus}: tx={h.get('total_tx_cnt')} rx={h.get('total_rx_cnt')} "
            f"err={h.get('total_error_cnt')} ticks={sent}",
            file=sys.stderr,
          )
        except Exception:
          pass
        last_diag = now

      try:
        p.send_heartbeat()
      except Exception:
        pass

      next_tick += 1.0 / TICK_HZ
      sleep = next_tick - time.monotonic()
      if sleep > 0:
        time.sleep(sleep)
      else:
        next_tick = time.monotonic()

  except KeyboardInterrupt:
    print("\nStopped.")
  finally:
    try:
      p.set_safety_mode(CarParams.SafetyModel.noOutput)
    except Exception:
      pass
    p.close()

  print(f"Done. streamed {sent} ticks.")
  return 0


# ---------------------------------------------------------------------------
# Mode: --observe (onroad cereal) — optional, when stack is up
# ---------------------------------------------------------------------------

def run_observe(
  *,
  bus: int,
  seconds: float,
  lat_deg: float,
  lon_mps2: float,
  lat_delay_s: float,
  listen: bool,
  require: bool,
) -> int:
  import cereal.messaging as messaging

  lat_raw, lon_raw = _expect_raws(lat_deg, lon_mps2)
  print("=== observe: cereal sendcan (needs onroad + soc2d) ===")
  print(f"  bus={bus} expect lat_raw={lat_raw} lon_raw={lon_raw} delay={lat_delay_s}s")

  send_sock = messaging.sub_sock("sendcan", timeout=100)
  can_sock = messaging.sub_sock("can", timeout=100) if listen else None
  counts = {ID_SOURCE_HEALTH: 0, ID_CMD_LAT: 0, ID_CMD_LON: 0}
  last_lat = last_lon = None
  first_lon_t = first_lat_t = None
  last_avail = None
  gate_n = 0
  last_gs = None

  t0 = time.monotonic()
  while time.monotonic() - t0 < seconds:
    for msg in messaging.drain_sock(send_sock):
      for c in msg.sendcan:
        if c.src != bus or c.address not in counts:
          continue
        data = bytes(c.dat)
        counts[c.address] += 1
        now = time.monotonic()
        if c.address == ID_SOURCE_HEALTH and len(data) >= 2:
          last_avail = data[1]
        elif c.address == ID_CMD_LAT and len(data) == CMD_LEN:
          last_lat = _decode_cmd(data)
          if last_lat["active"] and first_lat_t is None:
            first_lat_t = now
        elif c.address == ID_CMD_LON and len(data) == CMD_LEN:
          last_lon = _decode_cmd(data)
          if last_lon["active"] and first_lon_t is None:
            first_lon_t = now

    if can_sock is not None:
      for msg in messaging.drain_sock(can_sock):
        for c in msg.can:
          if c.src != bus or c.address != ID_GATE_STATUS:
            continue
          info = decode_gate_status(bytes(c.dat))
          if "note" not in info:
            gate_n += 1
            last_gs = info

  elapsed = max(time.monotonic() - t0, 1e-6)
  print("\n=== TX counts ===")
  for addr, name in (
    (ID_SOURCE_HEALTH, "0x400"),
    (ID_CMD_LAT, "0x410"),
    (ID_CMD_LON, "0x411"),
  ):
    print(f"  {name}: {counts[addr]} ({counts[addr] / elapsed:.1f} Hz)")
  print(f"  source_available={last_avail}")
  if last_lat:
    print(f"  last LAT: {last_lat}")
  if last_lon:
    print(f"  last LON: {last_lon}")
  if first_lon_t and first_lat_t:
    print(f"  measured LAT delay: {first_lat_t - first_lon_t:.2f}s")
  if listen:
    print(f"  GATE_STATUS frames={gate_n}")
    if last_gs:
      print(f"  latest: {format_gate_status(last_gs)}")

  streaming = counts[ID_CMD_LAT] > 0 and counts[ID_CMD_LON] > 0
  if not streaming:
    print(
      "\nFAIL: no frames. Device is likely offroad / soc2d not running.\n"
      "  Use default offline mode, or --tx with pandad stopped.",
      file=sys.stderr,
    )
    return 1

  errs: list[str] = []
  if last_lon and last_lon.get("ok") and last_lon["active"] and last_lon["target"] != lon_raw:
    errs.append(f"LON target {last_lon['target']} != {lon_raw}")
  if last_lat and last_lat.get("ok") and last_lat["active"] and last_lat["target"] != lat_raw:
    errs.append(f"LAT target {last_lat['target']} != {lat_raw}")

  if errs:
    for e in errs:
      print(f"FAIL: {e}")
    return 1 if require else 0

  print("\nOK: observed openpilot soc2d stream.")
  return 0


def main() -> int:
  ap = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter,
  )
  ap.add_argument("--lat", type=float, default=150.0, help="lateral target deg (default 150)")
  ap.add_argument("--lon", type=float, default=0.0, help="longitudinal m/s² (default 0)")
  ap.add_argument("--lat-delay-s", type=float, default=3.0, help="LAT engage delay (default 3)")
  ap.add_argument("--bus", type=int, default=0, help="C3X panda gate bus (default 0)")
  ap.add_argument("--ticks", type=int, default=400,
                  help="offline mode: number of 100 Hz ticks (default 400 = 4s)")
  ap.add_argument("--duration", type=float, default=10.0,
                  help="--tx mode: stream duration seconds (0 = until Ctrl-C)")
  ap.add_argument("--seconds", type=float, default=8.0, help="--observe window seconds")
  ap.add_argument("--tx", action="store_true",
                  help="TX on C3X panda via openpilot emitter (pandad must be stopped)")
  ap.add_argument("--observe", action="store_true",
                  help="observe cereal sendcan (needs onroad + soc2d)")
  ap.add_argument("--listen", action="store_true", help="print GATE_STATUS while --tx/--observe")
  ap.add_argument("--verbose", "-v", action="store_true", help="offline: print sample ticks")
  ap.add_argument("--require", action="store_true", help="--observe: non-zero exit on payload mismatch")
  args = ap.parse_args()

  if args.tx and args.observe:
    print("pick one of --tx or --observe (default is offline)", file=sys.stderr)
    return 2

  if args.tx:
    return run_tx(
      bus=args.bus,
      lat_deg=args.lat,
      lon_mps2=args.lon,
      lat_delay_s=args.lat_delay_s,
      duration=args.duration,
      listen=args.listen,
    )
  if args.observe:
    return run_observe(
      bus=args.bus,
      seconds=args.seconds,
      lat_deg=args.lat,
      lon_mps2=args.lon,
      lat_delay_s=args.lat_delay_s,
      listen=args.listen,
      require=args.require,
    )
  return run_offline(
    bus=args.bus,
    lat_deg=args.lat,
    lon_mps2=args.lon,
    lat_delay_s=args.lat_delay_s,
    ticks=args.ticks,
    verbose=args.verbose,
  )


if __name__ == "__main__":
  raise SystemExit(main())
