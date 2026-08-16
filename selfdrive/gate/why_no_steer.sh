#!/usr/bin/env bash
# Why engaged SOC2 CMD is not turning the wheel — one-shot diagnosis.
set -euo pipefail
cd "$(dirname "$0")/../.."

python3 - <<'PY'
import struct, time
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.selfdrive.gate.gate_encode import ID_CMD_LAT, ID_CMD_LON, ID_SOURCE_HEALTH
from openpilot.selfdrive.gate.gate_params import (
  gate_lat_delay_s, gate_soc2_bench_no_chassis, gate_soc2_bus,
)
from openpilot.selfdrive.gate.gate_status import decode_gate_status, format_gate_status

params = Params()
bus = gate_soc2_bus(params)
delay = gate_lat_delay_s(params, 0.05)
bench = gate_soc2_bench_no_chassis(params)

print(f"GateSOC2Bus={bus} BenchNoChassis={int(bench)} GateLatDelayS={delay:.2f}s")
if bench:
  print("BLOCK: BenchNoChassis=1 → fixed stream, not engage/carControl.")
  print("  Fix: ./selfdrive/gate/enable_controlsd_lat.sh && restart manager")

sm = messaging.SubMaster(
  ["deviceState", "selfdriveState", "carControl", "carState", "pandaStates", "modelV2", "managerState"]
)
send_sock = messaging.sub_sock("sendcan", timeout=100)
can_sock = messaging.sub_sock("can", timeout=100)

last_lat = last_lon = last_h = None
last_gs = None
n_lat = n_lon = n_h = n_gs = 0
t0 = time.monotonic()
while time.monotonic() - t0 < 2.5:
  sm.update(0)
  for msg in messaging.drain_sock(send_sock):
    for c in msg.sendcan:
      if c.src != bus:
        continue
      d = bytes(c.dat)
      if c.address == ID_CMD_LAT:
        n_lat += 1
        last_lat = d
      elif c.address == ID_CMD_LON:
        n_lon += 1
        last_lon = d
      elif c.address == ID_SOURCE_HEALTH:
        n_h += 1
        last_h = d
  for msg in messaging.drain_sock(can_sock):
    for c in msg.can:
      if c.src == bus and c.address == 0x500:
        n_gs += 1
        last_gs = decode_gate_status(bytes(c.dat))

def dec(d):
  if not d or len(d) < 8:
    return None
  return {
    "active": bool(d[1] & 1),
    "tgt_deg": struct.unpack_from("<i", d, 4)[0] / 1000.0,
  }

print()
if not sm.recv_frame["deviceState"]:
  print("BLOCK: openpilot not running (no deviceState). Start ./launch_openpilot.sh")
  raise SystemExit(1)

print(f"started={sm['deviceState'].started} mem%={sm['deviceState'].memoryUsagePercent}")
ss = sm["selfdriveState"]
print(f"enabled={ss.enabled} state={ss.state} alert={ss.alertText1!r}")
cc = sm["carControl"]
print(
  f"carControl enabled={cc.enabled} latActive={cc.latActive} "
  f"steer={cc.actuators.steeringAngleDeg:.2f}deg"
)
cs = sm["carState"]
print(f"carState vEgo={cs.vEgo:.2f} brake={cs.brakePressed} canValid={cs.canValid}")
if sm.recv_frame["modelV2"]:
  print(f"model curv={sm['modelV2'].action.desiredCurvature:.5f} drop%={sm['modelV2'].frameDropPerc:.1f}")

if sm.recv_frame["pandaStates"] and len(sm["pandaStates"]):
  ps = sm["pandaStates"][0]
  print(f"panda safety={ps.safetyModel} param={ps.safetyParam}")
  if str(ps.safetyModel) != "soc2Gate" and int(ps.safetyModel) != 36:
    print("BLOCK: panda not soc2Gate — CMD may be blocked by safety.")
else:
  print("BLOCK: no pandaStates")

soc2 = None
for p in sm["managerState"].processes:
  if p.name == "soc2d":
    soc2 = p
print(f"soc2d running={soc2.running if soc2 else None}")

lat, lon = dec(last_lat), dec(last_lon)
avail = last_h[1] if last_h and len(last_h) > 1 else None
print(f"\nTX (2.5s): LAT={n_lat} LON={n_lon} HEALTH={n_h}")
print(f"  CMD_LAT={lat}  CMD_LON={lon}  source_available={avail}")
if avail == 0:
  print("BLOCK: source_available=0 — gate ignores CMD (forces sec=RED).")
  print("  soc2d health bug or control stack not ready; restart soc2d/manager after fix.")

if n_lat == 0:
  print("BLOCK: no sendcan CMD — soc2d not publishing on this bus.")
elif not ss.enabled:
  print("BLOCK: not engaged — hit ENGAGE first.")
elif lat and not lat["active"]:
  print("BLOCK: engaged but CMD_LAT inactive.")
  if delay > 0:
    print(f"  GateLatDelayS={delay}s — wait or: echo -n 0 > /data/gate_bench/GateLatDelayS")
  if not cc.latActive:
    print("  carControl.latActive=False (standstill/fault?)")
elif lat and abs(lat["tgt_deg"]) < 0.05:
  print("WARN: CMD_LAT active but target≈0 (model curv≈0 / pointing straight).")
  print("  Point camera at a curve/lane; desk with curv=0 will not move EPS.")
else:
  print(f"OK C3X: CMD_LAT active @ {lat['tgt_deg']:.2f}deg")

print(f"\nRX GATE_STATUS: {n_gs} frames")
if last_gs and "note" not in last_gs:
  print(f"  {format_gate_status(last_gs)}")
  if last_gs.get("gate_state") != "SECONDARY_ACTIVE":
    print("BLOCK: gate is NOT SECONDARY_ACTIVE — SOC2 CMD is standby only.")
    print("  EPS only moves when gate_state=SECONDARY_ACTIVE (SOC1 RED, SOC2 GREEN).")
    print("  On bench: stop SOC1 / wake_bus1 with SOC1 absent so secondary takes over.")
  elif last_gs.get("sec_health") != "GREEN":
    print("BLOCK: sec_health not GREEN — gate rejects SOC2 actuation.")
  else:
    print("OK gate: SECONDARY_ACTIVE + sec GREEN — EPS should follow CMD_LAT.")
elif n_gs == 0:
  print("BLOCK: no GATE_STATUS 0x500 on bus — wire/gate-panda not linked to C3X bus", bus)
  print("  Check: C3X GateSOC2Bus → gate bus 1, gate firmware running.")
PY
