#!/usr/bin/env bash
# One-shot engage for COMMA_GATE (no cruise/LKAS buttons).
# Requires: GateSOC2BenchNoChassis=0, panda safety=soc2Gate, engageable=True.
set -euo pipefail

python3 - <<'PY'
from openpilot.selfdrive.gate.gate_params import (
  write_gate_bench, gate_soc2_bench_no_chassis, gate_soc2_enabled,
)
import cereal.messaging as messaging

if not gate_soc2_enabled():
  raise SystemExit("GateSOC2Enabled is off")
if gate_soc2_bench_no_chassis():
  raise SystemExit(
    "Still in bench stream mode (GateSOC2BenchNoChassis=1).\n"
    "Run: /data/openpilot/selfdrive/gate/enable_controlsd_lat.sh\n"
    "then restart manager."
  )

sm = messaging.SubMaster(['pandaStates', 'selfdriveState', 'carControl'])
for _ in range(30):
  sm.update(100)

ps = sm['pandaStates'][0]
print(f"panda safety={ps.safetyModel} controlsAllowed={ps.controlsAllowed}")
if str(ps.safetyModel) != 'soc2Gate' and int(ps.safetyModel) != 36:
  # DynamicEnum compare
  from cereal import car
  if ps.safetyModel != car.CarParams.SafetyModel.soc2Gate:
    raise SystemExit(
      f"panda still {ps.safetyModel} (want soc2Gate). Restart manager with "
      "GateForceIgnition=1 so launch exports STARTED=1."
    )

ss = sm['selfdriveState']
print(f"state={ss.state} enabled={ss.enabled} engageable={ss.engageable}")
print(f"alert: {ss.alertText1!r} / {ss.alertText2!r}")
if not ss.engageable and not ss.enabled:
  raise SystemExit("not engageable (often Low Memory). Free RAM or reboot, then retry.")

write_gate_bench("GateForceEngage", "1")
print("GateForceEngage=1 — selfdrived should enable within ~100ms")

for _ in range(50):
  sm.update(100)
  cc = sm['carControl']
  if cc.enabled and cc.latActive:
    print(
      f"OK: enabled={cc.enabled} latActive={cc.latActive} "
      f"steer={cc.actuators.steeringAngleDeg:.2f}deg"
    )
    break
else:
  cc = sm['carControl']
  print(
    f"timeout: enabled={cc.enabled} latActive={cc.latActive} "
    f"steer={cc.actuators.steeringAngleDeg:.2f}deg"
  )
  print("Check selfdriveState alerts / onroadEvents.")
PY
