#!/usr/bin/env bash
# Switch SOC2 from fixed bench targets (soc2_cmd-like) to controlsd → carControl → lat.
#
# Prerequisites (already onroad with COMMA_GATE / GateSOC2Enabled):
#   - GateForceIgnition=1 (desk) OR real panda ignition
#   - Manager launched with STARTED=1 when using GateForceIgnition
#     (launch_chffrplus.sh exports this automatically)
#
# After this script:
#   1. Restart manager (CarParams must reload: steerAtStandstill, safety apply)
#   2. Wait until pandaStates.safetyModel == soc2Gate
#   3. Engage:  ./engage_gate.sh
#   4. Observe: python3 test_soc2_cmd.py --observe --bus 0 --seconds 10 --listen
#
# To go back to fixed lat/lon: ./enable_bench_soc2.sh
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR/../.."

python3 - <<'PY'
from openpilot.selfdrive.gate.gate_params import write_gate_bench, gate_soc2_enabled, gate_force_ignition

if not gate_soc2_enabled():
  raise SystemExit("GateSOC2Enabled is off. Run enable_bench_soc2.sh first (or set GateSOC2Enabled=1).")

# Leave fixed-target stream; soc2d will map carControl.actuators.steeringAngleDeg.
write_gate_bench("GateSOC2BenchNoChassis", "0")
# Keep ignition spoof for desk; clear any stale engage pulse.
write_gate_bench("GateForceEngage", "0")
if not gate_force_ignition():
  write_gate_bench("GateForceIgnition", "1")
  print("  GateForceIgnition = 1  (desk; set 0 if you have real ignition)")

print("  GateSOC2BenchNoChassis = 0  (soc2d follows carControl)")
print("  GateForceEngage = 0")
print("controlsd lat mode armed")
PY

echo
echo "Next steps:"
echo "  1. Restart manager (Ctrl-C launch_openpilot.sh, re-run it)."
echo "     STARTED=1 is exported automatically when GateForceIgnition=1."
echo "  2. Confirm panda safety:"
echo "       python3 -c \"import cereal.messaging as m; sm=m.SubMaster(['pandaStates']);"
echo "       [sm.update(100) for _ in range(20)]; print(sm['pandaStates'][0].safetyModel)\""
echo "     Expect: soc2Gate (not noOutput)"
echo "  3. Engage openpilot (one-shot):"
echo "       $DIR/engage_gate.sh"
echo "  4. Watch lat from controlsd:"
echo "       python3 $DIR/test_soc2_cmd.py --observe --bus 0 --seconds 10 --listen"
echo
echo "Note: Low Memory (NO_ENTRY) blocks engage — free RAM or reboot if engageable=False."
