#!/usr/bin/env bash
# Enable openpilot SOC2 bench mode on the C3X.
#
# Mirrors the working gate host command:
#   ./soc2_cmd.sh --lat 150 --lon 0 --lat-delay-s 3 --listen
#
# GateSOC2* keys are NOT in the running params binary. Writing them under
# /data/params/d/ gets deleted by Params' async writer. They live in
# /data/gate_bench/ instead (read by gate_params.py).
#
# After running: restart manager (Ctrl-C launch_openpilot.sh, re-run it).
set -euo pipefail

cd "$(dirname "$0")/../.."

python3 - "$@" <<'PY'
import sys
from pathlib import Path
from openpilot.common.params import Params
from openpilot.selfdrive.gate.gate_params import write_gate_bench

# Defaults match soc2_cmd.sh --lat 150 --lon 0 --lat-delay-s 3
cfg = {
  "GateSOC2Enabled": "1",
  "GateSOC2BenchNoChassis": "1",
  "GateSOC2Bus": "0",
  "GateLatDelayS": "3",
  "GateSOC2BenchLat": "150",
  "GateSOC2BenchLon": "0",
  "GateForceIgnition": "1",
}

for arg in sys.argv[1:]:
  if "=" in arg:
    k, v = arg.split("=", 1)
    cfg[k] = v

p = Params()

for k, v in cfg.items():
  path = write_gate_bench(k, v)
  print(f"  {k} = {v}  ({path})")

# Known Params keys (safe to put):
try:
  p.put("CarPlatformBundle", {"platform": "COMMA_GATE", "name": "comma Gate SOC2"})
  print("  CarPlatformBundle = COMMA_GATE")
except Exception as e:
  print(f"  CarPlatformBundle FAILED: {e}")

# DeviceBootMode=1 ("Always Offroad") re-sets OffroadMode on every manager start.
try:
  p.put("DeviceBootMode", 0)
  print("  DeviceBootMode = 0")
except Exception:
  try:
    p.put("DeviceBootMode", "0")
    print("  DeviceBootMode = 0")
  except Exception as e:
    Path("/data/params/d/DeviceBootMode").write_bytes(b"0")
    print(f"  DeviceBootMode = 0 (file; put failed: {e})")

try:
  p.put_bool("OffroadMode", False)
  print("  OffroadMode = 0")
except Exception as e:
  Path("/data/params/d/OffroadMode").write_bytes(b"0")
  print(f"  OffroadMode = 0 (file; put failed: {e})")

print("gate SOC2 bench params set")
PY

echo
echo "Next steps:"
echo "  1. Restart manager (Ctrl-C launch_openpilot.sh, then ./launch_openpilot.sh)"
echo "  2. Confirm: deviceState.started=1 and pgrep -af soc2d"
echo "  3. Verify (bus 0 on this bench):"
echo "       python3 /data/openpilot/selfdrive/gate/verify_soc2d.py --bus 0"
echo "     or: python3 /data/openpilot/selfdrive/gate/test_soc2_cmd.py --observe --bus 0 --lat 150 --lon 0 --lat-delay-s 3 --require --listen"
