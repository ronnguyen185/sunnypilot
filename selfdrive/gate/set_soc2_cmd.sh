#!/usr/bin/env bash
# Change onroad soc2d bench targets — same knobs as:
#   ./soc2_cmd.sh --lat 150 --lon 0 --lat-delay-s 3
#
# Requires openpilot onroad with GateSOC2BenchNoChassis=1.
# Do NOT run soc2_cmd.sh while openpilot owns the panda.
#
# Examples:
#   ./set_soc2_cmd.sh --lat 150 --lon 0 --lat-delay-s 3
#   ./set_soc2_cmd.sh --lat 90
#   ./set_soc2_cmd.sh --lat 0 --lon -1.5
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
LAT=""
LON=""
DELAY=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --lat) LAT="$2"; shift 2 ;;
    --lon) LON="$2"; shift 2 ;;
    --lat-delay-s) DELAY="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,14p' "$0"
      exit 0
      ;;
    *)
      echo "unknown arg: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "$LAT$LON$DELAY" ]]; then
  echo "usage: $0 --lat DEG [--lon MPS2] [--lat-delay-s SEC]" >&2
  exit 1
fi

python3 - <<PY
from openpilot.selfdrive.gate.gate_params import (
  write_gate_bench, gate_bench_lat_deg, gate_bench_lon_mps2,
  gate_lat_delay_s, gate_soc2_bench_no_chassis,
)
from openpilot.common.params import Params

if not gate_soc2_bench_no_chassis():
  raise SystemExit(
    "GateSOC2BenchNoChassis is off. Run:\n"
    "  /data/openpilot/selfdrive/gate/enable_bench_soc2.sh"
  )

lat, lon, delay = "${LAT}", "${LON}", "${DELAY}"
if lat:
  write_gate_bench("GateSOC2BenchLat", lat)
if lon:
  write_gate_bench("GateSOC2BenchLon", lon)
if delay:
  write_gate_bench("GateLatDelayS", delay)

p = Params()
print(
  f"set: lat={gate_bench_lat_deg(p)}deg lon={gate_bench_lon_mps2(p)}m/s^2 "
  f"lat_delay_s={gate_lat_delay_s(p, 0.05)}"
)
print("soc2d reloads /data/gate_bench within ~1s (no restart needed).")
PY

echo
echo "Verify:  python3 $DIR/verify_soc2d.py --bus 0"
echo "Observe: python3 $DIR/test_soc2_cmd.py --observe --bus 0 --lat ${LAT:-150} --lon ${LON:-0} --lat-delay-s ${DELAY:-3} --seconds 5 --listen"
