#!/usr/bin/env bash
# Bring up openpilot onroad with SOC2 bench stream (GateSOC2BenchNoChassis=1).
#
# Prerequisites:
#   - C3X bus 0 wired to gate-panda bus 1 (SOC2 link)
#   - Gate-panda running gate firmware (not stock/SILENT)
#   - GATE_STATUS 0x500 visible on C3X bus 0 (~50 Hz) — confirms link is alive
#   - Desk bench: GateForceIgnition=1 (set by enable_bench_soc2.sh) fakes ignition
#     so hardwared can go onroad without a harness/jungle
#
# This script sets params; restart manager or reboot so hardwared + soc2d pick them up.
# Stop any bench tool that holds the panda first.
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== SOC2 bench onroad bring-up ==="
echo

# Bench tools (read_bus, emulate_soc2, test_soc2_cmd --tx) own the panda and block pandad.
if pgrep -f "read_bus.py|emulate_soc2|test_soc2_cmd.py.*--tx|soc2_emit.py" >/dev/null 2>&1; then
  echo "WARN: a bench SOC2 tool is still running and will block pandad on reboot."
  echo "      Stop it first (Ctrl-C read_bus / emulate_soc2, etc.)."
  echo
fi

bash "$DIR/enable_bench_soc2.sh" "$@"

echo
echo "After manager restart (~30–60 s onroad):"
echo "  pgrep -af 'soc2d|manager|pandad'"
echo "  python3 $DIR/test_soc2_cmd.py --observe --lat 150 --lon 0 --lat-delay-s 3 --seconds 10 --listen --require"
echo "  python3 $DIR/verify_soc2d.py"
echo
echo "Restart manager now (Ctrl-C the launch_openpilot.sh terminal, then re-run it),"
echo "or: sudo reboot"
