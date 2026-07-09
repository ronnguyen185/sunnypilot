#!/usr/bin/env bash
# Test openpilot SOC2 command modules (no onroad required by default).
#
# Parity CLI with gate-soc2/soc2_cmd.sh:
#   ./soc2_cmd_test.sh --lat 150 --lon 0 --lat-delay-s 3
#
# Default = offline (emitter + encode only).
#   --tx       stream on C3X panda (stop pandad / openpilot first)
#   --observe  cereal sendcan when onroad + soc2d eventually works
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
exec python3 "$DIR/test_soc2_cmd.py" "$@"
