#!/usr/bin/env bash
# Free RAM on C3X (helps clear openpilot "Low Memory" / engage blocked).
#
# Usage:
#   ./free_ram.sh           # drop caches + stop optional heavy daemons
#   ./free_ram.sh --cursor  # also stop Cursor/VS Code remote server (~300–400MB)
#   ./free_ram.sh --all     # --cursor + aggressive tmp cleanup
set -euo pipefail

CURSOR=0
ALL=0
for a in "$@"; do
  case "$a" in
    --cursor) CURSOR=1 ;;
    --all) CURSOR=1; ALL=1 ;;
    -h|--help)
      sed -n '2,8p' "$0"
      exit 0
      ;;
  esac
done

before() { free -m | awk '/^Mem:/{printf "used=%sM avail=%sM free=%sM buff/cache=%sM\n",$3,$7,$4,$6}'; }

echo "=== before ==="
before

# Flush filesystem caches (safe; kernel will refill as needed).
sync
if [ "$(id -u)" -eq 0 ]; then
  echo 3 > /proc/sys/vm/drop_caches
else
  sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches' 2>/dev/null || \
    echo "warn: could not drop_caches (need sudo)"
fi

# Optional openpilot / sunnypilot processes that are safe to kill for desk SOC2.
# Manager will respawn mapd if it should be running; killing still helps briefly.
for pat in \
  'sunnypilot.mapd' \
  'selfdrive.navd' \
  'system.loggerd' \
  ; do
  if pgrep -f "$pat" >/dev/null 2>&1; then
    echo "stop: $pat"
    pkill -f "$pat" 2>/dev/null || true
  fi
done

if [ "$CURSOR" -eq 1 ]; then
  echo "stop: Cursor / vscode-server"
  pkill -f 'vscode-server|cursor-server|\.cursor-server' 2>/dev/null || true
  pkill -f 'cursor-remote-code' 2>/dev/null || true
fi

if [ "$ALL" -eq 1 ]; then
  # Large transient junk only — do not touch /data/params or openpilot.
  rm -rf /tmp/vscode-typescript* /tmp/node-compile-cache 2>/dev/null || true
  find /data/tmp -maxdepth 2 -type f -name '*.log' -size +20M -delete 2>/dev/null || true
fi

sleep 1
echo "=== after ==="
before
echo
echo "Tip: if still Low Memory, reboot or run:  $0 --cursor"
