"""Aggregate C3X functional health for Gate SOURCE_HEALTH.source_available."""

from __future__ import annotations

import cereal.messaging as messaging

REQUIRED_PROCESSES = frozenset({"controlsd", "card", "selfdrived", "soc2d"})
CAR_CONTROL_STALE_NS = 50_000_000  # 50 ms
CAR_STATE_STALE_NS = 100_000_000  # 100 ms (card @ 100 Hz)


def processes_healthy(sm: messaging.SubMaster) -> bool:
  if not sm.updated['managerState']:
    return False
  running = {p.name for p in sm['managerState'].processes if p.running}
  return REQUIRED_PROCESSES.issubset(running)


def car_control_fresh(sm: messaging.SubMaster, now_mono: int) -> bool:
  if not sm.alive['carControl'] or not sm.valid['carControl']:
    return False
  return (now_mono - sm.logMonoTime['carControl']) < CAR_CONTROL_STALE_NS


def car_state_fresh(sm: messaging.SubMaster, now_mono: int) -> bool:
  """Chassis bus 0 feedback via card → carState (required for onroad SOC2)."""
  if not sm.alive['carState'] or not sm.valid['carState']:
    return False
  if not sm['carState'].canValid:
    return False
  return (now_mono - sm.logMonoTime['carState']) < CAR_STATE_STALE_NS


def source_available(
  sm: messaging.SubMaster,
  now_mono: int,
  *,
  bench_no_chassis: bool = False,
) -> bool:
  if not processes_healthy(sm):
    return False
  if bench_no_chassis:
    # Gate-link bench: C3X bus 0 not wired to chassis (matches soc2_cmd.sh always-available).
    # CMD_LAT/LON still follow carControl freshness/validity in soc2d mapping.
    return True
  if not car_control_fresh(sm, now_mono):
    return False
  if not sm['carControl'].valid:
    return False
  if not car_state_fresh(sm, now_mono):
    return False
  return True
