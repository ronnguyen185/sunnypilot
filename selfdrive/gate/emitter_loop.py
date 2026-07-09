"""Pure SOC2 emitter tick logic (no panda/hardware). Used by soc2d and tests."""

from __future__ import annotations

from dataclasses import dataclass

from openpilot.selfdrive.gate.gate_encode import (
  ID_CMD_LAT,
  ID_CMD_LON,
  ID_SOURCE_HEALTH,
  LAT_REQ_ANGLE,
  LAT_REQ_NONE,
  LAT_UNIT_DEGREE,
  LAT_UNIT_NONE,
  LON_REQ_ACCEL,
  LON_REQ_NONE,
  LON_UNIT_MPS2,
  LON_UNIT_NONE,
  build_cmd_lat,
  build_cmd_lon,
  build_source_health,
)

TICK_HZ = 100


def lat_active_for_tick(tick: int, arm_tick: int, lat_delay_s: float, hz: int = TICK_HZ) -> bool:
  """Defer CMD_LAT after arm_tick; CMD_LON may stay active (EPS neutral preamble)."""
  if lat_delay_s <= 0.0:
    return True
  return tick >= arm_tick + int(lat_delay_s * hz)


@dataclass
class CanSend:
  can_id: int
  data: bytes
  bus: int
  fd: bool = True


@dataclass
class EmitterState:
  health_counter: int = 0
  control_cycle_counter: int = 0
  tick: int = 0
  lat_arm_tick: int | None = None


@dataclass
class EmitterConfig:
  bus: int = 2
  lat_wanted: bool = False
  lon_wanted: bool = False
  lat_target: int = 0
  lon_target: int = 0
  lat_delay_s: float = 0.0
  source_available: bool = True


def emitter_tick(state: EmitterState, cfg: EmitterConfig) -> list[CanSend]:
  """One 100 Hz tick: CMD_LAT + CMD_LON; SOURCE_HEALTH every 2nd tick."""
  if cfg.lat_wanted:
    if state.lat_arm_tick is None:
      state.lat_arm_tick = state.tick
    lat_active = lat_active_for_tick(state.tick, state.lat_arm_tick, cfg.lat_delay_s)
  else:
    state.lat_arm_tick = None
    lat_active = False

  lon_active = cfg.lon_wanted
  if lat_active:
    lat_type, lat_unit, lat_tgt = LAT_REQ_ANGLE, LAT_UNIT_DEGREE, cfg.lat_target
  else:
    lat_type, lat_unit, lat_tgt = LAT_REQ_NONE, LAT_UNIT_NONE, 0
  if lon_active:
    lon_type, lon_unit, lon_tgt = LON_REQ_ACCEL, LON_UNIT_MPS2, cfg.lon_target
  else:
    lon_type, lon_unit, lon_tgt = LON_REQ_NONE, LON_UNIT_NONE, 0

  sends: list[CanSend] = [
    CanSend(
      ID_CMD_LAT,
      build_cmd_lat(state.control_cycle_counter, lat_active, lat_type, lat_unit, lat_tgt),
      cfg.bus,
    ),
    CanSend(
      ID_CMD_LON,
      build_cmd_lon(state.control_cycle_counter, lon_active, lon_type, lon_unit, lon_tgt),
      cfg.bus,
    ),
  ]

  if state.tick % 2 == 0:
    sends.append(
      CanSend(
        ID_SOURCE_HEALTH,
        build_source_health(state.health_counter, cfg.source_available),
        cfg.bus,
      )
    )
    state.health_counter = (state.health_counter + 1) & 0xFF

  state.control_cycle_counter = (state.control_cycle_counter + 1) & 0xFF
  state.tick += 1
  return sends


def run_emitter_sim(ticks: int, cfg: EmitterConfig | None = None) -> list[CanSend]:
  """Simulate N emitter ticks and return all CAN sends in order."""
  # Default targets match soc2_cmd --lat 15 --lon 0.05 (0.001/LSB → 15000 / 50).
  cfg = cfg or EmitterConfig(lat_wanted=True, lon_wanted=True, lat_target=15000, lon_target=50)
  state = EmitterState()
  out: list[CanSend] = []
  for _ in range(ticks):
    out.extend(emitter_tick(state, cfg))
  return out
