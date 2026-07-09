"""Simulated emitter loop tests (no hardware), protocol v2."""

import struct

from openpilot.selfdrive.gate.emitter_loop import EmitterConfig, EmitterState, emitter_tick, run_emitter_sim
from openpilot.selfdrive.gate.gate_encode import (
  ID_CMD_LAT,
  ID_CMD_LON,
  ID_SOURCE_HEALTH,
  LAT_REQ_ANGLE,
  LAT_REQ_NONE,
  LAT_UNIT_DEGREE,
  LON_REQ_ACCEL,
)


class TestEmitterRates:
  def test_100_ticks_produce_100_lat_lon_and_50_health(self):
    sends = run_emitter_sim(100)
    assert sum(1 for s in sends if s.can_id == ID_CMD_LAT) == 100
    assert sum(1 for s in sends if s.can_id == ID_CMD_LON) == 100
    assert sum(1 for s in sends if s.can_id == ID_SOURCE_HEALTH) == 50

  def test_health_on_even_ticks_only(self):
    sends = run_emitter_sim(4)
    health_positions = [i for i, s in enumerate(sends) if s.can_id == ID_SOURCE_HEALTH]
    assert len(health_positions) == 2


class TestEmitterCounters:
  def test_lat_lon_share_control_cycle_counter_per_tick(self):
    sends = run_emitter_sim(5)
    lat_frames = [s for s in sends if s.can_id == ID_CMD_LAT]
    lon_frames = [s for s in sends if s.can_id == ID_CMD_LON]
    assert len(lat_frames) == 5
    assert len(lon_frames) == 5
    for i, (lat, lon) in enumerate(zip(lat_frames, lon_frames)):
      assert lat.data[0] == lon.data[0] == i

  def test_health_counter_increments_every_other_tick(self):
    sends = run_emitter_sim(6)
    health_frames = [s.data for s in sends if s.can_id == ID_SOURCE_HEALTH]
    counters = [f[0] for f in health_frames]
    assert counters == [0, 1, 2]

  def test_counters_wrap_at_256(self):
    cfg = EmitterConfig(lat_wanted=True, lon_wanted=True)
    state = EmitterState(health_counter=255, control_cycle_counter=255, tick=0)
    sends = emitter_tick(state, cfg)
    lat = next(s for s in sends if s.can_id == ID_CMD_LAT)
    assert lat.data[0] == 255
    assert state.control_cycle_counter == 0


class TestEmitterPayload:
  def test_default_bench_targets(self):
    sends = run_emitter_sim(1)
    lat = next(s for s in sends if s.can_id == ID_CMD_LAT)
    lon = next(s for s in sends if s.can_id == ID_CMD_LON)
    assert len(lat.data) == 12
    assert len(lon.data) == 12
    assert struct.unpack_from("<i", lat.data, 4)[0] == 15000
    assert struct.unpack_from("<i", lon.data, 4)[0] == 50
    assert lat.data[2] == LAT_REQ_ANGLE
    assert lat.data[3] == LAT_UNIT_DEGREE
    assert lon.data[2] == LON_REQ_ACCEL

  def test_inactive_mode(self):
    cfg = EmitterConfig(lat_wanted=False, lon_wanted=False, source_available=False)
    sends = run_emitter_sim(1, cfg)
    lat = next(s for s in sends if s.can_id == ID_CMD_LAT)
    lon = next(s for s in sends if s.can_id == ID_CMD_LON)
    health = next(s for s in sends if s.can_id == ID_SOURCE_HEALTH)
    assert lat.data[1] == 0x00
    assert lon.data[1] == 0x00
    assert health.data[1] == 0x00

  def test_all_sends_use_configured_bus_and_fd(self):
    cfg = EmitterConfig(bus=0, lat_wanted=True, lon_wanted=True)
    sends = run_emitter_sim(3, cfg)
    assert all(s.bus == 0 for s in sends)
    assert all(s.fd for s in sends)

  def test_lat_delay_defers_lateral_active(self):
    cfg = EmitterConfig(
      lat_wanted=True, lon_wanted=True, lat_target=150000, lon_target=0,
      lat_delay_s=0.01,
    )
    sends = run_emitter_sim(2, cfg)
    lat_frames = [s for s in sends if s.can_id == ID_CMD_LAT]
    assert lat_frames[0].data[1] == 0x00
    assert lat_frames[0].data[2] == LAT_REQ_NONE
    assert lat_frames[1].data[1] == 0x01
    assert struct.unpack_from("<i", lat_frames[1].data, 4)[0] == 150000

  def test_lat_delay_keeps_lon_active(self):
    """Inactive LAT + active LON during preamble (gate_pair_policy / EPS 0x37A)."""
    cfg = EmitterConfig(
      lat_wanted=True, lon_wanted=True, lat_target=150000, lon_target=0,
      lat_delay_s=0.01,
    )
    sends = run_emitter_sim(1, cfg)
    lat = next(s for s in sends if s.can_id == ID_CMD_LAT)
    lon = next(s for s in sends if s.can_id == ID_CMD_LON)
    assert lat.data[1] == 0x00
    assert lon.data[1] == 0x01
    assert lon.data[2] == LON_REQ_ACCEL
