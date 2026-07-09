"""Tests for carControl → gate emitter mapping and health aggregation."""

from unittest.mock import MagicMock

from openpilot.selfdrive.gate.gate_params import gate_soc2_bus
from openpilot.selfdrive.gate.health import car_state_fresh, source_available
from openpilot.selfdrive.gate.soc2d import bench_emitter_config, car_control_to_emitter_config


def _make_car_control(enabled=True, lat_active=True, long_active=True, valid=True,
                      steer=15.0, accel=0.5):
  cc = MagicMock()
  cc.valid = valid
  cc.enabled = enabled
  cc.latActive = lat_active
  cc.longActive = long_active
  cc.actuators.steeringAngleDeg = steer
  cc.actuators.accel = accel
  return cc


class TestCarControlMapping:
  def test_active_mapping(self):
    # Protocol v2: 15.0 deg → 15000, 0.5 m/s^2 → 500 (0.001/LSB)
    cfg = car_control_to_emitter_config(
      _make_car_control(), fresh=True, avail=True, bus=2, lat_delay_s=0.0,
    )
    assert cfg.lat_wanted
    assert cfg.lon_wanted
    assert cfg.lat_target == 15000
    assert cfg.lon_target == 500
    assert cfg.source_available
    assert cfg.bus == 2

  def test_stale_sends_inactive(self):
    cfg = car_control_to_emitter_config(
      _make_car_control(), fresh=False, avail=False, bus=2, lat_delay_s=0.0,
    )
    assert not cfg.lat_wanted
    assert not cfg.lon_wanted
    assert cfg.lat_target == 0
    assert cfg.lon_target == 0
    assert not cfg.source_available

  def test_gate_bus_zero(self):
    cfg = car_control_to_emitter_config(
      _make_car_control(), fresh=True, avail=True, bus=0, lat_delay_s=0.05,
    )
    assert cfg.bus == 0
    assert cfg.lat_delay_s == 0.05


class TestBenchEmitterConfig:
  def test_matches_soc2_cmd_defaults(self):
    # Protocol v2: 150 deg → 150000 @ 0.001 deg/LSB
    cfg = bench_emitter_config(bus=0, lat_deg=150.0, lon_mps2=0.0, lat_delay_s=3.0)
    assert cfg.bus == 0
    assert cfg.lat_wanted
    assert cfg.lon_wanted
    assert cfg.lat_target == 150000
    assert cfg.lon_target == 0
    assert cfg.lat_delay_s == 3.0
    assert cfg.source_available

  def test_lon_scaling(self):
    cfg = bench_emitter_config(bus=2, lat_deg=0.0, lon_mps2=-1.5, lat_delay_s=0.0)
    assert cfg.lat_target == 0
    assert cfg.lon_target == -1500


class TestGateSoc2BusParam:
  def test_default_bus_is_two(self):
    params = MagicMock()
    params.get.return_value = b"2"
    assert gate_soc2_bus(params) == 2

  def test_bench_bus_zero(self):
    params = MagicMock()
    params.get.return_value = b"0"
    assert gate_soc2_bus(params) == 0


def _make_sm(processes, car_valid=True, car_state_valid=True, car_state_can_valid=True):
  car_control = MagicMock(valid=car_valid)
  car_state = MagicMock(canValid=car_state_can_valid)
  manager_state = MagicMock(processes=processes)
  data = {
    'carControl': car_control,
    'carState': car_state,
    'managerState': manager_state,
  }

  class FakeSM:
    updated = {'managerState': True}
    alive = {'carControl': True, 'carState': True}
    valid = {'carControl': True, 'carState': True}
    logMonoTime = {'carControl': 1_000_000_000, 'carState': 1_000_000_000}

    def __getitem__(self, k):
      return data[k]

  return FakeSM()


def _proc(name: str, running: bool):
  p = MagicMock()
  p.name = name
  p.running = running
  return p


class TestHealthAggregator:
  def test_requires_all_processes(self):
    sm = _make_sm([
      _proc('controlsd', True),
      _proc('card', True),
      _proc('selfdrived', True),
      _proc('soc2d', False),
    ])
    assert not source_available(sm, 1_000_010_000)

  def test_all_healthy(self):
    sm = _make_sm([
      _proc('controlsd', True),
      _proc('card', True),
      _proc('selfdrived', True),
      _proc('soc2d', True),
    ])
    assert source_available(sm, 1_000_010_000)

  def test_stale_car_state_unavailable(self):
    sm = _make_sm([
      _proc('controlsd', True),
      _proc('card', True),
      _proc('selfdrived', True),
      _proc('soc2d', True),
    ])
    sm.logMonoTime['carState'] = 1_000_000_000
    assert not car_state_fresh(sm, 1_100_000_000)
    assert not source_available(sm, 1_100_000_000)

  def test_invalid_chassis_can_unavailable(self):
    sm = _make_sm([
      _proc('controlsd', True),
      _proc('card', True),
      _proc('selfdrived', True),
      _proc('soc2d', True),
    ], car_state_can_valid=False)
    assert not car_state_fresh(sm, 1_000_010_000)
    assert not source_available(sm, 1_000_010_000)

  def test_bench_no_chassis_skips_car_state(self):
    sm = _make_sm([
      _proc('controlsd', True),
      _proc('card', True),
      _proc('selfdrived', True),
      _proc('soc2d', True),
    ], car_state_can_valid=False)
    assert source_available(sm, 1_000_010_000, bench_no_chassis=True)

  def test_bench_no_chassis_still_requires_processes(self):
    sm = _make_sm([
      _proc('controlsd', True),
      _proc('card', True),
      _proc('selfdrived', True),
      _proc('soc2d', False),
    ], car_state_can_valid=False)
    assert not source_available(sm, 1_000_010_000, bench_no_chassis=True)
