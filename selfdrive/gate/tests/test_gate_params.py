"""Tests for gate SOC2 param helpers (including /data/gate_bench fallback)."""

from pathlib import Path
from unittest.mock import MagicMock

import openpilot.selfdrive.gate.gate_params as gp
from openpilot.common.params import UnknownKeyName
from openpilot.selfdrive.gate.gate_params import (
  gate_bench_lat_deg,
  gate_bench_lon_mps2,
  gate_force_ignition,
  gate_lat_delay_s,
  gate_soc2_bench_no_chassis,
  gate_soc2_bus,
  gate_soc2_enabled,
)


def _params(get_value):
  params = MagicMock()
  params.get.return_value = get_value
  return params


class TestGateSoc2Bus:
  def test_default_when_empty(self, tmp_path, monkeypatch):
    monkeypatch.setattr(gp, "GATE_BENCH_DIR", tmp_path)
    assert gate_soc2_bus(_params(None)) == 2

  def test_bench_bus_zero(self):
    assert gate_soc2_bus(_params(b"0")) == 0


class TestGateLatDelayS:
  def test_uses_steer_actuator_delay_when_unset(self):
    assert gate_lat_delay_s(_params(b"0"), 0.05) == 0.05

  def test_override_when_positive(self):
    assert gate_lat_delay_s(_params(b"3"), 0.05) == 3.0


class TestGateSoc2BenchNoChassis:
  def test_default_off(self, tmp_path, monkeypatch):
    monkeypatch.setattr(gp, "GATE_BENCH_DIR", tmp_path)
    assert not gate_soc2_bench_no_chassis(_params(None))

  def test_enabled(self):
    assert gate_soc2_bench_no_chassis(_params(b"1"))


class TestGateSoc2Enabled:
  def test_off(self):
    assert not gate_soc2_enabled(_params(b"0"))

  def test_on(self):
    assert gate_soc2_enabled(_params(b"1"))


class TestBenchTargets:
  def test_defaults(self, tmp_path, monkeypatch):
    monkeypatch.setattr(gp, "GATE_BENCH_DIR", tmp_path)
    assert gate_bench_lat_deg(_params(None)) == 150.0
    assert gate_bench_lon_mps2(_params(None)) == 0.0

  def test_overrides(self):
    assert gate_bench_lat_deg(_params(b"90")) == 90.0
    assert gate_bench_lon_mps2(_params(b"-1.5")) == -1.5


class TestGateBenchFallback:
  """Uncompiled key: Params raises UnknownKeyName, read /data/gate_bench/<key>."""

  def _unknown_params(self, name: str):
    params = MagicMock()
    params.get.side_effect = UnknownKeyName(name.encode())
    return params

  def test_bool_from_gate_bench(self, tmp_path, monkeypatch):
    monkeypatch.setattr(gp, "GATE_BENCH_DIR", tmp_path)
    (tmp_path / "GateSOC2Enabled").write_bytes(b"1")
    assert gate_soc2_enabled(self._unknown_params("GateSOC2Enabled"))

  def test_bool_missing_defaults_off(self, tmp_path, monkeypatch):
    monkeypatch.setattr(gp, "GATE_BENCH_DIR", tmp_path)
    assert not gate_soc2_enabled(self._unknown_params("GateSOC2Enabled"))

  def test_int_from_gate_bench(self, tmp_path, monkeypatch):
    monkeypatch.setattr(gp, "GATE_BENCH_DIR", tmp_path)
    (tmp_path / "GateSOC2Bus").write_bytes(b"0")
    assert gate_soc2_bus(self._unknown_params("GateSOC2Bus")) == 0

  def test_force_ignition(self, tmp_path, monkeypatch):
    monkeypatch.setattr(gp, "GATE_BENCH_DIR", tmp_path)
    (tmp_path / "GateForceIgnition").write_bytes(b"1")
    assert gate_force_ignition(self._unknown_params("GateForceIgnition"))
