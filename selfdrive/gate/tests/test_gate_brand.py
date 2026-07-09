"""Tests for the comma Gate car brand (SOC2 link, no VinFast)."""

from unittest.mock import MagicMock

from opendbc.car.gate.interface import CarInterface
from opendbc.car.gate.values import CAR
from opendbc.car.structs import CarParams
from openpilot.system.manager import process_config


def test_gate_brand_params_use_soc2_gate_safety():
  CP = CarInterface.get_params(CAR.COMMA_GATE, {}, [], False, False, docs=False)
  assert CP.brand == "gate"
  assert CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.soc2Gate
  assert CP.steerControlType == CarParams.SteerControlType.angle
  assert CP.steerAtStandstill


def _params_enabled(bundle=None):
  params = MagicMock()

  def _get(key, *args, **kwargs):
    if key == "CarPlatformBundle":
      return bundle
    if key.startswith("Gate"):
      return b"1"
    return None

  params.get.side_effect = _get
  return params


def test_soc2d_runs_for_gate_brand_not_vinfast():
  gate_cp = MagicMock()
  gate_cp.brand = "gate"
  vinfast_cp = MagicMock()
  vinfast_cp.brand = "vinfast"

  params = _params_enabled()
  assert process_config.gate_soc2(True, params, gate_cp)
  assert not process_config.gate_soc2(True, params, vinfast_cp)


def test_soc2d_runs_with_comma_gate_bundle_before_fingerprint():
  empty_cp = MagicMock()
  empty_cp.brand = ""
  params = _params_enabled(bundle={"platform": "COMMA_GATE", "name": "comma Gate SOC2"})
  assert process_config.gate_soc2(True, params, empty_cp)
  assert not process_config.gate_soc2(False, params, empty_cp)
