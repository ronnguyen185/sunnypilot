"""Gate SOC2 config helpers.

On a prebuilt release the GateSOC2* keys are not in the running params binary
(UnknownKeyName). Writing them under /data/params/d/ is unsafe: Params' async
writer periodically rewrites that directory from the known-key set and deletes
unknown files within ~1s.

Bench config therefore lives in /data/gate_bench/<key> (plain files). When the
key is compiled into Params, we still prefer Params.get().
"""

from __future__ import annotations

from pathlib import Path

from opendbc.car.gate.values import CANBUS
from openpilot.common.params import Params, UnknownKeyName

DEFAULT_GATE_SOC2_BUS = CANBUS.gate
DEFAULT_BENCH_LAT_DEG = 150.0
DEFAULT_BENCH_LON_MPS2 = 0.0

# Stable location outside Params' managed directory.
GATE_BENCH_DIR = Path("/data/gate_bench")


def gate_bench_path(name: str) -> Path:
  return GATE_BENCH_DIR / name


def _raw(params: Params | None, name: str) -> bytes | None:
  """Raw value: Params if known, else /data/gate_bench/<name>."""
  if params is None:
    params = Params()
  try:
    val = params.get(name, return_default=True)
  except UnknownKeyName:
    val = None
  if val is None or val == b"":
    try:
      val = gate_bench_path(name).read_bytes()
    except OSError:
      return None
  if val is None or val == b"":
    return None
  return val if isinstance(val, bytes) else str(val).encode()


def _bool(params: Params | None, name: str) -> bool:
  raw = _raw(params, name)
  return raw is not None and raw.strip() in (b"1", b"true", b"True")


def _int(params: Params | None, name: str, default: int) -> int:
  raw = _raw(params, name)
  if raw is None:
    return default
  try:
    return int(raw)
  except ValueError:
    return default


def _float(params: Params | None, name: str, default: float) -> float:
  raw = _raw(params, name)
  if raw is None:
    return default
  try:
    return float(raw)
  except ValueError:
    return default


def write_gate_bench(name: str, val: str) -> Path:
  """Persist a gate bench setting (no Params rebuild required)."""
  GATE_BENCH_DIR.mkdir(parents=True, exist_ok=True)
  path = gate_bench_path(name)
  path.write_bytes(val.encode())
  return path


def gate_soc2_enabled(params: Params | None = None) -> bool:
  """Master switch for soc2d + soc2Gate safety."""
  return _bool(params, "GateSOC2Enabled")


def gate_soc2_bus(params: Params | None = None) -> int:
  """C3X panda bus for gate SOC2 TX (default 2; bench harness often 0)."""
  return _int(params, "GateSOC2Bus", DEFAULT_GATE_SOC2_BUS)


def gate_soc2_bench_no_chassis(params: Params | None = None) -> bool:
  """Bench link only: assert source_available without C3X chassis CAN on bus 0."""
  return _bool(params, "GateSOC2BenchNoChassis")


def gate_force_ignition(params: Params | None = None) -> bool:
  """Desk bench: treat ignition as ON without harness/jungle."""
  return _bool(params, "GateForceIgnition")


def gate_force_engage(params: Params | None = None) -> bool:
  """One-shot engage request for gate brand (no cruise/LKAS buttons)."""
  return _bool(params, "GateForceEngage")


def gate_lat_delay_s(params: Params, steer_actuator_delay: float) -> float:
  """Lateral engage delay; GateLatDelayS overrides CarParams.steerActuatorDelay when > 0."""
  override = _float(params, "GateLatDelayS", 0.0)
  return override if override > 0.0 else steer_actuator_delay


def gate_bench_lat_deg(params: Params | None = None) -> float:
  """Fixed lateral target (deg) streamed in bench mode (matches soc2_cmd.sh --lat)."""
  return _float(params, "GateSOC2BenchLat", DEFAULT_BENCH_LAT_DEG)


def gate_bench_lon_mps2(params: Params | None = None) -> float:
  """Fixed longitudinal target (m/s^2) streamed in bench mode (matches soc2_cmd.sh --lon)."""
  return _float(params, "GateSOC2BenchLon", DEFAULT_BENCH_LON_MPS2)
