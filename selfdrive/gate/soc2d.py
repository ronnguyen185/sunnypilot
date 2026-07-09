#!/usr/bin/env python3
"""Gate Protocol v2 daemon: carControl → gate CAN-FD frames on C3X gate bus."""

import time

from cereal import car
import cereal.messaging as messaging

from openpilot.common.params import Params
from openpilot.common.realtime import Priority, Ratekeeper, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.gate.emitter_loop import EmitterConfig, EmitterState, emitter_tick
from openpilot.selfdrive.gate.gate_encode import encode_accel_mps2, encode_steer_deg
from openpilot.selfdrive.gate.gate_params import (
  gate_bench_lat_deg,
  gate_bench_lon_mps2,
  gate_lat_delay_s,
  gate_soc2_bench_no_chassis,
  gate_soc2_bus,
)
from openpilot.selfdrive.gate.health import car_control_fresh, source_available
from openpilot.selfdrive.pandad import can_list_to_can_capnp

from opendbc.car.can_definitions import CanData


def car_control_to_emitter_config(
  cc: car.CarControl,
  fresh: bool,
  avail: bool,
  *,
  bus: int,
  lat_delay_s: float,
) -> EmitterConfig:
  ok = fresh and cc.valid
  lat_wanted = ok and cc.enabled and cc.latActive
  lon_wanted = ok and cc.enabled and cc.longActive
  # Protocol v2: int32 @ 0.001/LSB for degree and m/s^2.
  lat_target = encode_steer_deg(cc.actuators.steeringAngleDeg) if lat_wanted else 0
  lon_target = encode_accel_mps2(cc.actuators.accel) if lon_wanted else 0
  return EmitterConfig(
    bus=bus,
    lat_wanted=lat_wanted,
    lon_wanted=lon_wanted,
    lat_target=lat_target,
    lon_target=lon_target,
    lat_delay_s=lat_delay_s,
    source_available=avail,
  )


def bench_emitter_config(
  *,
  bus: int,
  lat_deg: float,
  lon_mps2: float,
  lat_delay_s: float,
) -> EmitterConfig:
  """Fixed-target stream matching soc2_cmd.sh (no carControl / chassis needed)."""
  return EmitterConfig(
    bus=bus,
    lat_wanted=True,
    lon_wanted=True,
    lat_target=encode_steer_deg(lat_deg),
    lon_target=encode_accel_mps2(lon_mps2),
    lat_delay_s=lat_delay_s,
    source_available=True,
  )


def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("soc2d is waiting for CarParams")
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("soc2d got CarParams: %s", CP.brand)

  gate_bus = gate_soc2_bus(params)
  lat_delay_s = gate_lat_delay_s(params, CP.steerActuatorDelay)
  bench_no_chassis = gate_soc2_bench_no_chassis(params)
  cloudlog.info(
    "soc2d gate bus=%d lat_delay_s=%.3f bench_no_chassis=%s",
    gate_bus, lat_delay_s, bench_no_chassis,
  )

  bench_mode = bench_no_chassis
  if bench_mode:
    cloudlog.warning(
      "GateSOC2BenchNoChassis: streaming fixed lat/lon from /data/gate_bench "
      "(reload every 1s). Bench only — not for driving",
    )

  pm = messaging.PubMaster(['sendcan'])
  sm = messaging.SubMaster(['carControl', 'carState', 'managerState'], poll='carControl')
  rk = Ratekeeper(100, print_delay_threshold=None)

  state = EmitterState()
  last_bench_reload = 0.0
  bench_cfg: EmitterConfig | None = None
  last_bench_key: tuple[float, float, float] | None = None

  while True:
    sm.update()
    rk.keep_time()

    now_mono = int(time.monotonic() * 1e9)
    now = time.monotonic()
    if bench_mode:
      # Hot-reload targets from /data/gate_bench (set_soc2_cmd.sh / enable_bench_soc2.sh).
      if bench_cfg is None or (now - last_bench_reload) >= 1.0:
        last_bench_reload = now
        bench_lat = gate_bench_lat_deg(params)
        bench_lon = gate_bench_lon_mps2(params)
        lat_delay_s = gate_lat_delay_s(params, CP.steerActuatorDelay)
        key = (bench_lat, bench_lon, lat_delay_s)
        if key != last_bench_key:
          cloudlog.info(
            "soc2d bench targets lat=%.1fdeg lon=%.2fm/s^2 lat_delay=%.3fs",
            bench_lat, bench_lon, lat_delay_s,
          )
          last_bench_key = key
        bench_cfg = bench_emitter_config(
          bus=gate_bus, lat_deg=bench_lat, lon_mps2=bench_lon, lat_delay_s=lat_delay_s,
        )
      cfg = bench_cfg
    else:
      fresh = car_control_fresh(sm, now_mono)
      avail = source_available(sm, now_mono, bench_no_chassis=False)
      cc = sm['carControl']
      cfg = car_control_to_emitter_config(
        cc, fresh, avail, bus=gate_bus, lat_delay_s=lat_delay_s,
      )

    sends = emitter_tick(state, cfg)
    can_msgs = [CanData(s.can_id, s.data, s.bus) for s in sends]
    pm.send('sendcan', can_list_to_can_capnp(can_msgs, msgtype='sendcan'))


if __name__ == "__main__":
  main()
