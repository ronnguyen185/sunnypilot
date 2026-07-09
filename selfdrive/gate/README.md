# SOC2 — comma 3X Secondary Gate Client

This document describes the **comma 3X (SOC2) side** of the VinFast Safety Gate:
how openpilot streams gate protocol frames to the gate panda, how to enable it, and
how to verify the link on bench and onroad.

For gate firmware, arbitration, and cross-repo layout see [ARCHITECTURE.md](ARCHITECTURE.md).
For byte-exact wire format see `/data/gate-panda/gate-soc2/docs/PROTOCOL_SPECS.md`.

---

## 1. Role of SOC2

The Safety Gate sits between two compute sources and the car:

```text
gate-panda bus 2  ←  SOC1 (PRIMARY)     CANable / bench client
gate-panda bus 1  ←  SOC2 (SECONDARY)    comma 3X + openpilot   ← this doc
gate-panda bus 0  →  VinFast chassis
```

SOC2 is a **hot standby**. The gate normally drives the car from SOC1 (`PRIMARY_ACTIVE`).
SOC2 only actuates after failover (`SECONDARY_ACTIVE` when SOC1 is RED and SOC2 is GREEN).

Neither SOC touches the car CAN directly. SOC2 tells the gate only:

1. **Am I available?** → `SOURCE_HEALTH` (`0x400`)
2. **What is my control intent?** → `CMD_LAT` (`0x410`) + `CMD_LON` (`0x411`)

The gate validates, arbitrates, applies safety limits, and emits VinFast OEM frames
(e.g. EPS `0x37A`) on bus 0.

**Critical:** SOC2 must stream valid frames continuously while onroad and gate mode is
enabled — even when not the active source — or it cannot take over within the failover
budget (~300 ms dwell after SOC1 goes RED).

---

## 2. Physical wiring

### Gate panda (fixed in firmware)

| Gate bus | Role |
|----------|------|
| 0 | Real car / chassis |
| 1 | SOC2 secondary (listens here) |
| 2 | SOC1 primary |

### C3X internal panda

| C3X bus | Normal VinFast | Gate mode (`GateSOC2Enabled=1`) |
|---------|----------------|----------------------------------|
| **0** | Chassis → `carState` | Same — **required onroad** for `source_available` |
| 1 | Radar (CAN-FD) | Unchanged |
| **GateSOC2Bus** | SCAM / camera (bus 2) | Gate SOC2 TX → gate bus 1 |

**Bench harness (verified):** wire C3X **bus 0** → gate **bus 1**, set `GateSOC2Bus=0`.

**Production harness:** wire C3X **bus 2** → gate **bus 1**, leave `GateSOC2Bus=2` (default).

```text
                    ┌──────────────── gate-panda ────────────────┐
  C3X GateSOC2Bus ──┤ bus 1  SECONDARY (SOC2)                    │
  (0 bench / 2 prod)│ bus 2  PRIMARY   (SOC1)                    │
                    │ bus 0  CAR ──────────────► VinFast chassis │
                    └────────────────────────────────────────────┘
  C3X bus 0 (chassis) ──► card ──► carState ──► controlsd ──► carControl
```

Link is **CAN-FD 500 kbps / 2 Mbps** (ISO, BRS). Configured by `vinfast/interface.py`
~10 s after onroad start when gate mode is enabled.

---

## 3. Onroad data flow

```text
controlsd
  → carControl (cereal)
    → soc2d (100 Hz)
      → encode SOURCE_HEALTH + CMD_LAT + CMD_LON
        → sendcan on GateSOC2Bus
          → pandad
            → gate-panda bus 1
              → gate firmware → bus 0 → car
```

Parallel path for feedback (required onroad):

```text
C3X bus 0 (chassis) → card → carState → controlsd / soc2d health
```

When gate mode is on:

- **`soc2d`** is the only process that may send gate protocol frames (`0x400`/`0x410`/`0x411`).
- **`card`** still reads chassis CAN but **OEM actuation TX is blocked** by `soc2Gate` panda safety.
- All steering/accel to the car goes through the gate transcoder, not direct VinFast CAN from C3X.

---

## 4. Enable and disable

### Enable (one master switch + optional tuning)

```bash
# Master enable — soc2d + soc2Gate safety
echo -n 1 > /data/params/d/GateSOC2Enabled

# Bench: C3X bus wired to gate (not default bus 2)
echo -n 0 > /data/params/d/GateSOC2Bus

# Bench: no chassis on C3X bus 0 (gate bus 0 still wired to car)
echo -n 1 > /data/params/d/GateSOC2BenchNoChassis

# Optional: lateral engage delay in seconds (EPS neutral preamble)
# 0 = use CarParams.steerActuatorDelay (~0.05 s)
echo -n 3 > /data/params/d/GateLatDelayS

sudo reboot    # or offroad → onroad (CarParams must reload)
```

### Disable (single param)

```bash
echo -n 0 > /data/params/d/GateSOC2Enabled
sudo reboot
```

This stops `soc2d`, restores normal `vinfast` panda safety, and allows OEM actuation
from `card` again. `GateSOC2Bus` and `GateLatDelayS` are ignored while disabled.

### Params reference

| Param | Default | Description |
|-------|---------|-------------|
| `GateSOC2Enabled` | `0` | **Master switch.** `1` = run `soc2d` + `soc2Gate` safety |
| `GateSOC2Bus` | `2` | C3X panda bus index for gate TX (`0` on many benches) |
| `GateLatDelayS` | `0` | Seconds to defer `CMD_LAT` after engage; `0` = `steerActuatorDelay` |
| `GateSOC2BenchNoChassis` | `0` | **Bench only.** `1` = bench stream mode: fixed targets, `source_available=1`, no chassis needed |
| `GateSOC2BenchLat` | `150` | **Bench only.** Fixed lateral target (deg) streamed in bench mode |
| `GateSOC2BenchLon` | `0` | **Bench only.** Fixed longitudinal target (m/s²) streamed in bench mode |

**Bench stream mode (`GateSOC2BenchNoChassis=1`):** `soc2d` ignores `carControl` and
streams a **fixed** `CMD_LAT`/`CMD_LON` target with `source_available=1`, mirroring
`./soc2_cmd.sh --lat <BenchLat> --lon <BenchLon> --lat-delay-s <GateLatDelayS>`.
This needs no C3X chassis CAN — bench only, **not for driving.** Disable it for real driving,
where active CMD_LAT/LON require fresh/valid `carControl`.

> **Params not registering?** New `GateSOC2*` keys only exist in the running binary after a
> `scons` rebuild + reboot. Until then, `gate_params.py` falls back to reading the raw file
> under `/data/params/d/<key>`, so `echo -n ... > /data/params/d/<key>` still takes effect.

---

## 5. What soc2d sends

### Frame rates and IDs

| Frame | CAN ID | Len | Rate | Counter |
|-------|--------|-----|------|---------|
| `SOURCE_HEALTH` | `0x400` | 4 | 50 Hz | `health_counter` |
| `CMD_LAT` | `0x410` | 12 (CAN-FD DLC 9) | 100 Hz | `control_cycle_counter` (shared with LON) |
| `CMD_LON` | `0x411` | 12 (CAN-FD DLC 9) | 100 Hz | same as LAT each tick |

- Transport: CAN-FD, 11-bit IDs, little-endian, CRC-16/CCITT-FALSE (CMD CRC covers bytes `[0..9]`).
- Protocol **v2**: CMD frames carry `req_type` + `target_unit` + **int32** target.
- LAT and LON must share the same `control_cycle_counter` every tick and arrive within **15 ms** of each other (gate pairs them strictly).

### Mapping from carControl

| Gate field | Source | Encoding |
|------------|--------|----------|
| Lateral target | `actuators.steeringAngleDeg` | `round(deg / 0.001)` → **0.001°/LSB** (type=`angle`, unit=`degree`) |
| Longitudinal target | `actuators.accel` | `round(m/s² / 0.001)` → **0.001 m/s²/LSB** (type=`accel`, unit=`m/s²`) |
| Lateral wanted | `enabled && latActive` | deferred by `GateLatDelayS` |
| Longitudinal wanted | `enabled && longActive` | immediate on engage |
| `source_available` | see §6 | bit in `SOURCE_HEALTH` |

When `carControl` is stale or invalid, both domains send **inactive** flags (`active=0`,
`REQ_NONE`, target `0`) but frames still stream so gate counters/timeouts stay healthy.

### Lateral engage delay (EPS preamble)

On engage, SOC2 matches SOC1/bench behavior:

1. **LON** goes active immediately (`LON_REQ_ACCEL` + accel target).
2. **LAT** stays inactive (`LAT_REQ_NONE`, target `0`) for `GateLatDelayS`.
3. After delay, **LAT** goes active (`LAT_REQ_ANGLE` + steer target).

Gate firmware accepts inactive LAT + active LON (`gate_pair_policy.h`) and emits a
neutral EPS `0x37A` preamble (AOLAct=0) before lateral arms — required for reliable
EPS engagement.

---

## 6. source_available (when the gate trusts SOC2)

`SOURCE_HEALTH.source_available` is set by `health.py` and must be `1` for the gate
to actuate SOC2 commands. All of the following must be true:

| Check | Threshold |
|-------|-----------|
| `carControl` alive + valid | — |
| `carControl` fresh | &lt; 50 ms |
| `carState` alive + valid + `canValid` | — |
| `carState` fresh | &lt; 100 ms |
| Required processes running | `controlsd`, `card`, `selfdrived`, `soc2d` |

**Bench-only note:** standalone emitters (`soc2_cmd.sh`, `emulate_soc2_c3x.py`) do not
need chassis CAN; they always send `source_available=1` unless `--unavailable` is used.
Onroad openpilot **requires** chassis on C3X bus 0.

---

## 7. Gate health (what to expect on verify)

Gate broadcasts `GATE_STATUS` (`0x500`) on buses 1 and 2. Key fields for SOC2:

| Field | Healthy standby | Failover driving |
|-------|-----------------|----------------|
| `sec_health` | `GREEN` | `GREEN` |
| `gate_state` | `PRIMARY_ACTIVE` (SOC1 up) | `SECONDARY_ACTIVE` |
| `active_source` | `PRIMARY` | `SECONDARY` |

Gate freshness (forces RED if violated):

| Stream | Timeout |
|--------|---------|
| `SOURCE_HEALTH` | ≥ 120 ms |
| `CMD_LAT` / `CMD_LON` | ≥ 60 ms |

---

## 8. openpilot module layout

```text
selfdrive/gate/
├── soc2d.py           # Onroad daemon (100 Hz); carControl or bench stream → gate frames
├── health.py          # source_available aggregation
├── emitter_loop.py    # Tick logic: rates, counters, lat engage delay
├── gate_encode.py     # 0x400 / 0x410 / 0x411 builder (wire-identical to soc1 C)
├── gate_crc.py        # CRC-16/CCITT-FALSE
├── gate_params.py     # GateSOC2* helpers (+ on-disk fallback for pre-rebuild)
├── gate_status.py     # GATE_STATUS 0x500 decode (parity with gate-panda)
├── enable_bench_soc2.sh  # One-shot bench param setup (parity with soc2_cmd.sh)
├── soc2_cmd_test.sh   # Wrapper: test openpilot-stack SOC2 cmd (→ test_soc2_cmd.py)
├── test_soc2_cmd.py   # Observe/verify soc2d sendcan vs soc2_cmd --lat/--lon/--lat-delay-s
├── verify_soc2d.py    # TX frame rates + GATE_STATUS RX + LAT engage delay
├── listen_gate_status.py  # GATE_STATUS listener on C3X panda (bench / offroad)
└── tests/             # pytest (encoder, emitter, health, mapping, params, status)

opendbc/car/vinfast/
├── interface.py       # soc2Gate safety, CAN-FD on GateSOC2Bus
└── carstate.py        # Chassis-only canValid in gate mode

opendbc/safety/modes/
└── soc2_gate.h        # TX whitelist on GateSOC2Bus; blocks OEM actuation on bus 0

system/manager/process_config.py   # soc2d start condition
common/params_keys.h               # GateSOC2* params
```

---

## 9. Verification

### A. Unit tests (no hardware)

```bash
cd /data/openpilot
pytest selfdrive/gate/tests/

cd /data/gate-panda/gate-soc2/soc2
./run_offline_tests.sh
```

Safety tests (requires built `libsafety.so`):

```bash
cd /data/openpilot/opendbc_repo/opendbc/safety/tests
scons -j$(nproc) -D
pytest test_soc2_gate.py
```

### B. Bench link (openpilot stopped)

Use when bringing up wiring before enabling onroad `soc2d`:

```bash
# C3X — stop openpilot first
export C3X_SOC2_BUS=0
cd /data/gate-panda/gate-soc2
./soc2_cmd.sh --lat 150 --lon 0 --lat-delay-s 3 --listen
```

```bash
# Gate host
cd /data/gate-panda/gate-soc2
./wake_bus1.sh --reset --listen
```

Expect `GATE_STATUS`: `SECONDARY_ACTIVE`, `sec_health=GREEN` (with SOC1 absent/red).

### C. Onroad soc2d (openpilot running)

**With chassis on C3X bus 0** (production / full stack):

```bash
echo -n 1 > /data/params/d/GateSOC2Enabled
echo -n 0 > /data/params/d/GateSOC2Bus    # or 2 for production harness
sudo reboot
```

**Bench stream mode** (no C3X chassis — mirrors `soc2_cmd.sh --lat 150 --lon 0 --lat-delay-s 3`):

```bash
# Sets GateSOC2Enabled/BenchNoChassis/Bus=0/LatDelayS=3/BenchLat=150/BenchLon=0 + OffroadMode=0.
# Overrides: ./enable_bench_soc2.sh GateSOC2BenchLat=90 GateLatDelayS=2
/data/openpilot/selfdrive/gate/enable_bench_soc2.sh
# then, if GateSOC2* were newly added:
cd /data/openpilot && scons -j$(nproc)
sudo reboot
```

Equivalent by hand:

```bash
echo -n 1 > /data/params/d/GateSOC2Enabled
echo -n 1 > /data/params/d/GateSOC2BenchNoChassis
echo -n 0 > /data/params/d/GateSOC2Bus
echo -n 3 > /data/params/d/GateLatDelayS
sudo reboot
```

1. Go onroad (VinFast fingerprint or cached `CarParams`).
2. Confirm `soc2d` in process list / logs (`bench_no_chassis=True`, `streaming fixed lat=...`).
3. **While still offroad** — test openpilot gate modules alone (no soc2d/onroad):

```bash
# Same intent as: ./soc2_cmd.sh --lat 150 --lon 0 --lat-delay-s 3
python3 /data/openpilot/selfdrive/gate/test_soc2_cmd.py --lat 150 --lon 0 --lat-delay-s 3 -v

# Optional: TX on C3X panda (stop openpilot/pandad first)
sudo systemctl stop comma   # or otherwise stop pandad
python3 /data/openpilot/selfdrive/gate/test_soc2_cmd.py --tx --lat 150 --lon 0 --lat-delay-s 3 --duration 10 --listen
```

4. Verify on the C3X once onroad (TX rates + payload vs `soc2_cmd.sh`):

```bash
python3 /data/openpilot/selfdrive/gate/test_soc2_cmd.py --observe --lat 150 --lon 0 --lat-delay-s 3 --seconds 10 --listen --require
```

   If onroad `can` is unavailable, listen directly on the C3X panda (openpilot stopped):

```bash
python3 /data/openpilot/selfdrive/gate/listen_gate_status.py
```

   Or on the gate USB host (same decode as gate-panda):

```bash
PYTHONPATH=/data/gate-panda/gate-soc2/soc2 python3 verify_gate.py --bus 1
```

Expect `GATE_STATUS`: `sec_health=GREEN`, and with SOC1 absent/red often `SECONDARY_ACTIVE`.

4. Bench stream mode drives fixed targets — no `carControl` / chassis needed. Disable
   `GateSOC2BenchNoChassis` for real driving, where active CMD_LAT/LON need valid `carControl`.
5. Failover test: stop SOC1 or make it RED → gate `SECONDARY_ACTIVE`.

### D. Quick link test (bus index)

```bash
# C3X: listen on suspected bus
cd /data/gate-panda/gate-soc2/soc2
python3 read_bus.py --bus 0 --filter 0x123

# Gate: ping on bus 1
cd /data/gate-panda
./send_bus1.sh --addr 0x123 --hz 10
```

If no RX on bus 0, try `--bus 1` or `--bus 2`, then set `GateSOC2Bus` / `C3X_SOC2_BUS` accordingly.

---

## 10. Troubleshooting

| Symptom | Likely cause | What to check |
|---------|--------------|---------------|
| `sec_health=RED` | No frames ACKed on gate bus 1 | `GateSOC2Bus` matches harness; gate `./wake_bus1.sh --reset` |
| `sec_health=RED` onroad | Stale/missing chassis | C3X bus 0 wired; or `GateSOC2BenchNoChassis=1` for bench |
| `source_available=0` | Health aggregator failed | `soc2d`, `card`, `controlsd`, `selfdrived` running; fresh `carState` |
| No `soc2d` process | Param off or wrong brand | `GateSOC2Enabled=1`, VinFast onroad |
| OEM steer still from C3X | Old CarParams / safety | Reboot after param change; confirm `soc2Gate` in logs |
| Engage but no EPS | Lat delay / pair rejected | LON must be active during lat delay; check `GateLatDelayS` |
| BUS_OFF on gate bus 1 | Link order / termination | Start C3X stream first, then gate reset |

Logs: `soc2d` prints `gate bus=` and `lat_delay_s=` at startup in cloudlog.

---

## 11. Bench tools (gate-panda, openpilot stopped)

| Tool | Purpose |
|------|---------|
| `gate-soc2/soc2_cmd.sh` | SOC1-compatible CLI: `--lat`, `--lon`, `--lat-delay-s` |
| `gate-soc2/emulate_soc2_c3x.py` | Self-contained C3X emitter (inlined encoder) |
| `gate-soc2/soc2/soc2_emit.py` | Emitter using shared `emitter_loop` |
| `gate-soc2/soc2/gate_status.py` | GATE_STATUS decode (same wire format as openpilot `gate_status.py`) |
| `gate-soc2/soc2/verify_gate.py` | Listen for `GATE_STATUS` on gate USB host |
| `selfdrive/gate/listen_gate_status.py` | Same, on C3X SPI panda (bench / offroad) |
| `selfdrive/gate/verify_soc2d.py` | Onroad: sendcan TX + cereal `can` GATE_STATUS |
| `gate-soc2/wake_bus1.sh` | Hold gate bus 1 alive, recover BUS_OFF |

Bench default bus: `export C3X_SOC2_BUS=0` (software index may differ from harness label).

---

## 12. Related documents

| Document | Path |
|----------|------|
| Cross-repo architecture | [ARCHITECTURE.md](ARCHITECTURE.md) |
| C3X integration guide | `/data/gate-panda/gate-soc2/docs/C3X_SOC2.md` |
| Wire format (normative) | `/data/gate-panda/gate-soc2/docs/PROTOCOL_SPECS.md` |
| Design rationale | `/data/gate-panda/gate-soc2/docs/SOC1_TO_GATE.md` |
| Gate firmware overview | `/data/gate-panda/SAFETY_GATE.md` |
