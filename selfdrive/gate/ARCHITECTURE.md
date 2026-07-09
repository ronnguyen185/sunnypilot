# VinFast Safety Gate — Project Architecture

This document describes how the **VinFast Safety Gate** system is structured across
`gate-panda` (firmware + bench) and `openpilot` (comma 3X SOC2 client).

For operational steps (enable, disable, bench verify, troubleshooting), see [README.md](README.md).

---

## 1. Overview

The Safety Gate is a custom Panda STM32H7 firmware that sits between two independent
compute sources (SOC1 primary, SOC2 secondary) and the vehicle CAN bus. Neither SOC
touches the car directly — each streams a normalized gate protocol; the gate validates,
arbitrates, applies hard safety limits, and emits VinFast OEM actuation frames.

| Component | Location | Role |
|-----------|----------|------|
| Gate firmware | `/data/gate-panda/board/custom/` | Arbitration, safety envelope, OEM transcoder |
| SOC1 client | `/data/gate-panda/soc1/` | Reference primary source (C, gate bus 2) |
| SOC2 bench tools | `/data/gate-panda/gate-soc2/soc2/` | Bench emitter + verify (Python) |
| SOC2 onroad | `/data/openpilot/selfdrive/gate/` | `soc2d` daemon: `carControl` → gate frames |
| Panda safety | `opendbc/safety/modes/soc2_gate.h` | Blocks OEM TX on C3X; whitelists gate IDs |
| VinFast integration | `opendbc/car/vinfast/` | Car interface, carstate, gate bus config |

---

## 2. Physical topology

```text
                    ┌─────────────────────────────────────┐
                    │         gate-panda (STM32H7)        │
                    │                                     │
  SOC1 (CANable) ──►│ bus 2  PRIMARY                      │
                    │ bus 1  SECONDARY  ◄── C3X bus 2       │
                    │ bus 0  CAR        ──► VinFast chassis │
                    └─────────────────────────────────────┘
```

### Bus mapping (fixed in gate firmware)

| Gate panda bus | Role | Typical wiring |
|----------------|------|----------------|
| **0** | Real car / chassis | VinFast OEM harness |
| **1** | SOC2 secondary | C3X internal panda **bus 2** |
| **2** | SOC1 primary | CANable / SocketCAN bench |

Defined in `gate-panda/board/custom/main.c`:

```c
#define GATE_PRI_BUS 2U   // SOC1
#define GATE_SEC_BUS 1U   // SOC2
#define GATE_CAR_BUS 0U   // chassis
```

### C3X panda buses (openpilot)

| C3X bus | Normal VinFast | Gate mode (`GateSOC2Enabled=1`) |
|---------|----------------|----------------------------------|
| **0** | Chassis CAN → `carState` | Same — **required for onroad** |
| **1** | Radar (CAN-FD) | Radar (unchanged) |
| **2** | SCAM / camera | Gate SOC2 TX (`soc2d` → gate bus 1); bench harness may use **bus 0** via `GateSOC2Bus` |

---

## 3. Runtime data flow

### Onroad (full openpilot + gate)

```text
C3X bus 0 (chassis)
  → card → carState
    → controlsd → carControl
      → soc2d → sendcan (bus 2)
        → pandad → gate-panda bus 1
          → gate firmware → gate-panda bus 0 → car
```

- **Chassis feedback** on C3X bus 0 is required for real driving. `soc2d` sets
  `source_available=false` when `carState` is missing, invalid, or stale.
- **OEM actuation** from `card` is blocked by `soc2Gate` panda safety. Actuation
  goes only through the gate.
- The gate reads vehicle state from **its own bus 0** for safety envelope and MRM.
  SOC2 does not send vehicle state in the v0 protocol.

### Bench (no openpilot)

```text
soc2_emit.py → C3X bus 2 → gate-panda bus 1
verify_gate.py ← GATE_STATUS (0x500) on gate bus 1
```

Bench tests do not need chassis CAN on C3X bus 0.

---

## 4. Gate state machine

The gate runs a 4-state machine at 100 Hz (`gate-panda/board/custom/main.c`):

| State | Active source | Entry |
|-------|---------------|-------|
| `PRIMARY_ACTIVE` | SOC1 | Default; SOC1 healthy |
| `SECONDARY_ACTIVE` | SOC2 | SOC1 RED, SOC2 GREEN (300 ms dwell) |
| `MRM_ACTIVE` | Internal | Both sources RED, or `mrm_request` |
| `ISOLATED` | None | MRM finished |

SOC2 is a **hot standby**: it must stream healthy frames continuously even while SOC1
drives, or it cannot take over within the failover budget.

LED indicators (comma 3X gate panda):

| LED | Meaning |
|-----|---------|
| Green | Primary (SOC1) active |
| Blue | Secondary (SOC2) active |
| Red | MRM or isolated |

---

## 5. Gate Protocol v2

Normative spec: `/data/gate-panda/gate-soc2/docs/PROTOCOL_SPECS.md`

### SOC → Gate (SOC2 sends on C3X gate bus)

| Frame | CAN ID | Len | Rate | Content |
|-------|--------|-----|------|---------|
| `SOURCE_HEALTH` | `0x400` | 4 | 50 Hz | `health_counter`, `source_available` |
| `CMD_LAT` | `0x410` | 12 | 100 Hz | type+unit + int32 angle @ 0.001°/LSB |
| `CMD_LON` | `0x411` | 12 | 100 Hz | type+unit + int32 accel @ 0.001 m/s²/LSB |

- Transport: **CAN-FD**, 11-bit IDs, little-endian
- CRC: **CRC-16/CCITT-FALSE** (`poly 0x1021`, `init 0xFFFF`); CMD CRC covers bytes `[0..9]`
- `CMD_LAT` and `CMD_LON` must share the same `control_cycle_counter` each cycle
  and arrive within **15 ms** of each other

### Gate → SOC

| Frame | CAN ID | DLC | Rate | Content |
|-------|--------|-----|------|---------|
| `GATE_STATUS` | `0x500` | 16 | 50 Hz | gate state, pri/sec health, safety flags |
| `CAR_STATE` | `0x501` | 16 | 50 Hz | speed, steer, standstill (from car bus 0) |

### Gate freshness thresholds

| Check | Timeout |
|-------|---------|
| `SOURCE_HEALTH` | ≥ 120 ms → RED |
| `CMD_LAT` / `CMD_LON` | ≥ 60 ms → RED |
| LAT/LON pair window | > 15 ms → reject |

### VF6 target scaling

| Field | Source (`carControl`) | Encoding |
|-------|----------------------|----------|
| `lateral_target` | `actuators.steeringAngleDeg` | `round(deg / 0.001)` → 0.001°/LSB |
| `longitudinal_target` | `actuators.accel` | `round(m/s² / 0.001)` → 0.001 m/s²/LSB |

Active flags:

```text
lateral_active      = carControl.enabled && carControl.latActive
longitudinal_active = carControl.enabled && carControl.longActive
source_available    = processes healthy + fresh carControl + fresh carState (canValid)
```

---

## 6. openpilot module layout

```text
selfdrive/gate/
├── gate_params.py     # GateSOC2Bus, GateLatDelayS helpers
├── gate_crc.py        # CRC-16/CCITT-FALSE
├── gate_encode.py     # Build 0x400 / 0x410 / 0x411 payloads
├── emitter_loop.py    # Pure tick logic: 50 Hz health, 100 Hz LAT/LON, lat engage delay
├── health.py          # source_available: carState + carControl + process health
├── soc2d.py           # Onroad daemon (100 Hz), registered in process_config.py
├── tests/             # Unit tests (encoder, emitter, health, mapping)
├── README.md          # Detailed SOC2-side guide (enable, wiring, verify)
└── ARCHITECTURE.md    # Cross-repo architecture (this document)

opendbc_repo/opendbc/
├── car/gate/            # comma Gate brand (SOC2 link; not VinFast)
│   ├── interface.py     # gateLink safety + CAN-FD on GateSOC2Bus
│   ├── carstate.py      # Minimal carState (no OEM parsers)
│   └── fingerprints.py  # GATE_STATUS 0x500 fingerprint
├── car/vinfast/         # VinFast OEM (unchanged; no gate mode)
│   ├── interface.py
│   └── carstate.py
└── safety/modes/
    └── soc2_gate.h      # Whitelist used by SafetyModel.soc2Gate (@36)

system/manager/process_config.py   # soc2d process gate
common/params_keys.h               # GateSOC2Enabled, GateSOC2Bus, GateLatDelayS, GateSOC2BenchNoChassis
```

### Enable onroad SOC2

```bash
/data/openpilot/selfdrive/gate/enable_bench_soc2.sh   # sets COMMA_GATE brand + GateSOC2*
sudo reboot
```

The device fingerprints as **`COMMA_GATE`** (`brand=gate`), uses existing **`soc2Gate`**
panda safety (no rebuild), and runs `soc2d` when `GateSOC2Enabled=1`. VinFast is not on
the gate path.

---

## 7. gate-panda layout

```text
gate-panda/
├── board/custom/              # Safety Gate firmware
│   ├── main.c                 # State machine, arbitration, tick handler
│   ├── gate_protocol_defs.h   # CAN IDs, DLCs, timeouts, enums
│   ├── protocol.h             # v0 parsers + GATE_STATUS / CAR_STATE builders
│   ├── transcoder.h           # Gate cmd → VinFast OEM (0x37A, 0x32D, 0x131, 0x132)
│   ├── health_monitor.h       # Per-source health (GREEN/YELLOW/RED)
│   ├── safety_envelope.h      # Steer/accel/jerk clamps
│   ├── mrm.h                  # Minimal-risk maneuver
│   └── gate_can_rx.h          # CAN RX dispatch
│
├── soc1/                      # Reference SOC1 client (C)
│   ├── src/main.c             # SocketCAN → gate bus 2
│   └── include/gate_*.h       # Encoder + CRC (wire-identical to SOC2)
│
├── gate-soc2/
│   ├── docs/
│   │   ├── C3X_SOC2.md        # comma 3X integration guide
│   │   ├── PROTOCOL_SPECS.md  # Normative byte-exact wire format
│   │   └── SOC1_TO_GATE.md    # Design rationale
│   └── soc2/                  # Python bench tools
│       ├── soc2_emit.py       # Stream gate frames (openpilot stopped)
│       ├── verify_gate.py     # Listen for GATE_STATUS 0x500
│       ├── gate_encode.py     # Encoder (reference copy)
│       └── run_offline_tests.sh
│
├── scripts/                   # Bench helpers (emulate SOC1, transcoder, relay, …)
├── build.sh                   # Build panda_custom_canfd
├── flash_gate.sh              # Flash gate firmware
├── verify_gate.sh             # Pre-flash checks
└── check_gate_running.sh      # USB + CAN status verification
```

Build artifact: `board/obj/panda_custom_canfd.bin.signed`

---

## 8. Verification levels

### Level 1 — USB only (gate-panda host)

```bash
cd /data/gate-panda
./check_gate_running.sh --no-can
```

### Level 2 — Gate CAN status

```bash
./check_gate_running.sh --listen 5
# Expect GATE_STATUS (0x500) ~50 Hz on gate buses 1 and 2
```

### Level 3 — Bench SOC2 link (no car)

```bash
# C3X (openpilot stopped):
PYTHONPATH=/data/gate-panda/gate-soc2/soc2 python3 soc2_emit.py

# Gate-panda USB host:
PYTHONPATH=/data/gate-panda/gate-soc2/soc2 python3 verify_gate.py --bus 1
# Expect sec_health → GREEN
```

### Level 4 — Onroad openpilot

**Bench harness (C3X bus 0 ↔ gate bus 1):** `GATE_STATUS` (`0x500`) appears on **C3X bus 0**
at ~50 Hz once the gate is live. That is the same bus `GateSOC2Bus=0` uses for SOC2 TX, so
`listen_gate_status.py` / `verify_soc2d.py` need no extra bus flag.

**Onroad checklist:**

1. Stop bench tools that hold the panda (`read_bus.py`, `emulate_soc2_c3x.py`, `test_soc2_cmd.py --tx`).
2. Set params + clear offroad lock:

```bash
/data/openpilot/selfdrive/gate/go_onroad_bench_soc2.sh   # or enable_bench_soc2.sh
sudo reboot
```

3. **Ignition ON** — `hardwared` sets `deviceState.started` only when `ignitionLine` or
   `ignitionCan` is true (and `OffroadMode=0`, training/terms, etc.).
4. Confirm processes: `manager`, `pandad`, `card`, `controlsd`, `selfdrived`, **`soc2d`**.
5. Verify stream (~10 s after onroad; `vinfast/interface.py` configures CAN-FD on `GateSOC2Bus`
   after a 10 s delay):

```bash
python3 /data/openpilot/selfdrive/gate/test_soc2_cmd.py --observe --lat 150 --lon 0 --lat-delay-s 3 --seconds 10 --listen --require
python3 /data/openpilot/selfdrive/gate/verify_soc2d.py
```

Expect: `sec_health=GREEN`, often `SECONDARY_ACTIVE` with SOC1 absent; `CMD_LAT`/`CMD_LON` on
bus 0 via cereal `sendcan`.

**Production harness:** `GateSOC2Bus=2`, disable `GateSOC2BenchNoChassis`; chassis required
on C3X bus 0 for real `carControl` / `source_available`.

### Unit tests

```bash
cd /data/openpilot
pytest selfdrive/gate/tests/

cd /data/gate-panda/gate-soc2/soc2
./run_offline_tests.sh
```

---

## 9. Key design decisions

1. **Wire protocol is identical for SOC1 and SOC2.** Only the physical bus differs
   (gate bus 2 vs gate bus 1).

2. **SOC2 does not send vehicle state.** The gate owns car-bus parsing and safety
   envelope. SOC2 sends intent (`CMD_LAT`/`CMD_LON`) and availability (`SOURCE_HEALTH`).

3. **openpilot chassis feedback is still required onroad.** `carState` from C3X bus 0
   feeds `controlsd` and gates `source_available`. Without it, the gate will not trust
   SOC2 commands.

4. **Bus 2 is repurposed in gate mode.** The SCAM/cam parser is disabled; `canValid`
   is based on chassis bus 0 only.

5. **Actuation path is gate-only.** `soc2Gate` safety blocks VinFast OEM TX from
   `card` on C3X bus 0. All actuation goes through the gate transcoder.

---

## 10. Reference documents

| Document | Path |
|----------|------|
| SOC2 operational guide | `openpilot/selfdrive/gate/README.md` |
| C3X SOC2 integration | `gate-panda/gate-soc2/docs/C3X_SOC2.md` |
| Protocol spec (normative) | `gate-panda/gate-soc2/docs/PROTOCOL_SPECS.md` |
| SOC1 design rationale | `gate-panda/gate-soc2/docs/SOC1_TO_GATE.md` |
| Gate firmware overview | `gate-panda/SAFETY_GATE.md` |
| Gate quick start | `gate-panda/README.md` |
