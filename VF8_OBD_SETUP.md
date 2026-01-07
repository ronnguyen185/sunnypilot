# VF8 OBD Mode Setup Guide

This guide explains how to use Panda in OBD mode with CAN3 connected to the OBD port on a VinFast VF8.

## Overview

When OBD mode is enabled:
- **CAN3 is multiplexed** to use CAN2's transceiver
- Messages from CAN3 appear on **bus 1** (not bus 2)
- This allows accessing the OBD port through CAN3

## Hardware Connection

1. Connect CAN3 to the OBD port on your VF8
2. Ensure proper wiring (CAN-H, CAN-L, GND, 12V)

## VF8 Bus Configuration

VF8 uses the following bus assignments:
- **Bus 0**: SCAM/Camera bus
- **Bus 1**: Radar bus (CAN-FD) - **OR OBD when OBD mode enabled**
- **Bus 2**: Chassis bus

When OBD mode is enabled, bus 1 is used for OBD communication.

## Quick Start

### Using the Test Script (elm327 Safety Mode - Recommended)

```bash
cd /data/openpilot
python panda/examples/vf8_obd_elm327_test.py
```

Or the original script:
```bash
python panda/examples/vf8_obd_test.py
```

The elm327 safety mode script will:
1. Set elm327 safety mode with OBD enabled (param=0)
2. Automatically enable OBD mode (no need to call set_obd separately)
3. Validate ISO 15765-4 (OBD-II) protocol messages
4. Set CAN speed to 500 kbps (OBD-II standard)
5. Monitor messages from the OBD port
6. Display statistics and sample messages

**Why elm327 mode?**
- Automatically enables OBD mode when `safety_param=0`
- Validates OBD-II protocol messages (ISO 15765-4)
- Safer than `allOutput` mode - only allows valid OBD messages
- Used by openpilot for firmware queries and diagnostics

### Manual Setup (Python)

**Option 1: Using elm327 Safety Mode (Recommended)**
```python
from panda import Panda
from opendbc.car.structs import CarParams

p = Panda()
p.reset()

# Use elm327 safety mode with param=0 to enable OBD mode
# param=0: Enables OBD mode automatically
# param=1: Disables OBD mode (normal CAN)
p.set_safety_mode(CarParams.SafetyModel.elm327, 0)

# Set CAN speed for OBD (500 kbps is standard)
p.set_can_speed_kbps(1, 500)  # Bus 1 = CAN3 in OBD mode
p.set_can_enable(1, True)

# Read messages
while True:
    msgs = p.can_recv()
    for addr, dat, bus in msgs:
        if bus == 1:  # OBD bus
            print(f"OBD: 0x{addr:03X} = {dat.hex()}")
```

**Option 2: Manual OBD Mode (Alternative)**
```python
from panda import Panda
from opendbc.car.structs import CarParams

p = Panda()
p.reset()

# Set safety mode (allOutput for testing, or elm327 for validation)
p.set_safety_mode(CarParams.SafetyModel.allOutput)  # or elm327

# Manually enable OBD mode
p.set_obd(True)

# Set CAN speed for OBD (500 kbps is standard)
p.set_can_speed_kbps(1, 500)  # Bus 1 = CAN3 in OBD mode
p.set_can_enable(1, True)

# Read messages
while True:
    msgs = p.can_recv()
    for addr, dat, bus in msgs:
        if bus == 1:  # OBD bus
            print(f"OBD: 0x{addr:03X} = {dat.hex()}")
```

### Using can_printer.py

```bash
# Set CAN environment variable to 3, which enables OBD mode
CAN=3 python panda/scripts/can_printer.py
```

When `CAN=3`, the script automatically:
- Enables OBD mode (`p.set_obd(True)`)
- Monitors bus 1 (where CAN3 messages appear)

## Important Notes

1. **Bus Numbering**: When OBD mode is enabled, CAN3 messages appear on **bus 1**, not bus 2
2. **CAN Speed**: OBD-II typically uses 500 kbps, but some vehicles may use different speeds
3. **Safety Mode Options**:
   - **elm327 with param=0** (Recommended): Automatically enables OBD mode and validates ISO 15765-4 messages
   - **elm327 with param=1**: Normal CAN mode (OBD disabled)
   - **allOutput**: Allows all messages (for testing only)
   - **noOutput**: Safe mode, no messages sent
4. **VF8 Radar**: When OBD mode is enabled, bus 1 is used for OBD, so radar (normally on bus 1) will be unavailable
5. **elm327 Mode Benefits**:
   - Automatically enables OBD mode when `param=0`
   - Validates OBD-II protocol (ISO 15765-4)
   - Only allows valid OBD diagnostic addresses
   - Used by openpilot for firmware queries

## Disabling OBD Mode

```python
p.set_obd(False)  # Disable OBD mode
```

## Troubleshooting

### No messages received
- Check OBD port connection
- Verify CAN speed (try 500 kbps, 250 kbps, or 125 kbps)
- Ensure OBD mode is enabled: `p.set_obd(True)`
- Check that bus 1 is enabled: `p.set_can_enable(1, True)`

### Wrong bus number
- Remember: CAN3 messages appear on **bus 1** when OBD mode is enabled
- Use `bus == 1` to filter OBD messages

### VF8 specific issues
- VF8 uses bus 1 for radar (CAN-FD) normally
- When OBD mode is enabled, radar on bus 1 will be unavailable
- Consider using a different bus for radar if needed

## Integration with openpilot

To use OBD mode with openpilot for VF8:

1. The `obd_callback` in `selfdrive/car/card.py` handles OBD multiplexing
2. OBD mode is automatically enabled/disabled during firmware queries
3. For continuous OBD access, you may need to modify the car interface

## Example: Reading OBD PIDs

```python
from panda import Panda
from opendbc.car.structs import CarParams
import struct

p = Panda()
p.reset()

# Use elm327 mode with OBD enabled
p.set_safety_mode(CarParams.SafetyModel.elm327, 0)
p.set_can_speed_kbps(1, 500)
p.set_can_enable(1, True)
p.can_clear(0xFFFF)

# Request Mode 01 PID 0D (Vehicle Speed)
# OBD-II request format: [0x02, 0x01, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00]
speed_request = bytes([0x02, 0x01, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00])
p.can_send(0x7DF, speed_request, 1)  # 0x7DF is OBD broadcast address

# Read response
import time
for _ in range(50):  # Wait up to 0.5 seconds
    msgs = p.can_recv()
    for addr, dat, bus in msgs:
        if bus == 1 and 0x7E8 <= addr <= 0x7EF:  # OBD response addresses
            print(f"OBD Response from 0x{addr:03X}: {dat.hex()}")
            # Parse response (simplified)
            if len(dat) >= 3:
                mode = dat[0] & 0x3F
                pid = dat[1]
                value = dat[2]
                print(f"  Mode: {mode:02X}, PID: {pid:02X}, Value: {value} km/h")
    time.sleep(0.01)
```

Or use the example script:
```bash
python panda/examples/vf8_obd_elm327_test.py query
```

## References

- Panda OBD mode: `panda/board/main_comms.h` (command 0xdb)
- VF8 car interface: `opendbc_repo/opendbc/car/vinfast/`
- OBD callback: `selfdrive/car/card.py` (obd_callback function)

