#!/usr/bin/env python3
"""
VF8 OBD Mode Test Script using elm327 Safety Mode
Uses elm327 safety mode which automatically enables OBD mode and validates ISO 15765-4 messages

Usage:
    python panda/examples/vf8_obd_elm327_test.py
"""

import time
from collections import defaultdict
import binascii

from opendbc.car.structs import CarParams
from panda import Panda


def vf8_obd_elm327_test():
    """Test OBD mode on CAN3 for VF8 using elm327 safety mode"""
    print("Connecting to Panda...")
    p = Panda()
    print(f"Connected to Panda: {p.get_serial()[0]} - {p.get_version()}")
    
    # Reset panda
    p.reset()
    time.sleep(1)
    
    # Use elm327 safety mode with param=0 to enable OBD mode
    # elm327 mode automatically enables OBD multiplexing when param=0
    # This is safer than allOutput and validates ISO 15765-4 (OBD-II) messages
    print("\nSetting elm327 safety mode with OBD enabled (param=0)...")
    print("  - param=0: Enables OBD mode (CAN3 -> OBD port)")
    print("  - param=1: Disables OBD mode (normal CAN mode)")
    p.set_safety_mode(CarParams.SafetyModel.elm327, 0)
    print("✓ elm327 safety mode enabled with OBD mode!")
    print("  Note: When OBD mode is enabled, CAN3 messages appear on bus 1")
    print("  elm327 mode validates ISO 15765-4 (OBD-II) protocol messages")
    
    # Set CAN speed for OBD (try different speeds)
    # Bus 1 is where CAN3 messages appear when OBD mode is enabled
    print("\nSetting CAN speed...")
    speeds_to_try = [500, 250, 125, 1000]  # Common OBD speeds
    print(f"  Trying speeds: {speeds_to_try} kbps")
    
    for speed in speeds_to_try:
        try:
            p.set_can_speed_kbps(1, speed)
            p.set_can_enable(1, True)
            print(f"  ✓ Set to {speed} kbps")
            break
        except Exception as e:
            print(f"  ✗ Failed {speed} kbps: {e}")
    
    # Try to wake up OBD port with a request
    print("\nSending OBD wake-up request...")
    try:
        # Mode 01 PID 00 - Request supported PIDs (common wake-up)
        wakeup_request = bytes([0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        p.can_send(0x7DF, wakeup_request, 1)  # 0x7DF is OBD broadcast
        print("  ✓ Sent wake-up request to 0x7DF")
        time.sleep(0.5)  # Wait for response
    except Exception as e:
        print(f"  ✗ Failed to send wake-up: {e}")
    
    print("\nBus configuration:")
    print("  Bus 0: SCAM/Camera (VF8)")
    print("  Bus 1: OBD (CAN3 multiplexed) - Connected to OBD port")
    print("  Bus 2: Chassis (VF8)")
    
    # Clear CAN buffers
    p.can_clear(0xFFFF)
    
    print("\n" + "="*60)
    print("Monitoring CAN messages from OBD port (bus 1)...")
    print("elm327 mode will validate ISO 15765-4 messages")
    print("Press Ctrl+C to stop")
    print("="*60 + "\n")
    
    start_time = time.monotonic()
    last_print = start_time
    all_msgs = defaultdict(list)
    
    # Example OBD-II request addresses
    OBD_BROADCAST = 0x7DF  # OBD-II broadcast request
    OBD_RESPONSE_MIN = 0x7E8  # Minimum OBD response address
    OBD_RESPONSE_MAX = 0x7EF  # Maximum OBD response address
    
    try:
        while True:
            # Receive CAN messages
            can_recv = p.can_recv()
            
            for addr, dat, bus in can_recv:
                # In OBD mode, CAN3 messages appear on bus 1
                if bus == 1:  # OBD bus
                    all_msgs[(addr, bus)].append(dat)
                    
                    # Highlight OBD-II protocol messages
                    if addr == OBD_BROADCAST:
                        print(f"  [OBD Request] 0x{addr:03X}: {binascii.hexlify(dat).decode()}")
                    elif OBD_RESPONSE_MIN <= addr <= OBD_RESPONSE_MAX:
                        print(f"  [OBD Response] 0x{addr:03X}: {binascii.hexlify(dat).decode()}")
            
            # Print statistics every 1 second
            current_time = time.monotonic()
            if current_time - last_print > 1.0:
                elapsed = current_time - start_time
                print(f"\r[{elapsed:6.1f}s] Bus 1 (OBD): {len(all_msgs)} unique messages", end="", flush=True)
                last_print = current_time
                
                # Show some sample messages
                if all_msgs and (current_time - start_time) % 5 < 1:  # Every 5 seconds
                    print("\n  Sample messages:")
                    for (addr, bus), msgs in sorted(list(all_msgs.items())[:5]):
                        latest = msgs[-1] if msgs else b''
                        msg_type = ""
                        if addr == OBD_BROADCAST:
                            msg_type = " [OBD Request]"
                        elif OBD_RESPONSE_MIN <= addr <= OBD_RESPONSE_MAX:
                            msg_type = " [OBD Response]"
                        print(f"    0x{addr:03X} ({addr:4d}) on bus {bus}{msg_type}: {binascii.hexlify(latest).decode()}")
                    print()
            
            time.sleep(0.01)  # Small delay to prevent CPU spinning
            
    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print("Stopping...")
        print(f"Total unique messages received: {len(all_msgs)}")
        
        # Count OBD-II protocol messages
        obd_requests = sum(1 for (addr, _) in all_msgs.keys() if addr == OBD_BROADCAST)
        obd_responses = sum(1 for (addr, _) in all_msgs.keys() if OBD_RESPONSE_MIN <= addr <= OBD_RESPONSE_MAX)
        print(f"  OBD-II requests: {obd_requests}")
        print(f"  OBD-II responses: {obd_responses}")
        
        # Reset safety mode (this will disable OBD mode automatically)
        print("\nResetting safety mode...")
        p.set_safety_mode(CarParams.SafetyModel.noOutput)
        print("✓ Done!")
        print("="*60)


def send_obd_query_example():
    """Example: Send an OBD-II query using elm327 mode"""
    print("\n" + "="*60)
    print("Example: Sending OBD-II Query")
    print("="*60)
    
    p = Panda()
    p.reset()
    time.sleep(1)
    
    # Set elm327 mode with OBD enabled
    p.set_safety_mode(CarParams.SafetyModel.elm327, 0)
    p.set_can_speed_kbps(1, 500)
    p.set_can_enable(1, True)
    p.can_clear(0xFFFF)
    
    # OBD-II Mode 01 PID 0D = Vehicle Speed
    # Format: [Mode, PID, padding...]
    speed_request = bytes([0x02, 0x01, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    print("\nSending OBD-II query: Mode 01 PID 0D (Vehicle Speed)")
    print(f"  Address: 0x7DF (OBD broadcast)")
    print(f"  Data: {binascii.hexlify(speed_request).decode()}")
    
    # Send request
    p.can_send(0x7DF, speed_request, 1)  # Bus 1 = OBD
    
    # Wait for response
    print("\nWaiting for response...")
    for _ in range(50):  # Wait up to 0.5 seconds
        msgs = p.can_recv()
        for addr, dat, bus in msgs:
            if bus == 1 and 0x7E8 <= addr <= 0x7EF:
                print(f"  Response from 0x{addr:03X}: {binascii.hexlify(dat).decode()}")
                # Parse response (simplified - real parsing needs ISO-TP handling)
                if len(dat) >= 3:
                    mode = dat[0] & 0x3F
                    pid = dat[1]
                    value = dat[2]
                    print(f"    Mode: {mode:02X}, PID: {pid:02X}, Value: {value} km/h")
                return
        time.sleep(0.01)
    
    print("  No response received")
    
    # Cleanup
    p.set_safety_mode(CarParams.SafetyModel.noOutput)


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "query":
        send_obd_query_example()
    else:
        vf8_obd_elm327_test()

