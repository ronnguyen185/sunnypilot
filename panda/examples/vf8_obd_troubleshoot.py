#!/usr/bin/env python3
"""
VF8 OBD Troubleshooting Script
Diagnoses OBD connection issues

Usage:
    python panda/examples/vf8_obd_troubleshoot.py
"""

import time
import binascii
from panda import Panda
from opendbc.car.structs import CarParams


def test_all_buses():
    """Check all buses for any CAN activity"""
    print("\n" + "="*60)
    print("Step 1: Checking all buses for CAN activity")
    print("="*60)
    
    p = Panda()
    p.reset()
    time.sleep(1)
    
    # Set safety mode
    p.set_safety_mode(CarParams.SafetyModel.elm327, 0)
    
    # Enable all buses with different speeds
    speeds = [500, 250, 125, 1000]  # Common OBD speeds
    
    for speed in speeds:
        print(f"\nTesting speed: {speed} kbps")
        for bus in [0, 1, 2]:
            try:
                p.set_can_speed_kbps(bus, speed)
                p.set_can_enable(bus, True)
            except Exception as e:
                print(f"  Bus {bus}: Error setting speed - {e}")
                continue
        
        p.can_clear(0xFFFF)
        time.sleep(0.5)  # Wait for messages
        
        # Check all buses
        msgs = p.can_recv()
        bus_counts = {0: 0, 1: 0, 2: 0}
        
        for addr, dat, bus in msgs:
            if bus in bus_counts:
                bus_counts[bus] += 1
        
        print(f"  Messages received:")
        for bus, count in bus_counts.items():
            if count > 0:
                print(f"    Bus {bus}: {count} messages")
        
        if any(bus_counts.values()):
            print(f"  ✓ Found activity at {speed} kbps!")
            return speed, bus_counts
    
    print("\n  ✗ No CAN activity detected on any bus")
    return None, None


def test_obd_wakeup():
    """Try to wake up OBD port by sending requests"""
    print("\n" + "="*60)
    print("Step 2: Attempting to wake up OBD port")
    print("="*60)
    
    p = Panda()
    p.reset()
    time.sleep(1)
    
    # Set elm327 mode with OBD enabled
    p.set_safety_mode(CarParams.SafetyModel.elm327, 0)
    p.set_can_speed_kbps(1, 500)
    p.set_can_enable(1, True)
    p.can_clear(0xFFFF)
    
    # Common OBD-II wake-up requests
    wakeup_requests = [
        (0x7DF, bytes([0x02, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00])),  # Mode 03 - DTCs
        (0x7DF, bytes([0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),  # Mode 01 PID 00 - PIDs supported
        (0x7DF, bytes([0x02, 0x01, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00])),  # Mode 01 PID 0D - Speed
        (0x7E0, bytes([0x02, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00])),  # Direct to ECU
    ]
    
    print("\nSending OBD wake-up requests...")
    for addr, data in wakeup_requests:
        try:
            p.can_send(addr, data, 1)
            print(f"  Sent to 0x{addr:03X}: {binascii.hexlify(data).decode()}")
        except Exception as e:
            print(f"  Failed to send to 0x{addr:03X}: {e}")
        time.sleep(0.1)
    
    # Wait for responses
    print("\nWaiting for responses (2 seconds)...")
    time.sleep(2)
    
    responses = []
    for _ in range(20):  # Check multiple times
        msgs = p.can_recv()
        for addr, dat, bus in msgs:
            if bus == 1:
                responses.append((addr, dat))
        time.sleep(0.1)
    
    if responses:
        print(f"  ✓ Received {len(responses)} responses:")
        for addr, dat in responses[:10]:  # Show first 10
            print(f"    0x{addr:03X}: {binascii.hexlify(dat).decode()}")
        return True
    else:
        print("  ✗ No responses received")
        return False


def test_manual_obd_mode():
    """Test manually enabling OBD mode"""
    print("\n" + "="*60)
    print("Step 3: Testing manual OBD mode")
    print("="*60)
    
    p = Panda()
    p.reset()
    time.sleep(1)
    
    # Try different approaches
    approaches = [
        ("elm327 param=0", lambda: p.set_safety_mode(CarParams.SafetyModel.elm327, 0)),
        ("elm327 param=1 + set_obd", lambda: (p.set_safety_mode(CarParams.SafetyModel.elm327, 1), p.set_obd(True))),
        ("allOutput + set_obd", lambda: (p.set_safety_mode(CarParams.SafetyModel.allOutput), p.set_obd(True))),
    ]
    
    for name, setup_func in approaches:
        print(f"\nTesting: {name}")
        try:
            p.reset()
            time.sleep(0.5)
            setup_func()
            p.set_can_speed_kbps(1, 500)
            p.set_can_enable(1, True)
            p.can_clear(0xFFFF)
            
            time.sleep(1)
            msgs = p.can_recv()
            count = sum(1 for _, _, bus in msgs if bus == 1)
            
            if count > 0:
                print(f"  ✓ Received {count} messages on bus 1")
                return True
            else:
                print(f"  ✗ No messages on bus 1")
        except Exception as e:
            print(f"  ✗ Error: {e}")
    
    return False


def check_panda_health():
    """Check panda health and configuration"""
    print("\n" + "="*60)
    print("Step 4: Checking Panda health")
    print("="*60)
    
    p = Panda()
    
    try:
        health = p.health()
        print(f"Panda Serial: {p.get_serial()}")
        print(f"Panda Version: {p.get_version()}")
        print(f"Health: {health}")
        
        # Check if panda is internal
        is_internal = p.is_internal()
        print(f"Internal: {is_internal}")
        
        return True
    except Exception as e:
        print(f"✗ Error checking panda: {e}")
        return False


def main():
    print("="*60)
    print("VF8 OBD Troubleshooting")
    print("="*60)
    print("\nThis script will help diagnose OBD connection issues")
    print("Make sure CAN3 is connected to the OBD port")
    
    # Check panda health
    if not check_panda_health():
        print("\n✗ Panda health check failed. Check hardware connection.")
        return
    
    # Test all buses
    speed, bus_counts = test_all_buses()
    
    if speed:
        print(f"\n✓ Found CAN activity at {speed} kbps")
        print("  Try using this speed in your script")
    
    # Test OBD wake-up
    obd_working = test_obd_wakeup()
    
    # Test manual OBD mode
    manual_working = test_manual_obd_mode()
    
    # Summary
    print("\n" + "="*60)
    print("TROUBLESHOOTING SUMMARY")
    print("="*60)
    
    if speed:
        print(f"✓ CAN activity detected at {speed} kbps")
    else:
        print("✗ No CAN activity detected")
        print("  - Check physical connections")
        print("  - Verify OBD port wiring (CAN-H, CAN-L, GND, 12V)")
        print("  - Try different CAN speeds")
    
    if obd_working:
        print("✓ OBD port is responding")
    else:
        print("✗ OBD port not responding")
        print("  - OBD port may need ignition ON")
        print("  - Some cars require specific wake-up sequence")
        print("  - Check OBD port power (12V)")
    
    if manual_working:
        print("✓ OBD mode configuration working")
    else:
        print("✗ OBD mode configuration issue")
        print("  - Try different safety modes")
        print("  - Check bus enable status")
    
    print("\nRecommendations:")
    if not speed and not obd_working:
        print("1. Verify physical connection: CAN3 to OBD port")
        print("2. Check OBD port power (12V present)")
        print("3. Ensure ignition is ON (some cars require this)")
        print("4. Try different CAN speeds: 500, 250, 125, 1000 kbps")
    elif speed and not obd_working:
        print("1. OBD port may need specific wake-up sequence")
        print("2. Try sending OBD requests to wake up the port")
        print("3. Some cars only respond to specific OBD addresses")
    else:
        print("1. Use the working configuration in your script")
        print("2. Set CAN speed to the detected speed")


if __name__ == "__main__":
    main()

