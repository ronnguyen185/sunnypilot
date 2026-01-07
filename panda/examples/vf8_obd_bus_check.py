#!/usr/bin/env python3
"""
VF8 OBD Bus Detection Script
Checks all buses to determine which one has OBD/UDS messages when elm327 mode is enabled

Usage:
    python panda/examples/vf8_obd_bus_check.py
"""

import time
from collections import defaultdict
import binascii

from opendbc.car.structs import CarParams
from panda import Panda


def check_obd_bus():
    """Check all buses to find which one has OBD messages"""
    print("Connecting to Panda...")
    p = Panda()
    print(f"Connected to Panda: {p.get_serial()[0]} - {p.get_version()}")
    
    # Reset panda
    p.reset()
    time.sleep(1)
    
    # Use elm327 safety mode with param=0 to enable OBD mode
    print("\nSetting elm327 safety mode with OBD enabled (param=0)...")
    print("  According to elm327.h: 'If safety_param == 0, bus 1 is multiplexed to the OBD-II port'")
    p.set_safety_mode(CarParams.SafetyModel.elm327, 0)
    print("✓ elm327 safety mode enabled with OBD mode!")
    
    # Enable all buses with 500 kbps (OBD-II standard)
    print("\nEnabling all buses (0, 1, 2) at 500 kbps...")
    for bus in [0, 1, 2]:
        p.set_can_speed_kbps(bus, 500)
        p.set_can_enable(bus, True)
        print(f"  ✓ Bus {bus} enabled")
    
    # Clear CAN buffers
    p.can_clear(0xFFFF)
    
    print("\n" + "="*70)
    print("OBD Bus Detection - Monitoring all buses for 10 seconds")
    print("="*70)
    print("Looking for:")
    print("  - OBD-II addresses (0x7DF, 0x7E8-0x7EF)")
    print("  - ISO-TP frames (UDS protocol)")
    print("  - Standard UDS services")
    print("\n")
    
    start_time = time.monotonic()
    bus_stats = {
        0: {"total": 0, "obd_addrs": 0, "isotp": 0, "uds": 0, "addresses": defaultdict(int)},
        1: {"total": 0, "obd_addrs": 0, "isotp": 0, "uds": 0, "addresses": defaultdict(int)},
        2: {"total": 0, "obd_addrs": 0, "isotp": 0, "uds": 0, "addresses": defaultdict(int)},
    }
    
    # OBD-II addresses
    OBD_BROADCAST = 0x7DF
    OBD_RESPONSE_MIN = 0x7E8
    OBD_RESPONSE_MAX = 0x7EF
    
    try:
        while time.monotonic() - start_time < 10:
            can_recv = p.can_recv()
            
            for addr, dat, bus in can_recv:
                if bus in [0, 1, 2]:
                    stats = bus_stats[bus]
                    stats["total"] += 1
                    stats["addresses"][addr] += 1
                    
                    # Check for OBD-II addresses
                    is_obd = (addr == OBD_BROADCAST or 
                             (OBD_RESPONSE_MIN <= addr <= OBD_RESPONSE_MAX))
                    if is_obd:
                        stats["obd_addrs"] += 1
                    
                    # Check for ISO-TP frame
                    if len(dat) >= 1:
                        frame_type = (dat[0] >> 4) & 0xF
                        if frame_type in [0, 1, 2, 3]:  # ISO-TP frame types
                            stats["isotp"] += 1
                            
                            # Try to detect UDS service ID
                            if frame_type == 0:  # Single frame
                                payload_start = 2 if dat[0] & 0x0F == 0 else 1
                                if len(dat) > payload_start:
                                    service_id = dat[payload_start]
                                    # Standard UDS services
                                    if service_id in [0x10, 0x11, 0x19, 0x22, 0x27, 0x3E] or \
                                       (service_id >= 0x40 and service_id < 0x80):
                                        stats["uds"] += 1
                            elif frame_type == 1:  # First frame
                                if len(dat) >= 3:
                                    service_id = dat[2]
                                    if service_id in [0x10, 0x11, 0x19, 0x22, 0x27, 0x3E] or \
                                       (service_id >= 0x40 and service_id < 0x80):
                                        stats["uds"] += 1
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        pass
    
    # Print results
    print("\n" + "="*70)
    print("RESULTS")
    print("="*70)
    
    for bus in [0, 1, 2]:
        stats = bus_stats[bus]
        print(f"\nBus {bus}:")
        print(f"  Total messages: {stats['total']}")
        print(f"  OBD-II addresses (0x7DF, 0x7E8-0x7EF): {stats['obd_addrs']}")
        print(f"  ISO-TP frames: {stats['isotp']}")
        print(f"  UDS services detected: {stats['uds']}")
        
        if stats['addresses']:
            print(f"  Top 10 addresses:")
            sorted_addrs = sorted(stats['addresses'].items(), key=lambda x: x[1], reverse=True)[:10]
            for addr, count in sorted_addrs:
                is_obd = (addr == OBD_BROADCAST or (OBD_RESPONSE_MIN <= addr <= OBD_RESPONSE_MAX))
                marker = "[OBD]" if is_obd else ""
                print(f"    0x{addr:03X} ({addr:4d}): {count:5d} {marker}")
    
    # Determine which bus is most likely OBD
    print("\n" + "="*70)
    print("RECOMMENDATION")
    print("="*70)
    
    # Score each bus
    bus_scores = {}
    for bus in [0, 1, 2]:
        stats = bus_stats[bus]
        score = 0
        if stats['obd_addrs'] > 0:
            score += 100  # OBD addresses are strong indicator
        if stats['uds'] > 0:
            score += 50   # UDS services are strong indicator
        if stats['isotp'] > 0:
            score += 10   # ISO-TP frames are indicator
        score += stats['total'] // 10  # More messages = more activity
        bus_scores[bus] = score
    
    sorted_buses = sorted(bus_scores.items(), key=lambda x: x[1], reverse=True)
    
    print(f"\nBus scores (higher = more likely to be OBD):")
    for bus, score in sorted_buses:
        stats = bus_stats[bus]
        print(f"  Bus {bus}: {score} points (OBD addrs: {stats['obd_addrs']}, UDS: {stats['uds']}, ISO-TP: {stats['isotp']})")
    
    best_bus = sorted_buses[0][0]
    best_score = sorted_buses[0][1]
    
    if best_score > 0:
        print(f"\n✓ Recommended bus: Bus {best_bus}")
        print(f"  This bus shows the most OBD/UDS activity")
        if best_bus == 1:
            print(f"  ✓ Matches elm327.h documentation: 'bus 1 is multiplexed to the OBD-II port'")
        else:
            print(f"  ⚠ Note: elm327.h says bus 1 should be OBD, but bus {best_bus} shows more activity")
    else:
        print(f"\n⚠ No clear OBD activity detected on any bus")
        print(f"  Check connections and ensure OBD port is connected to CAN3")
    
    # Reset safety mode
    print("\nResetting safety mode...")
    p.set_safety_mode(CarParams.SafetyModel.noOutput)
    print("Done!")


if __name__ == "__main__":
    check_obd_bus()

