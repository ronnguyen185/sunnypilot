#!/usr/bin/env python3
"""
VF8 OBD Mode Test Script
Connects CAN3 to OBD port and monitors messages

Usage:
    python panda/examples/vf8_obd_test.py
"""

import time
from collections import defaultdict
import binascii

from opendbc.car.structs import CarParams
from panda import Panda


def vf8_obd_test(check_all_buses=False):
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
    p.set_safety_mode(CarParams.SafetyModel.elm327, 0)
    print("elm327 safety mode enabled with OBD mode!")
    print("Checking bus 0 for OBD messages")
    print("elm327 mode validates ISO 15765-4 (OBD-II) protocol messages")
    
    # Set CAN speed for OBD (typically 500 kbps for OBD-II)
    # Check bus 0 for OBD messages
    print("\nSetting CAN speed to 500 kbps (OBD-II standard)...")
    if check_all_buses:
        # Enable all buses to see where messages appear
        for bus in [0, 1, 2]:
            p.set_can_speed_kbps(bus, 500)
            p.set_can_enable(bus, True)
        print("  Enabled all buses (0, 1, 2) for testing")
    else:
        p.set_can_speed_kbps(0, 500)  # Bus 0 for OBD
        p.set_can_enable(0, True)      # Enable bus 0
        print("  Enabled bus 0 only")
    
    # Also enable other buses if needed for VF8
    # VF8 uses: Bus 0 = SCAM, Bus 1 = Radar (CAN-FD), Bus 2 = Chassis
    # Checking bus 0 for OBD messages
    print("\nBus configuration:")
    print("  Bus 0: OBD (CAN3 multiplexed) - Connected to OBD port [MONITORING]")
    print("  Bus 1: Radar (CAN-FD) (VF8)" + (" [MONITORING]" if check_all_buses else ""))
    print("  Bus 2: Chassis (VF8)" + (" [MONITORING]" if check_all_buses else ""))
    
    # Clear CAN buffers
    p.can_clear(0xFFFF)
    
    print("\nMonitoring CAN messages from OBD port (bus 0)...")
    print("Press Ctrl+C to stop\n")
    
    start_time = time.monotonic()
    last_print = start_time
    all_msgs = defaultdict(list)
    
    try:
        while True:
            # Receive CAN messages
            can_recv = p.can_recv()
            
            for addr, dat, bus in can_recv:
                # Check bus 0 for OBD messages (or all buses if check_all_buses)
                if check_all_buses or bus == 0:
                    all_msgs[(addr, bus)].append(dat)
                    
                    # Check for UDS/OBD-II messages
                    if bus == 0:
                        # OBD-II addresses
                        is_obd = (addr == 0x7DF or (0x7E8 <= addr <= 0x7EF))
                        # ISO-TP frame detection (first byte indicates frame type)
                        if len(dat) >= 1:
                            frame_type = (dat[0] >> 4) & 0xF
                            if is_obd or frame_type in [0, 1, 2, 3]:  # ISO-TP frame types
                                service_id = None
                                is_response = False
                                
                                # Check for UDS service ID (first byte of payload)
                                if frame_type == 0:  # Single frame
                                    payload_start = 2 if dat[0] & 0x0F == 0 else 1
                                    if len(dat) > payload_start:
                                        service_id = dat[payload_start]
                                        is_response = (service_id >= 0x40 and service_id < 0x80)
                                        
                                elif frame_type == 1:  # First frame
                                    if len(dat) >= 3:
                                        service_id = dat[2]
                                        is_response = (service_id >= 0x40 and service_id < 0x80)
                                
                                if service_id is not None:
                                    # UDS service names (standard ISO 14229)
                                    uds_services = {
                                        0x10: "DiagnosticSessionControl", 0x11: "ECUReset", 0x14: "ClearDiagnosticInformation",
                                        0x19: "ReadDTCInformation", 0x22: "ReadDataByIdentifier", 0x23: "ReadMemoryByAddress",
                                        0x24: "ReadScalingDataByIdentifier", 0x27: "SecurityAccess", 0x28: "CommunicationControl",
                                        0x29: "Authentication", 0x2A: "ReadDataByPeriodicIdentifier", 0x2C: "DynamicallyDefineDataIdentifier",
                                        0x2E: "WriteDataByIdentifier", 0x2F: "InputOutputControlByIdentifier", 0x31: "RoutineControl",
                                        0x34: "RequestDownload", 0x35: "RequestUpload", 0x36: "TransferData",
                                        0x37: "RequestTransferExit", 0x3D: "WriteMemoryByAddress", 0x3E: "TesterPresent",
                                        0x83: "AccessTimingParameter", 0x84: "SecuredDataTransmission", 0x85: "ControlDTCSetting",
                                        0x86: "ResponseOnEvent", 0x87: "LinkControl",
                                    }
                                    
                                    if is_response:
                                        original_service = service_id - 0x40
                                        service_name = uds_services.get(original_service, f"ManufacturerSpecific(0x{original_service:02X})")
                                        print(f"\n[UDS] 0x{addr:03X}: {service_name} (Response 0x{service_id:02X})")
                                    else:
                                        service_name = uds_services.get(service_id, f"ManufacturerSpecific(0x{service_id:02X})")
                                        frame_info = " [Multi-frame]" if frame_type == 1 else ""
                                        print(f"\n[UDS] 0x{addr:03X}: {service_name} (Request 0x{service_id:02X}){frame_info}")
            
            # Print statistics every 0.5 seconds
            current_time = time.monotonic()
            if current_time - last_print > 0.5:
                elapsed = current_time - start_time
                if check_all_buses:
                    bus_counts = {0: 0, 1: 0, 2: 0}
                    for (addr, bus), msgs in all_msgs.items():
                        if bus in bus_counts:
                            bus_counts[bus] += 1
                    print(f"\r[{elapsed:6.1f}s] Bus 0: {bus_counts[0]}, Bus 1: {bus_counts[1]}, Bus 2: {bus_counts[2]} unique messages", end="", flush=True)
                else:
                    print(f"\r[{elapsed:6.1f}s] Bus 0 (OBD): {len(all_msgs)} unique messages", end="", flush=True)
                last_print = current_time
                
                # Show some sample messages
                if all_msgs:
                    print("\n  Sample messages:")
                    for (addr, bus), msgs in sorted(list(all_msgs.items())[:5]):
                        latest = msgs[-1] if msgs else b''
                        print(f"    0x{addr:03X} ({addr:4d}) on bus {bus}: {binascii.hexlify(latest).decode()}")
                    print()
            
            time.sleep(0.01)  # Small delay to prevent CPU spinning
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
        print(f"Total unique messages received: {len(all_msgs)}")
        
        # Reset safety mode (this will disable OBD mode automatically)
        print("\nResetting safety mode...")
        p.set_safety_mode(CarParams.SafetyModel.noOutput)
        print("Done!")


if __name__ == "__main__":
    import sys
    # Use --all-buses to check all buses
    check_all = "--all-buses" in sys.argv or "-a" in sys.argv
    vf8_obd_test(check_all_buses=check_all)

