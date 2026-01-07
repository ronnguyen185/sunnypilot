#!/usr/bin/env python3
"""
VF8 UDS Monitor Script
Monitors and parses UDS (Unified Diagnostic Services) messages on OBD port

Usage:
    python panda/examples/vf8_uds_monitor.py
"""

import time
from collections import defaultdict
import binascii

from opendbc.car.structs import CarParams
from panda import Panda


# UDS Service IDs
UDS_SERVICES = {
    0x10: "DiagnosticSessionControl",
    0x11: "ECUReset",
    0x14: "ClearDiagnosticInformation",
    0x19: "ReadDTCInformation",
    0x22: "ReadDataByIdentifier",
    0x23: "ReadMemoryByAddress",
    0x24: "ReadScalingDataByIdentifier",
    0x27: "SecurityAccess",
    0x28: "CommunicationControl",
    0x29: "Authentication",
    0x2A: "ReadDataByPeriodicIdentifier",
    0x2C: "DynamicallyDefineDataIdentifier",
    0x2E: "WriteDataByIdentifier",
    0x2F: "InputOutputControlByIdentifier",
    0x31: "RoutineControl",
    0x34: "RequestDownload",
    0x35: "RequestUpload",
    0x36: "TransferData",
    0x37: "RequestTransferExit",
    0x3D: "WriteMemoryByAddress",
    0x3E: "TesterPresent",
    0x83: "AccessTimingParameter",
    0x84: "SecuredDataTransmission",
    0x85: "ControlDTCSetting",
    0x86: "ResponseOnEvent",
    0x87: "LinkControl",
}

# OBD-II Mode 01 PIDs
OBD_PIDS = {
    0x00: "PIDs supported [01-20]",
    0x01: "Monitor status since DTCs cleared",
    0x02: "Freeze DTC",
    0x03: "Fuel system status",
    0x04: "Calculated engine load value",
    0x05: "Engine coolant temperature",
    0x06: "Short term fuel % trim—Bank 1",
    0x07: "Long term fuel % trim—Bank 1",
    0x08: "Short term fuel % trim—Bank 2",
    0x09: "Long term fuel % trim—Bank 2",
    0x0A: "Fuel pressure",
    0x0B: "Intake manifold absolute pressure",
    0x0C: "Engine RPM",
    0x0D: "Vehicle speed",
    0x0E: "Timing advance",
    0x0F: "Intake air temperature",
    0x10: "MAF air flow rate",
    0x11: "Throttle position",
    0x12: "Commanded secondary air status",
    0x13: "Oxygen sensors present",
    0x14: "Bank 1, Sensor 1: Oxygen sensor voltage",
    0x1C: "OBD standards this vehicle conforms to",
    0x1F: "Run time since engine start",
    0x20: "PIDs supported [21-40]",
    0x21: "Distance traveled with MIL on",
    0x2F: "Fuel tank level input",
    0x33: "Absolute Barometric Pressure",
    0x42: "Control module voltage",
    0x43: "Absolute load value",
    0x44: "Fuel/Air commanded equivalence ratio",
    0x45: "Relative throttle position",
    0x46: "Ambient air temperature",
    0x47: "Absolute throttle position B",
    0x48: "Absolute throttle position C",
    0x49: "Accelerator pedal position D",
    0x4A: "Accelerator pedal position E",
    0x4B: "Accelerator pedal position F",
    0x4C: "Commanded throttle actuator",
    0x4D: "Time run with MIL on",
    0x4E: "Time since trouble codes cleared",
    0x4F: "Maximum value for equivalence ratio",
    0x50: "Maximum value for air flow rate",
    0x51: "Fuel type",
    0x52: "Ethanol fuel %",
    0x53: "Absolute Evap system Vapor Pressure",
    0x54: "Evap system vapor pressure",
    0x55: "Short term secondary oxygen sensor trim",
    0x56: "Long term secondary oxygen sensor trim",
    0x57: "Short term secondary oxygen sensor trim",
    0x58: "Long term secondary oxygen sensor trim",
    0x59: "Fuel rail absolute pressure",
    0x5A: "Relative accelerator pedal position",
    0x5B: "Hybrid battery pack remaining life",
    0x5C: "Engine oil temperature",
    0x5D: "Fuel injection timing",
    0x5E: "Engine fuel rate",
    0x5F: "Emission requirements to which vehicle is designed",
    0x60: "PIDs supported [61-80]",
    0x61: "Driver's demand engine - percent torque",
    0x62: "Actual engine - percent torque",
    0x63: "Engine reference torque",
    0x64: "Engine percent torque data",
    0x65: "Auxiliary input / output supported",
    0x66: "Mass air flow sensor",
    0x67: "Engine coolant temperature",
    0x68: "Intake air temperature sensor",
    0x69: "Commanded EGR and EGR Error",
    0x6A: "Commanded Diesel intake air flow control and relative intake air flow position",
    0x6B: "Exhaust gas recirculation temperature",
    0x6C: "Commanded throttle actuator control and relative throttle position",
    0x6D: "Fuel pressure control system",
    0x6E: "Injection pressure control system",
    0x6F: "Turbocharger compressor inlet pressure",
    0x70: "Boost pressure control",
    0x71: "Variable Geometry turbo (VGT) control",
    0x72: "Wastegate control",
    0x73: "Exhaust pressure",
    0x74: "Turbocharger RPM",
    0x75: "Turbocharger temperature",
    0x76: "Turbocharger temperature",
    0x77: "Charge air cooler temperature (CACT)",
    0x78: "Exhaust Gas temperature (EGT) Bank 1",
    0x79: "Exhaust Gas temperature (EGT) Bank 2",
    0x7A: "Diesel particulate filter (DPF)",
    0x7B: "Diesel particulate filter (DPF)",
    0x7C: "Diesel Particulate filter (DPF) temperature",
    0x7D: "NOx NTE (Not-To-Exceed) control area status",
    0x7E: "PM NTE (Not-To-Exceed) control area status",
    0x7F: "Engine run time",
    0x80: "PIDs supported [81-A0]",
    0x81: "Engine run time for Auxiliary Emissions Control Device (AECD)",
    0x82: "Engine run time for Auxiliary Emissions Control Device (AECD)",
    0x83: "NOx sensor",
    0x84: "Manifold surface temperature",
    0x85: "NOx reagent system",
    0x86: "Particulate matter (PM) sensor",
    0x87: "Intake manifold absolute pressure",
}

# OBD-II Response addresses
OBD_BROADCAST = 0x7DF
OBD_RESPONSE_MIN = 0x7E8
OBD_RESPONSE_MAX = 0x7EF

# ISO-TP Frame Types
ISOTP_SINGLE = 0x0
ISOTP_FIRST = 0x1
ISOTP_CONSECUTIVE = 0x2
ISOTP_FLOW_CONTROL = 0x3


def parse_isotp_frame(data):
    """Parse ISO-TP frame type"""
    if len(data) < 1:
        return None, None
    
    frame_type = (data[0] >> 4) & 0xF
    
    if frame_type == ISOTP_SINGLE:
        length = data[0] & 0x0F
        if length == 0 and len(data) > 1:
            # CAN-FD single frame
            length = data[1]
            payload = data[2:2+length] if len(data) >= 2+length else data[2:]
        else:
            payload = data[1:1+length] if len(data) >= 1+length else data[1:]
        return "SINGLE", payload
    
    elif frame_type == ISOTP_FIRST:
        length = ((data[0] & 0x0F) << 8) | data[1]
        payload = data[2:] if len(data) >= 2 else b''
        return "FIRST", payload
    
    elif frame_type == ISOTP_CONSECUTIVE:
        seq_num = data[0] & 0x0F
        payload = data[1:] if len(data) >= 1 else b''
        return f"CONSECUTIVE({seq_num})", payload
    
    elif frame_type == ISOTP_FLOW_CONTROL:
        fc_type = data[0] & 0x0F
        fc_types = {0: "Continue", 1: "Wait", 2: "Overflow"}
        return f"FLOW_CONTROL({fc_types.get(fc_type, 'Unknown')})", data[1:]
    
    return None, None


def parse_uds_message(data):
    """Parse UDS message"""
    if len(data) < 1:
        return None
    
    service_id = data[0]
    
    # Check for negative response
    if service_id == 0x7F and len(data) >= 3:
        requested_service = data[1]
        error_code = data[2]
        error_codes = {
            0x10: "General Reject",
            0x11: "Service Not Supported",
            0x12: "Sub-Function Not Supported",
            0x13: "Incorrect Message Length Or Invalid Format",
            0x21: "Busy Repeat Request",
            0x22: "Conditions Not Correct",
            0x24: "Request Sequence Error",
            0x31: "Request Out Of Range",
            0x33: "Security Access Denied",
            0x35: "Invalid Key",
            0x36: "Exceed Number Of Attempts",
            0x37: "Required Time Delay Not Expired",
            0x78: "Response Pending",
        }
        error_desc = error_codes.get(error_code, f"Unknown({error_code:02X})")
        return f"NEGATIVE_RESPONSE: Service 0x{requested_service:02X}, Error: {error_desc}"
    
    # Check for positive response (service + 0x40)
    if service_id >= 0x40:
        original_service = service_id - 0x40
        service_name = UDS_SERVICES.get(original_service, f"Unknown(0x{original_service:02X})")
        return f"POSITIVE_RESPONSE: {service_name}"
    
    # Check for request
    service_name = UDS_SERVICES.get(service_id, f"Unknown(0x{service_id:02X})")
    
    # Parse OBD-II Mode 01 PIDs
    if service_id == 0x01 and len(data) >= 2:
        pid = data[1]
        pid_desc = OBD_PIDS.get(pid, f"Unknown PID(0x{pid:02X})")
        return f"REQUEST: {service_name}, PID: {pid_desc}"
    
    return f"REQUEST: {service_name}"


def vf8_uds_monitor(filter_obd_only=False, bus=1):
    """Monitor UDS messages on specified bus
    
    Args:
        filter_obd_only: Only show messages on standard OBD-II addresses
        bus: Bus number to monitor (default 1, as per elm327.h: 'bus 1 is multiplexed to the OBD-II port')
    """
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
    
    # Set CAN speed for OBD
    print(f"\nSetting CAN speed to 500 kbps (OBD-II standard) on bus {bus}...")
    p.set_can_speed_kbps(bus, 500)
    p.set_can_enable(bus, True)
    print(f"✓ Monitoring bus {bus} for OBD/UDS messages")
    
    # Clear CAN buffers
    p.can_clear(0xFFFF)
    
    # Known OBD/UDS addresses
    obd_addresses = {0x7DF, 0x7E0, 0x7E1, 0x7E2, 0x7E3, 0x7E4, 0x7E5, 0x7E6, 0x7E7, 0x7E8, 0x7E9, 0x7EA, 0x7EB, 0x7EC, 0x7ED, 0x7EE, 0x7EF}
    # Common diagnostic addresses (manufacturer-specific)
    diagnostic_addresses = {0x7DF, 0x7E0, 0x7E8, 0x18DA, 0x18DB, 0x18DAF1, 0x18DB33F1}
    
    print("\n" + "="*70)
    print(f"UDS Message Monitor - Bus {bus} (OBD Port)")
    print("="*70)
    if filter_obd_only:
        print("Filter: OBD-II addresses only (0x7DF, 0x7E8-0x7EF)")
    else:
        print("Filter: All addresses (use --obd-only to filter)")
    print("\nMonitoring for:")
    print("  - ISO-TP frames (Single, First, Consecutive, Flow Control)")
    print("  - UDS service requests/responses")
    print("  - OBD-II PIDs (Mode 01)")
    print("  - Negative responses")
    print("\nPress Ctrl+C to stop\n")
    
    start_time = time.monotonic()
    uds_messages = []
    isotp_frames = defaultdict(int)
    all_messages = defaultdict(list)
    address_counts = defaultdict(int)
    
    try:
        while True:
            # Receive CAN messages
            can_recv = p.can_recv()
            
            for addr, dat, msg_bus in can_recv:
                if msg_bus == bus:  # OBD bus
                    all_messages[(addr, msg_bus)].append(dat)
                    address_counts[addr] += 1
                    
                    # Check if it's an OBD-II/UDS address
                    is_obd = (addr == OBD_BROADCAST or 
                             (OBD_RESPONSE_MIN <= addr <= OBD_RESPONSE_MAX))
                    
                    # Filter if requested
                    if filter_obd_only and not is_obd:
                        continue
                    
                    # Parse ISO-TP frame
                    frame_type, payload = parse_isotp_frame(dat)
                    
                    if frame_type:
                        isotp_frames[frame_type] += 1
                        
                        # Only show UDS messages on known diagnostic addresses or if it's clearly a UDS message
                        # This reduces false positives from regular CAN messages
                        show_message = is_obd or filter_obd_only == False
                        
                        # Try to parse UDS if we have payload
                        if payload and len(payload) > 0 and show_message:
                            uds_info = parse_uds_message(payload)
                            if uds_info:
                                # Filter out "Unknown" services unless on OBD addresses
                                if "Unknown" in uds_info and not is_obd:
                                    continue
                                
                                timestamp = time.monotonic() - start_time
                                uds_messages.append((timestamp, addr, frame_type, uds_info, payload))
                                
                                # Print UDS message
                                addr_marker = "[OBD]" if is_obd else "[DIAG]"
                                print(f"[{timestamp:6.1f}s] {addr_marker} 0x{addr:03X} | {frame_type:15s} | {uds_info}")
                                if len(payload) > 8:
                                    print(f"           Payload: {binascii.hexlify(payload[:32]).decode()}...")
                                else:
                                    print(f"           Payload: {binascii.hexlify(payload).decode()}")
                    
                    elif is_obd and len(dat) == 8:
                        # Might be OBD-II message (8 bytes, standard format)
                        if dat[0] == 0x02 and len(dat) >= 3:
                            # OBD-II format: [length, mode, PID, ...]
                            mode = dat[1]
                            pid = dat[2] if len(dat) > 2 else None
                            if mode == 0x01 and pid is not None:
                                pid_desc = OBD_PIDS.get(pid, f"Unknown(0x{pid:02X})")
                                timestamp = time.monotonic() - start_time
                                print(f"[{timestamp:6.1f}s] 0x{addr:03X} | OBD-II Request | Mode 0x{mode:02X}, PID: {pid_desc}")
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("SUMMARY")
        print("="*70)
        print(f"Total messages received: {len(all_messages)}")
        print(f"UDS messages parsed: {len(uds_messages)}")
        
        # Show address statistics
        print(f"\nAddress Statistics (top 20):")
        sorted_addrs = sorted(address_counts.items(), key=lambda x: x[1], reverse=True)[:20]
        for addr, count in sorted_addrs:
            is_obd = (addr == OBD_BROADCAST or (OBD_RESPONSE_MIN <= addr <= OBD_RESPONSE_MAX))
            marker = "[OBD]" if is_obd else ""
            print(f"  0x{addr:03X} ({addr:4d}): {count:5d} messages {marker}")
        
        print(f"\nISO-TP Frame Types:")
        for frame_type, count in sorted(isotp_frames.items()):
            print(f"  {frame_type}: {count}")
        
        if uds_messages:
            print(f"\nUDS Messages (last 10):")
            for timestamp, addr, frame_type, uds_info, payload in uds_messages[-10:]:
                print(f"  [{timestamp:6.1f}s] 0x{addr:03X} | {frame_type} | {uds_info}")
        elif filter_obd_only:
            print(f"\n⚠ No UDS messages found on OBD-II addresses (0x7DF, 0x7E8-0x7EF)")
            print(f"  Try running without --obd-only to see all diagnostic addresses")
            print(f"  Most active addresses shown above")
        
        # Reset safety mode
        print("\nResetting safety mode...")
        p.set_safety_mode(CarParams.SafetyModel.noOutput)
        print("Done!")


if __name__ == "__main__":
    import sys
    # Use --obd-only to filter only OBD-II addresses
    filter_obd = "--obd-only" in sys.argv or "-o" in sys.argv
    # Use --bus N to specify bus number (default 1)
    bus_num = 1  # Default per elm327.h documentation
    for i, arg in enumerate(sys.argv):
        if arg in ["--bus", "-b"] and i + 1 < len(sys.argv):
            try:
                bus_num = int(sys.argv[i + 1])
            except ValueError:
                print(f"Invalid bus number: {sys.argv[i + 1]}")
                sys.exit(1)
    vf8_uds_monitor(filter_obd_only=filter_obd, bus=bus_num)

