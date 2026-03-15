#!/usr/bin/env python3
"""
Test script to spam ADAS_BCM_Status message on bus 1 of the second USB panda at 20 Hz.
Message ID: 527 (0x20F) from vinfast_vf8_body_can.dbc
"""

import time
import signal
import sys
from panda import Panda
from opendbc.car.structs import CarParams
from opendbc.can import CANPacker

stop_event = False

def signal_handler(sig, frame):
    global stop_event
    print("\n[MAIN] Received interrupt signal, stopping...")
    stop_event = True

def main():
    global stop_event
    
    # Setup signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # List all available pandas
    print("[MAIN] Scanning for pandas...")
    all_pandas_list = Panda.list()
    usb_pandas_list = Panda.usb_list()
    
    if len(all_pandas_list) == 0:
        print("[ERROR] No pandas found!")
        return 1
    
    print(f"[MAIN] Found {len(all_pandas_list)} total panda(s):")
    for idx, serial in enumerate(all_pandas_list):
        is_usb = serial in usb_pandas_list
        conn_type = "USB" if is_usb else "SPI"
        print(f"  [{idx}]: {serial} ({conn_type})")
    
    # Check if we have USB pandas
    if len(usb_pandas_list) == 0:
        print(f"[ERROR] No USB pandas found!")
        return 1
    
    print(f"\n[MAIN] Found {len(usb_pandas_list)} USB panda(s):")
    for idx, serial in enumerate(usb_pandas_list):
        print(f"  USB[{idx}]: {serial}")
    
    # Use the second USB panda if available, otherwise use the first
    if len(usb_pandas_list) >= 2:
        target_panda_serial = usb_pandas_list[1]
        print(f"\n[MAIN] Using second USB panda (index 1): {target_panda_serial}")
    else:
        target_panda_serial = usb_pandas_list[0]
        print(f"\n[MAIN] Only one USB panda found, using it: {target_panda_serial}")
    
    print(f"\n[MAIN] Connecting to USB panda: {target_panda_serial}")
    
    try:
        panda = Panda(serial=target_panda_serial)
        actual_serial = panda.get_serial()
        print(f"[MAIN] Connected to Panda: {actual_serial}")
        
        # Verify it's actually a USB connection
        if panda.is_connected_usb():
            print(f"[MAIN] ✓ Confirmed: Connected via USB")
        else:
            print(f"[WARN] ⚠ Panda is not connected via USB (connected via SPI)")
        
        # Get panda info
        try:
            health = panda.health()
            print(f"[MAIN] Panda type: {health.get('hw_type', 'Unknown')}")
            print(f"[MAIN] Panda uptime: {health.get('uptime', 'Unknown')} ms")
        except Exception as health_err:
            print(f"[WARN] Could not get panda health info: {health_err}")
        
        # Set safety mode to allOutput to allow reading and sending messages
        print("[MAIN] Setting safety mode to allOutput...")
        panda.set_safety_mode(CarParams.SafetyModel.allOutput)
        
        # First, read from bus 1 to detect if 0x20F message exists
        target_message_id = 527  # 0x20F
        bus = 1
        
        print(f"\n[MAIN] Reading from bus {bus} to detect message 0x{target_message_id:03X}...")
        print(f"[MAIN] Listening for 5 seconds...")
        
        found_0x20f = False
        message_count_0x20f = 0
        last_0x20f_data = None
        start_read_time = time.time()
        all_messages = {}
        
        try:
            while time.time() - start_read_time < 5.0:
                can_recv = panda.can_recv()
                for address, dat, src in can_recv:
                    if src == bus:
                        # Track all messages on this bus
                        if address not in all_messages:
                            all_messages[address] = {'count': 0, 'last_data': None}
                        all_messages[address]['count'] += 1
                        all_messages[address]['last_data'] = dat.hex()
                        
                        # Check for our target message
                        if address == target_message_id:
                            found_0x20f = True
                            message_count_0x20f += 1
                            last_0x20f_data = dat.hex()
                            print(f"[READ] Found 0x{target_message_id:03X} on bus {bus}: {dat.hex()} (count: {message_count_0x20f})")
                time.sleep(0.01)  # Small delay
        except Exception as e:
            print(f"[WARN] Error during read: {e}")
        
        print(f"\n[MAIN] Read summary:")
        print(f"[MAIN] Total unique messages on bus {bus}: {len(all_messages)}")
        if found_0x20f:
            print(f"[MAIN] ✓ Found message 0x{target_message_id:03X} ({message_count_0x20f} times)")
            print(f"[MAIN] Last data: {last_0x20f_data}")
        else:
            print(f"[MAIN] ✗ Message 0x{target_message_id:03X} NOT found on bus {bus}")
            print(f"[MAIN] Available messages on bus {bus}:")
            for addr, info in sorted(all_messages.items()):
                print(f"  - 0x{addr:03X}: {info['count']} messages, last data: {info['last_data']}")
        
        print(f"\n[MAIN] Proceeding to send messages...")
        
        # Initialize DBC packer for vinfast_vf8_body_can
        print("[MAIN] Initializing DBC packer...")
        packer = CANPacker("vinfast_vf8_body_can")
        
        # Message details from vinfast_vf8_body_can.dbc
        # BO_ 527 ADAS_BCM_Status: 8 ADAS_Body
        # SG_ ADAS_BCM_Status : 2|3@0+ (1,0) [0|7] ""  BCM
        # SG_ ADAS_BCM_IndicatorlightReq : 7|3@0+ (1,0) [0|7] ""  BCM
        
        # ADAS_BCM_Status values: 0 "Off", 1 "Standby", 2 "Enable", 3 "Active", 
        #                         4 "Finished", 5 "Suspend", 6 "Abort", 7 "Failed"
        adas_status_value = 3  # "Active"
        
        # ADAS_BCM_IndicatorlightReq values: 0 "No request", 1 "Left", 2 "Right", 
        #                                    3 "Hazard", 4 "Turn Indicator Off"
        indicator_req_value = 3  # "Right"
        
        # Headlight request: 0 "No request", 1 "Request turn on"
        headlight_request = 1
        
        # Fold mirror request: 0 "No request", 1 "Fold in", 2 "Fold out", 3 "Reserved"
        foldmirror_request = 0
        
        # SECCAN value (typically 0 or 1)
        seccan_val = 1
        
        # Create message using DBC packer (properly handles bit packing)
        frame = 0  # Frame counter (not used in this message but kept for compatibility)
        addr, message_data, _ = packer.make_can_msg("ADAS_BCM_Status", bus, {
            "ADAS_BCM_Status": adas_status_value,
            "ADAS_BCM_HeadlightReq": headlight_request,
            "ADAS_BCM_IndicatorlightReq": indicator_req_value,
            "ADAS_BCM_FoldMirrorReq": foldmirror_request,
            "SECCAN_ADAS_BCM_Sts": seccan_val,
        })
        
        if addr == 0 or len(message_data) == 0:
            print(f"[ERROR] Failed to encode message using DBC packer")
            return 1
        
        print(f"\n[MAIN] Starting to spam ADAS_BCM_Status message...")
        print(f"[MAIN] Message ID: {addr} (0x{addr:03X})")
        print(f"[MAIN] Bus: {bus}")
        status_names = {0: "Off", 1: "Standby", 2: "Enable", 3: "Active", 
                        4: "Finished", 5: "Suspend", 6: "Abort", 7: "Failed"}
        print(f"[MAIN] ADAS_BCM_Status value: {adas_status_value} ({status_names.get(adas_status_value, 'Unknown')})")
        indicator_names = {0: "No request", 1: "Left", 2: "Right", 3: "Hazard", 4: "Turn Indicator Off"}
        indicator_name = indicator_names.get(indicator_req_value, f"Unknown({indicator_req_value})")
        print(f"[MAIN] ADAS_BCM_IndicatorlightReq value: {indicator_req_value} ({indicator_name})")
        print(f"[MAIN] ADAS_BCM_HeadlightReq value: {headlight_request}")
        print(f"[MAIN] ADAS_BCM_FoldMirrorReq value: {foldmirror_request}")
        print(f"[MAIN] SECCAN_ADAS_BCM_Sts value: {seccan_val}")
        print(f"[MAIN] Message data: {message_data.hex()}")
        print(f"[MAIN] Press Ctrl+C to stop\n")
        
        message_count = 0
        start_time = time.time()
        last_print_time = time.time()
        
        try:
            while not stop_event:
                # Send the message
                panda.can_send(addr, message_data, bus)
                message_count += 1
                
                # Print statistics every second
                if time.time() - last_print_time >= 1.0:
                    elapsed = time.time() - start_time
                    rate = message_count / elapsed if elapsed > 0 else 0
                    print(f"[STATS] Sent: {message_count} messages, Rate: {rate:.1f} msg/s", end='\r')
                    last_print_time = time.time()
                
                # Small delay to maintain 20 Hz rate
                time.sleep(0.1)  # 20 Hz (1/20 = 0.05s)
                
        except KeyboardInterrupt:
            print("\n[MAIN] Keyboard interrupt received")
        except Exception as e:
            print(f"\n[ERROR] Error sending CAN messages: {e}")
        finally:
            # Send "no request" message before closing
            print("\n[MAIN] Sending 'no request' message (indicator_request=0)...")
            try:
                # Create "no request" message
                no_request_addr, no_request_data, _ = packer.make_can_msg("ADAS_BCM_Status", bus, {
                    "ADAS_BCM_Status": adas_status_value,
                    "ADAS_BCM_HeadlightReq": 0,  # No headlight request
                    "ADAS_BCM_IndicatorlightReq": 4,  # No turn indicator off
                    "ADAS_BCM_FoldMirrorReq": 0,  # No fold mirror request
                    "SECCAN_ADAS_BCM_Sts": seccan_val,
                })
                
                if no_request_addr != 0 and len(no_request_data) > 0:
                    # Send 10 frames to ensure it's received
                    for i in range(10):
                        panda.can_send(no_request_addr, no_request_data, bus)
                        time.sleep(0.05)  # 20 Hz
                    print(f"[MAIN] Sent 10 'no request' messages")
                else:
                    print(f"[WARN] Failed to create 'no request' message")
            except Exception as e:
                print(f"[WARN] Error sending 'no request' message: {e}")
            
            panda.close()
            print("\n[MAIN] Disconnected from Panda")
            
            if message_count > 0:
                elapsed = time.time() - start_time
                total_rate = message_count / elapsed if elapsed > 0 else 0
                print(f"\n[SUMMARY] Total messages sent: {message_count}")
                print(f"[SUMMARY] Total rate: {total_rate:.1f} msg/s")
                print(f"[SUMMARY] Total time: {elapsed:.2f}s")
    
    except Exception as e:
        error_str = str(e)
        if "LIBUSB_ERROR_BUSY" in error_str or "USBErrorBusy" in error_str or "BUSY" in error_str.upper():
            print(f"[ERROR] Panda is busy (already in use by another process)")
            print(f"[INFO] The panda may be in use by openpilot or another script")
            print(f"[INFO] Try stopping other processes that might be using the panda")
            print(f"[INFO] Error details: {e}")
        else:
            print(f"[ERROR] Failed to connect to USB panda: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

