#!/usr/bin/env python3
"""
Script to test a specific panda device by ID and read CAN messages from all buses.
The panda ID is extracted from hex string: 0b0034000851313339353335 -> "Q139535"
Safety mode is set to allOutput to allow reading all messages.
"""

import argparse
import signal
import sys
import time
from panda import Panda
from opendbc.can.parser import CANParser

stop_event = False

def signal_handler(sig, frame):
    global stop_event
    print("\n[MAIN] Received interrupt signal, stopping...")
    stop_event = True

def decode_panda_id_from_hex(hex_str):
    """
    Decode panda ID from hex string format: 0b0034000851313339353335
    Returns the ASCII serial number (e.g., "Q139535")
    """
    try:
        bytes_data = bytes.fromhex(hex_str)
        if len(bytes_data) >= 5:
            # Parse: [length_byte][CAN_ID_2bytes][data_length_2bytes][data]
            data_length = int.from_bytes(bytes_data[3:5], byteorder='big')
            data = bytes_data[5:5+data_length] if len(bytes_data) >= 5+data_length else bytes_data[5:]
            # Try to decode as ASCII
            ascii_str = data.decode('ascii', errors='ignore').strip('\x00')
            return ascii_str
    except Exception as e:
        print(f"[WARN] Error decoding hex string: {e}")
    return None

def main():
    global stop_event

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description="Test a specific panda device and read CAN messages from all buses",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --panda 1          # Test first panda (Q139535)
  %(prog)s --panda 2          # Test second panda (Q308621)
  %(prog)s --serial Q139535   # Test by serial number
  %(prog)s --hex 0b0034000851313339353335  # Test by hex ID
        """
    )
    parser.add_argument(
        '--panda', type=int, choices=[1, 2],
        help='Select panda by number (1=Q139535, 2=Q308621)'
    )
    parser.add_argument(
        '--serial', type=str,
        help='Select panda by serial number (e.g., Q139535 or Q308621)'
    )
    parser.add_argument(
        '--hex', type=str,
        help='Select panda by hex ID (e.g., 0b0034000851313339353335)'
    )

    args = parser.parse_args()

    # Determine target panda based on arguments
    target_hex = None
    target_serial = None

    if args.panda:
        # Predefined pandas
        if args.panda == 1:
            target_hex = "0b0034000851313339353335"
            target_serial = "Q139535"
            print(f"[MAIN] Selected panda 1 (Q139535)")
        elif args.panda == 2:
            target_hex = "170015000d51333038363231"
            target_serial = "Q308621"
            print(f"[MAIN] Selected panda 2 (Q308621)")
    elif args.serial:
        target_serial = args.serial
        print(f"[MAIN] Selected panda by serial: '{target_serial}'")
    elif args.hex:
        target_hex = args.hex
        decoded = decode_panda_id_from_hex(target_hex)
        if decoded:
            target_serial = decoded
            print(f"[MAIN] Selected panda by hex: '{target_hex}' -> '{decoded}'")
        else:
            print(f"[MAIN] Selected panda by hex: '{target_hex}' (could not decode)")
    else:
        # Default to panda 1 if no argument provided
        target_hex = "0b0034000851313339353335"
        target_serial = "Q139535"
        print(f"[MAIN] No argument provided, defaulting to panda 1 (Q139535)")
        print(f"[INFO] Use --panda 1 or --panda 2 to select a specific panda")

    # Decode hex if we have hex but not serial
    if target_hex and not target_serial:
        target_serial = decode_panda_id_from_hex(target_hex)
        if target_serial:
            print(f"[MAIN] Decoded panda ID from hex: '{target_serial}'")
        else:
            print(f"[WARN] Could not decode hex string, using raw search")

    # Setup signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # List all available pandas
    print("[MAIN] Scanning for pandas...")
    pandas_list = Panda.list()

    if len(pandas_list) == 0:
        print("[ERROR] No pandas found!")
        return 1

    print(f"[MAIN] Found {len(pandas_list)} panda(s):")
    for idx, serial in enumerate(pandas_list):
        decoded = decode_panda_id_from_hex(serial) if len(serial) > 10 else None
        if decoded:
            print(f"  [{idx}]: {serial} -> '{decoded}'")
        else:
            print(f"  [{idx}]: {serial}")

    # First, try exact hex match (if we have a hex ID)
    target_panda_serial = None
    if target_hex and target_hex in pandas_list:
        target_panda_serial = target_hex
        print(f"\n[MAIN] Found target panda by exact hex match: {target_hex}")
    else:
        # Try to find panda by decoded serial number
        for serial in pandas_list:
            # Check if the hex string itself matches (if we have target_hex)
            if target_hex and serial == target_hex:
                target_panda_serial = serial
                print(f"\n[MAIN] Found target panda: {serial} (exact hex match)")
                break
            # Check if decoded serial matches
            decoded_serial = decode_panda_id_from_hex(serial)
            if decoded_serial and target_serial and (target_serial in decoded_serial or decoded_serial == target_serial):
                target_panda_serial = serial
                print(f"\n[MAIN] Found target panda: {serial} -> '{decoded_serial}' (matches {target_serial})")
                break

    # If not found by serial, try to match by connecting and checking actual serial
    if target_panda_serial is None:
        print(f"\n[MAIN] Serial '{target_serial}' not found in list, trying to match by connecting...")
        for serial in pandas_list:
            try:
                test_panda = Panda(serial=serial)
                test_serial = test_panda.get_serial()
                test_panda.close()
                print(f"[DEBUG] Panda {serial} -> actual serial: '{test_serial}'")
                if target_serial and (target_serial in test_serial or test_serial == target_serial):
                    target_panda_serial = serial
                    print(f"[MAIN] Found target panda: {serial} -> {test_serial}")
                    break
            except Exception as e:
                print(f"[DEBUG] Error connecting to {serial}: {e}")
                continue

    if target_panda_serial is None:
        print(f"[ERROR] Could not find panda with serial '{target_serial}' or matching pattern")
        print(f"[INFO] Available pandas: {pandas_list}")
        return 1

    print(f"\n[MAIN] Connecting to target panda: {target_panda_serial}")

    try:
        panda = Panda(serial=target_panda_serial)
        actual_serial = panda.get_serial()
        print(f"[MAIN] Connected to Panda: {actual_serial}")

        # Verify we're connected to the correct panda
        if target_serial:
            # Check if actual serial matches target (handle both list and string formats)
            if isinstance(actual_serial, list):
                actual_serial_str = actual_serial[0] if len(actual_serial) > 0 else str(actual_serial)
            else:
                actual_serial_str = str(actual_serial)

            if target_serial not in actual_serial_str and actual_serial_str not in target_serial:
                print(f"[WARN] Serial mismatch! Expected '{target_serial}', got '{actual_serial_str}'")
                print(f"[WARN] Panda may be in use by another process, causing connection to wrong device")

        # Get panda info
        try:
            health = panda.health()
            print(f"[MAIN] Panda type: {health.get('hw_type', 'Unknown')}")
            print(f"[MAIN] Panda uptime: {health.get('uptime', 'Unknown')} ms")
        except Exception as health_err:
            print(f"[WARN] Could not get panda health info: {health_err}")

    except Exception as e:
        error_str = str(e)
        if "LIBUSB_ERROR_BUSY" in error_str or "USBErrorBusy" in error_str or "BUSY" in error_str.upper():
            print(f"[ERROR] Panda is busy (already in use by another process)")
            print(f"[INFO] The panda may be in use by openpilot or another script")
            print(f"[INFO] Try stopping other processes that might be using the panda")
            print(f"[INFO] Error details: {e}")
        else:
            print(f"[ERROR] Failed to connect to target panda: {e}")
        return 1

    # Reset CAN to ensure clean state
    print("[MAIN] Resetting CAN communications...")
    try:
        panda.can_reset_communications()
        time.sleep(0.1)
    except Exception as e:
        print(f"[WARN] Error resetting CAN: {e}")

    # Test CAN communication before starting main loop
    print("[MAIN] Testing CAN communication...")
    try:
        test_recv = panda.can_recv()
        print(f"[MAIN] CAN test successful (received {len(test_recv)} messages)")
    except Exception as e:
        print(f"[WARN] CAN test failed: {e}")
        print(f"[WARN] Continuing anyway, but CAN communication may not work properly")

    # Initialize DBC parser for bus 2 (turn indicator and blind spot monitor)
    print("[MAIN] Initializing DBC parser for bus 2 (turn indicator & blind spot monitor)...")
    try:
        # Parse messages: BCM_LIGHT (262), BCM_SwitchSts (265), ADAS_BSD (307)
        bus2_parser = CANParser("vinfast_vf8_info_can", [
            ("BCM_LIGHT", 10),      # Turn indicator status
            ("BCM_SwitchSts", 10),  # Turn indicator switch
            ("ADAS_BSD", 10),       # Blind spot detection
        ], bus=2)
        print("[MAIN] DBC parser initialized successfully")
    except Exception as e:
        print(f"[WARN] Failed to initialize DBC parser: {e}")
        print(f"[WARN] Continuing without DBC parsing")
        bus2_parser = None

    # Start reading from all buses
    print("\n[MAIN] Starting to read CAN messages from all buses...")
    if bus2_parser:
        print("[MAIN] Bus 2 messages will be parsed for turn indicator and blind spot monitor")
    print("[MAIN] Press Ctrl+C to stop\n")
    print(f"{'Time':<12} {'Bus':<4} {'ID':<8} {'Data':<32} {'Length':<6} {'Parsed Info'}")
    print("-" * 100)

    message_count = 0
    bus_counts = {}  # Track messages per bus
    start_time = time.time()
    last_print_time = time.time()
    consecutive_errors = 0
    max_consecutive_errors = 10

    try:
        while not stop_event:
            try:
                can_recv = panda.can_recv()
                consecutive_errors = 0  # Reset error counter on success
            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors >= max_consecutive_errors:
                    print(f"\n[ERROR] Too many consecutive CAN receive errors ({consecutive_errors})")
                    print(f"[ERROR] Last error: {e}")
                    print(f"[ERROR] Panda may be in use by another process or not properly connected")
                    break
                time.sleep(0.1)  # Brief pause before retry
                continue

            for address, dat, src in can_recv:
                message_count += 1

                # Track bus counts
                if src not in bus_counts:
                    bus_counts[src] = 0
                bus_counts[src] += 1

                elapsed = time.time() - start_time
                data_hex = dat.hex()

                # Parse bus 2 messages for turn indicator and blind spot monitor
                parsed_info = ""
                if src == 2 and bus2_parser is not None:
                    try:
                        # Update parser with current message
                        # Format: (timestamp_nanos, [(address, dat, src), ...])
                        timestamp_nanos = int(time.time() * 1e9)
                        bus2_parser.update([(timestamp_nanos, [(address, dat, src)])])
                        vl = bus2_parser.vl

                        # Check for turn indicator messages
                        turn_indicator_sts = None
                        turn_indicator_switch = None
                        bsd_info = []

                        # BCM_LIGHT (0x106 = 262)
                        if 262 in vl:
                            turn_indicator_sts = vl[262].get("BCM_TurnIndicatorSts")

                        # BCM_SwitchSts (0x109 = 265)
                        if 265 in vl:
                            turn_indicator_switch = vl[265].get("BCM_TurnIndicator")

                        # ADAS_BSD (0x133 = 307)
                        if 307 in vl:
                            bsd_state = vl[307].get("ADAS_BSD_state")
                            bsd_ind_left = vl[307].get("ADAS_BSD_IndLeft")
                            bsd_ind_right = vl[307].get("ADAS_BSD_IndRight")
                            bsd_snd_warn = vl[307].get("ADAS_BSD_SndWarn")
                            bsd_mode = vl[307].get("ADAS_BSD_Mode_Feed")

                            if bsd_state is not None:
                                bsd_info.append(f"BSD_State={int(bsd_state)}")
                            if bsd_ind_left is not None:
                                bsd_info.append(f"BSD_Left={int(bsd_ind_left)}")
                            if bsd_ind_right is not None:
                                bsd_info.append(f"BSD_Right={int(bsd_ind_right)}")
                            if bsd_snd_warn is not None:
                                bsd_info.append(f"BSD_Sound={int(bsd_snd_warn)}")
                            if bsd_mode is not None:
                                bsd_info.append(f"BSD_Mode={int(bsd_mode)}")

                        # Build parsed info string
                        info_parts = []
                        if turn_indicator_sts is not None:
                            turn_states = {0: "OFF", 1: "LEFT", 2: "RIGHT", 3: "INVALID"}
                            turn_str = turn_states.get(int(turn_indicator_sts), f"UNK({int(turn_indicator_sts)})")
                            info_parts.append(f"TurnSts={turn_str}")
                        if turn_indicator_switch is not None:
                            switch_states = {0: "OFF", 1: "LEFT", 2: "RIGHT", 3: "INVALID"}
                            switch_str = switch_states.get(int(turn_indicator_switch), f"UNK({int(turn_indicator_switch)})")
                            info_parts.append(f"TurnSwitch={switch_str}")
                        if bsd_info:
                            info_parts.extend(bsd_info)

                        if info_parts:
                            parsed_info = " | " + ", ".join(info_parts)
                    except Exception as parse_err:
                        # Silently ignore parsing errors to avoid spam
                        pass

                # Print every message from all buses
                print(f"{elapsed:>10.3f}s {src:<4} 0x{address:03X} {data_hex:<32} {len(dat):<6}{parsed_info}")

                # Print statistics every 5 seconds
                if time.time() - last_print_time >= 5.0:
                    elapsed = time.time() - start_time
                    total_rate = message_count / elapsed if elapsed > 0 else 0
                    bus_stats = ", ".join([f"Bus {bus}: {count} ({count/elapsed:.1f} msg/s)"
                                          for bus, count in sorted(bus_counts.items())])
                    print(f"\n[STATS] Total: {message_count} ({total_rate:.1f} msg/s), {bus_stats}, Elapsed: {elapsed:.1f}s\n")
                    last_print_time = time.time()

    except KeyboardInterrupt:
        print("\n[MAIN] Keyboard interrupt received")
    except Exception as e:
        print(f"\n[ERROR] Error reading CAN messages: {e}")
    finally:
        panda.close()
        print("[MAIN] Disconnected from Panda")

        if message_count > 0:
            elapsed = time.time() - start_time
            total_rate = message_count / elapsed if elapsed > 0 else 0
            print(f"\n[SUMMARY] Total messages received: {message_count}")
            print(f"[SUMMARY] Total rate: {total_rate:.1f} msg/s")
            print(f"[SUMMARY] Messages per bus:")
            for bus in sorted(bus_counts.keys()):
                count = bus_counts[bus]
                rate = count / elapsed if elapsed > 0 else 0
                print(f"  Bus {bus}: {count} messages ({rate:.1f} msg/s)")
            print(f"[SUMMARY] Total time: {elapsed:.2f}s")

    return 0

if __name__ == "__main__":
    sys.exit(main())

