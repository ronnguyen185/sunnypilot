import argparse
import select
import sys
import termios
import threading
import time
import tty

from panda import Panda

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.structs import CarParams, CarParamsSP
from opendbc.car.vinfast.vinfastcan import (
    create_acc_control,
    create_lka_control,
    create_steering_control,
)
from opendbc.car.vinfast.values import CAR
from opendbc.car.vinfast.carstate import CarState

# VinFast CAN buses (matches safety_vinfast.h defaults)
VINFAST_CHASSIS_BUS = 2
VINFAST_CAMERA_BUS = 0

# Steering constraints
STEER_MIN = -470.0
STEER_MAX = 470.0
STEER_STEP = 3.0
LAT_ACTIVATION_DELAY_FRAMES = 200  # wait 2 seconds at 100Hz
FINE_CONTROL_THRESHOLD = 80.0
FINE_CONTROL_STEP = 10.0
CENTER_RETURN_STEP = 2.0
CENTER_TARGET_STEP = 2.0

# Acceleration limits (m/s^2)
ACCEL_MIN = -3.5
ACCEL_MAX = 2.0
ACCEL_STEP = 0.1

# Loop frequency
DESIRED_FREQUENCY = 100.0  # Hz
PERIOD = 1.0 / DESIRED_FREQUENCY

stop_event = threading.Event()

steer_angle_lock = threading.Lock()
steer_angle_target = 0.0
steer_angle_current = 0.0
auto_center_active = False

accel_lock = threading.Lock()
accel_value = 0.0

# CarState display
carstate_lock = threading.Lock()
last_carstate_update = 0.0


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))


def format_value(value, unit=""):
    """Format a value with unit, handling None and floats."""
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.2f}{unit}"
    return f"{value}{unit}"


def print_car_state_compact(cs):
    """Print compact car state information for real-time display."""
    print("\n" + "─" * 80)
    print("CAR STATE")
    print("─" * 80)

    # Vehicle speed
    speed_kph = cs.vEgo * 3.6 if cs.vEgo else 0.0
    print(f"Speed: {format_value(cs.vEgo, ' m/s')} ({format_value(speed_kph, ' km/h')}) | "
          f"Standstill: {cs.standstill if hasattr(cs, 'standstill') else 'N/A'}")

    # Steering
    print(f"Steering: {format_value(cs.steeringAngleDeg, '°')} | "
          f"Rate: {format_value(cs.steeringRateDeg, '°/s')} | "
          f"Torque: {format_value(cs.steeringTorque, ' Nm')}")

    # Wheel speeds
    if hasattr(cs, 'wheelSpeeds') and cs.wheelSpeeds:
        ws = cs.wheelSpeeds
        print(f"Wheels: FL={format_value(ws.fl, ' m/s')} FR={format_value(ws.fr, ' m/s')} "
              f"RL={format_value(ws.rl, ' m/s')} RR={format_value(ws.rr, ' m/s')}")

    # Cruise control
    if hasattr(cs, 'cruiseState'):
        cruise = cs.cruiseState
        print(f"Cruise: Available={cruise.available if hasattr(cruise, 'available') else 'N/A'} | "
              f"Enabled={cruise.enabled if hasattr(cruise, 'enabled') else 'N/A'}")

    print("─" * 80)


def heartbeat_thread(panda: Panda):
    """Keep Panda safety happy by sending a heartbeat at 10Hz."""
    while not stop_event.is_set():
        try:
            panda.send_heartbeat(engaged=True)
            time.sleep(0.1)
        except Exception as e:
            print(f"[HEARTBEAT] Error: {e}")
            break


def keyboard_listener():
    """
    Non-blocking keyboard controls:
      - 'a'/'d': steer left/right
      - 's': smooth return to center
      - 'w'/'x': increase/decrease acceleration
      - 'q': quit
    """
    global steer_angle_target, auto_center_active, accel_value

    print(
        "[KEYBOARD] Controls: 'a'=left, 'd'=right, 's'=center, "
        "'w'=accelerate, 'x'=decelerate, 'q'=quit"
    )

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        while not stop_event.is_set():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not rlist:
                continue

            ch = sys.stdin.read(1).lower()

            if ch == "a":
                with steer_angle_lock:
                    auto_center_active = False
                    step = (
                        FINE_CONTROL_STEP
                        if abs(steer_angle_target) <= FINE_CONTROL_THRESHOLD
                        else STEER_STEP
                    )
                    steer_angle_target = clamp(
                        steer_angle_target - step, STEER_MIN, STEER_MAX
                    )
                print(f"[KEYBOARD] Target steer angle: {steer_angle_target:.1f} deg")

            elif ch == "d":
                with steer_angle_lock:
                    auto_center_active = False
                    step = (
                        FINE_CONTROL_STEP
                        if abs(steer_angle_target) <= FINE_CONTROL_THRESHOLD
                        else STEER_STEP
                    )
                    steer_angle_target = clamp(
                        steer_angle_target + step, STEER_MIN, STEER_MAX
                    )
                print(f"[KEYBOARD] Target steer angle: {steer_angle_target:.1f} deg")

            elif ch == "s":
                with steer_angle_lock:
                    auto_center_active = True
                with accel_lock:
                    accel_value = 0.0
                print("[KEYBOARD] Smooth center return enabled, acceleration set to 0")

            elif ch == "w":
                with accel_lock:
                    accel_value = clamp(accel_value + ACCEL_STEP, ACCEL_MIN, ACCEL_MAX)
                print(f"[KEYBOARD] Acceleration: {accel_value:.1f} m/s²")

            elif ch == "x":
                with accel_lock:
                    accel_value = clamp(accel_value - ACCEL_STEP, ACCEL_MIN, ACCEL_MAX)
                print(f"[KEYBOARD] Acceleration: {accel_value:.1f} m/s²")

            elif ch == "q":
                print("[KEYBOARD] Quit requested")
                stop_event.set()
                break

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def send_loop(panda: Panda, packer: CANPacker, car_state: CarState, can_parsers: dict, CP: CarParams,
              rx_debug: bool = False, carstate_rate: float = 1.0):
    """
    Main control loop:
      - EPS_LATE_CON (0x37A) @ 100Hz on chassis bus
      - ADAS_LKA      (0x132) @ 20Hz on camera bus
      - ADAS_ACC_STATUS (0x32D) @ 20Hz on camera bus (optional)
      - CarState parsing and display
    """
    global steer_angle_current, steer_angle_target, auto_center_active, accel_value
    global last_carstate_update

    frame = 0

    print("[SEND] Starting control loop (100Hz)")

    # Track time more accurately to prevent drift
    next_frame_time = time.time()

    try:
        while not stop_event.is_set():
            # 1. CRITICAL: Always drain the RX buffer to prevent USB lag/bursts
            incoming_messages = panda.can_recv()

            # Parse incoming messages for CarState
            current_time = time.time()
            timestamp_nanos = int(current_time * 1e9)
            frames_for_parser = []

            for address, dat, src in incoming_messages:
                # Collect messages from chassis bus for parsing
                if src == VINFAST_CHASSIS_BUS:
                    frames_for_parser.append((address, dat, src))

                if rx_debug:
                    # Filter for specific IDs to avoid flooding console
                    if address in (0x37A, 0x132, 0x32D):
                        print(f"[RX] ID: 0x{address:X}, Bus: {src}, Data: {bytes(dat).hex()}")

            # Update CAN parsers with received messages
            if frames_for_parser and can_parsers:
                if Bus.chassis in can_parsers:
                    parser = can_parsers[Bus.chassis]
                    parser.update([(timestamp_nanos, frames_for_parser)])

            # Update and display CarState periodically
            if car_state and can_parsers and (current_time - last_carstate_update) >= carstate_rate:
                try:
                    cs, cs_sp = car_state.update(can_parsers)
                    print_car_state_compact(cs)
                    last_carstate_update = current_time
                except Exception as e:
                    if rx_debug:
                        print(f"[CARSTATE] Error updating: {e}")

            # 2. Control Logic
            with accel_lock:
                accel_cmd = accel_value

            with steer_angle_lock:
                # [Logic remains the same as your original script...]
                if auto_center_active:
                    if abs(steer_angle_target) <= CENTER_TARGET_STEP:
                        steer_angle_target = 0.0
                        auto_center_active = False
                    else:
                        steer_angle_target -= (
                            CENTER_TARGET_STEP
                            if steer_angle_target > 0
                            else -CENTER_TARGET_STEP
                        )

                error = steer_angle_target - steer_angle_current
                if error != 0.0:
                    if steer_angle_target == 0.0:
                        step = (
                            FINE_CONTROL_STEP
                            if abs(steer_angle_current) <= FINE_CONTROL_THRESHOLD
                            else CENTER_RETURN_STEP
                        )
                    else:
                        step = (
                            FINE_CONTROL_STEP
                            if abs(steer_angle_current) <= FINE_CONTROL_THRESHOLD
                            else STEER_STEP
                        )

                    if abs(error) <= step:
                        steer_angle_current = steer_angle_target
                    else:
                        steer_angle_current += step if error > 0 else -step

                apply_angle = clamp(steer_angle_current, STEER_MIN, STEER_MAX)

            lat_active = frame >= LAT_ACTIVATION_DELAY_FRAMES
            long_active = True
            standstill = False

            # 3. Packing & Sending
            # EPS_LATE_CON @ 100Hz
            eps_addr, eps_data, _ = create_steering_control(
                packer, CP, frame, -apply_angle, lat_active
            )
            panda.can_send(eps_addr, eps_data, VINFAST_CHASSIS_BUS)

            if frame % 2 == 0:

                # ADAS_ACC_STATUS @ 50Hz
                acc_addr, acc_data, _ = create_acc_control(
                    packer, CP, frame // 2, accel_cmd, long_active, standstill,
                )
                panda.can_send(acc_addr, acc_data, VINFAST_CHASSIS_BUS)

            frame += 1

            # 4. Accurate Timing Loop
            # Calculate how much sleep is needed to hit exactly the next 10ms mark
            next_frame_time += PERIOD
            sleep_time = next_frame_time - time.time()

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we are falling behind, reset the clock to avoid trying to catch up fast
                # This prevents "dense" bursts if the computer lags
                if frame % 100 == 0:
                    print(f"[WARN] Lagging behind by {-sleep_time*1000:.2f}ms")
                next_frame_time = time.time()

    except KeyboardInterrupt:
        print("[SEND] KeyboardInterrupt")
        stop_event.set()
    except Exception as e:
        print(f"[SEND] Error: {e}")
        import traceback
        traceback.print_exc()


def parse_args():
    parser = argparse.ArgumentParser(description="VinFast keyboard control tester")
    parser.add_argument(
        "--safety",
        choices=["vinfast", "alloutput"],
        default="vinfast",
        help="Panda safety mode to use",
    )
    parser.add_argument(
        "--rx-debug",
        action="store_true",
        help="Print received CAN frames for debugging",
    )
    parser.add_argument(
        "--carstate-rate",
        type=float,
        default=1.0,
        help="CarState display update rate in seconds (default: 1.0)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    panda = None
    hb_thread = None
    keyboard_thread = None

    try:
        print("[MAIN] Connecting to Panda...")
        panda = Panda()

        try:
            panda.set_power_save(False)
        except Exception as e:
            print(f"[MAIN] set_power_save failed (not critical): {e}")

        # Initialize CANPacker
        dbc_name = CAR.VINFAST_VF8.config.dbc_dict[Bus.chassis]
        packer = CANPacker(dbc_name)
        print(f"[MAIN] Using DBC: {dbc_name}")

        # Initialize CarState for parsing incoming messages
        CP = CarParams.new_message()
        CP.carFingerprint = "VINFAST_VF8"
        CP.brand = "vinfast"

        CP_SP = CarParamsSP()

        car_state = CarState(CP, CP_SP)
        can_parsers = car_state.get_can_parsers(CP, CP_SP)
        print(f"[MAIN] CarState initialized, parsing on bus {VINFAST_CHASSIS_BUS}")
        print(f"[MAIN] CarState display rate: {args.carstate_rate} seconds")

        # Start heartbeat thread
        hb_thread = threading.Thread(target=heartbeat_thread, args=(panda,), daemon=True)
        hb_thread.start()

        # Set requested safety mode
        safety_mode = (
            CarParams.SafetyModel.vinfast
            if args.safety == "vinfast"
            else CarParams.SafetyModel.allOutput
        )

        # Set safety parameter: bit 0 = longitudinal control enabled
        # When enabled, ACC_STATUS forwarding from bus 0 to bus 2 will be blocked
        VINFAST_PARAM_LONGITUDINAL = 1  # bit 0
        safety_param = VINFAST_PARAM_LONGITUDINAL if args.safety == "vinfast" else 0

        print(f"[MAIN] Setting safety mode: {args.safety} (param={safety_param})")
        panda.set_safety_mode(safety_mode, param=safety_param)
        panda.send_heartbeat(engaged=True)


        # Start keyboard listener
        keyboard_thread = threading.Thread(
            target=keyboard_listener, name="keyboard-listener", daemon=True
        )
        keyboard_thread.start()

        # Run send loop
        send_loop(panda, packer, car_state, can_parsers, CP,
                  rx_debug=args.rx_debug, carstate_rate=args.carstate_rate)

    finally:
        stop_event.set()
        if hb_thread:
            hb_thread.join(timeout=0.5)
        if keyboard_thread:
            keyboard_thread.join(timeout=0.5)
        if panda:
            try:
                panda.set_safety_mode(CarParams.SafetyModel.silent)
            except Exception:
                pass

        print("[MAIN] Shutdown complete")


if __name__ == "__main__":
    main()
