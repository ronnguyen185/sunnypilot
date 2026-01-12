#!/usr/bin/env python3
"""
Debug script for speed limit detection on VinFast
Checks all sources of speed limit data to identify why UI shows speed limit
when carstate doesn't read it from the car
"""

import cereal.messaging as messaging
from cereal import car, log
from openpilot.common.params import Params
from openpilot.common.constants import CV

def format_speed_ms_to_kph(speed_ms):
    """Format speed in m/s to km/h"""
    return speed_ms * CV.MS_TO_KPH

def format_speed_ms_to_mph(speed_ms):
    """Format speed in m/s to mph"""
    return speed_ms * CV.MS_TO_MPH

def main():
    print("=" * 100)
    print("Speed Limit Debug Tool for VinFast")
    print("=" * 100)
    print()

    # Get car params
    params = Params()
    CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
    print(f"Car: {CP.brand} ({CP.carFingerprint})")
    print()

    # Subscribe to messages
    sm = messaging.SubMaster([
        'carState',
        'carStateSP',  # SunnyPilot extended car state
        'liveMapDataSP',  # Map-based speed limits
        'longitudinalPlan',  # May contain speed limit info
        'longitudinalPlanSP',  # SunnyPilot extended - contains cached speed limit values
        'driverAssistance',  # May contain speed limit info
    ], poll='carState')

    frame_count = 0
    last_print_time = 0

    print("Waiting for messages... (Press Ctrl+C to exit)")
    print()

    try:
        while True:
            sm.update(1000)

            if not sm.updated['carState']:
                continue

            frame_count += 1
            current_time = sm.logMonoTime.get('carState', 0) / 1e9

            # Only print every 0.5 seconds to avoid spam
            if current_time - last_print_time < 0.5:
                continue
            last_print_time = current_time

            print("=" * 100)
            print(f"Frame: {frame_count} | Time: {current_time:.2f}s")
            print("=" * 100)

            # Get car state
            car_state = sm['carState']
            print(f"\n{'CAR STATE (Base)':-^100}")
            print(f"Speed: {format_speed_ms_to_kph(car_state.vEgo):.1f} km/h")
            print("Note: carState does not have speedLimit field (it's in carStateSP)")

            # Get SunnyPilot extended car state
            if sm.valid['carStateSP']:
                car_state_sp = sm['carStateSP']
                print(f"\n{'CAR STATE SP (SunnyPilot Extended)':-^100}")
                print(f"Speed Limit: {format_speed_ms_to_kph(car_state_sp.speedLimit):.1f} km/h" if car_state_sp.speedLimit > 0 else "Speed Limit: 0.0 km/h (NOT SET)")
            else:
                print(f"\n{'CAR STATE SP':-^100}")
                print("Not available")

            # Get map data speed limit
            if sm.valid['liveMapDataSP']:
                map_data = sm['liveMapDataSP']
                print(f"\n{'MAP DATA (SunnyPilot)':-^100}")
                print(f"Speed Limit Valid: {map_data.speedLimitValid}")
                print(f"Speed Limit: {format_speed_ms_to_kph(map_data.speedLimit):.1f} km/h" if map_data.speedLimit > 0 else "Speed Limit: 0.0 km/h (NOT SET)")
                print(f"Speed Limit Ahead: {format_speed_ms_to_kph(map_data.speedLimitAhead):.1f} km/h" if map_data.speedLimitAhead > 0 else "Speed Limit Ahead: 0.0 km/h (NOT SET)")
                print(f"Speed Limit Ahead Distance: {map_data.speedLimitAheadDistance:.1f} m")
            else:
                print(f"\n{'MAP DATA':-^100}")
                print("Not available")

            # Get longitudinal plan (may contain speed limit info)
            if sm.valid['longitudinalPlan']:
                long_plan = sm['longitudinalPlan']
                print(f"\n{'LONGITUDINAL PLAN':-^100}")
                # Check if there's speed limit info in longitudinal plan
                if hasattr(long_plan, 'speedLimit'):
                    print(f"Speed Limit: {format_speed_ms_to_kph(long_plan.speedLimit):.1f} km/h" if long_plan.speedLimit > 0 else "Speed Limit: 0.0 km/h (NOT SET)")
                else:
                    print("No speed limit field in longitudinalPlan")
            else:
                print(f"\n{'LONGITUDINAL PLAN':-^100}")
                print("Not available")

            # Get SunnyPilot longitudinal plan (contains cached speed limit values)
            if sm.valid['longitudinalPlanSP']:
                long_plan_sp = sm['longitudinalPlanSP']
                print(f"\n{'LONGITUDINAL PLAN SP (SunnyPilot)':-^100}")
                if hasattr(long_plan_sp, 'speedLimit'):
                    speed_limit_info = long_plan_sp.speedLimit
                    if hasattr(speed_limit_info, 'resolver'):
                        resolver = speed_limit_info.resolver
                        print(f"Resolver Speed Limit: {format_speed_ms_to_kph(resolver.speedLimit):.1f} km/h" if resolver.speedLimit > 0 else "Resolver Speed Limit: 0.0 km/h (NOT SET)")
                        print(f"Resolver Speed Limit Last (CACHED): {format_speed_ms_to_kph(resolver.speedLimitLast):.1f} km/h" if resolver.speedLimitLast > 0 else "Resolver Speed Limit Last: 0.0 km/h (NOT SET)")
                        print(f"Resolver Speed Limit Final: {format_speed_ms_to_kph(resolver.speedLimitFinal):.1f} km/h" if resolver.speedLimitFinal > 0 else "Resolver Speed Limit Final: 0.0 km/h (NOT SET)")
                        print(f"Resolver Speed Limit Final Last (CACHED): {format_speed_ms_to_kph(resolver.speedLimitFinalLast):.1f} km/h" if resolver.speedLimitFinalLast > 0 else "Resolver Speed Limit Final Last: 0.0 km/h (NOT SET)")
                        print(f"Speed Limit Valid: {resolver.speedLimitValid}")
                        print(f"Speed Limit Last Valid: {resolver.speedLimitLastValid}")
                        print(f"Source: {resolver.source}")
                    else:
                        print("No resolver field in speedLimit")
                else:
                    print("No speedLimit field in longitudinalPlanSP")
            else:
                print(f"\n{'LONGITUDINAL PLAN SP':-^100}")
                print("Not available")

            # Get driver assistance (may contain speed limit info)
            if sm.valid['driverAssistance']:
                driver_assist = sm['driverAssistance']
                print(f"\n{'DRIVER ASSISTANCE':-^100}")
                if hasattr(driver_assist, 'speedLimit'):
                    print(f"Speed Limit: {format_speed_ms_to_kph(driver_assist.speedLimit):.1f} km/h" if driver_assist.speedLimit > 0 else "Speed Limit: 0.0 km/h (NOT SET)")
                else:
                    print("No speed limit field in driverAssistance")
            else:
                print(f"\n{'DRIVER ASSISTANCE':-^100}")
                print("Not available")

            # Check which source is actually being used
            print(f"\n{'SPEED LIMIT SOURCE ANALYSIS':-^100}")
            sources = []

            if sm.valid['carStateSP'] and car_state_sp.speedLimit > 0:
                sources.append(f"carStateSP.speedLimit: {format_speed_ms_to_kph(car_state_sp.speedLimit):.1f} km/h")

            if sm.valid['liveMapDataSP'] and map_data.speedLimitValid and map_data.speedLimit > 0:
                sources.append(f"liveMapDataSP.speedLimit: {format_speed_ms_to_kph(map_data.speedLimit):.1f} km/h")

            # Check cached values in longitudinalPlanSP
            if sm.valid['longitudinalPlanSP']:
                long_plan_sp = sm['longitudinalPlanSP']
                if hasattr(long_plan_sp, 'speedLimit') and hasattr(long_plan_sp.speedLimit, 'resolver'):
                    resolver = long_plan_sp.speedLimit.resolver
                    if resolver.speedLimitLast > 0:
                        sources.append(f"longitudinalPlanSP.resolver.speedLimitLast (CACHED): {format_speed_ms_to_kph(resolver.speedLimitLast):.1f} km/h")
                    if resolver.speedLimitFinalLast > 0:
                        sources.append(f"longitudinalPlanSP.resolver.speedLimitFinalLast (CACHED): {format_speed_ms_to_kph(resolver.speedLimitFinalLast):.1f} km/h")

            if not sources:
                print("⚠️  NO SPEED LIMIT SOURCES ACTIVE")
                print("   This means the UI should NOT be showing a speed limit")
            else:
                print("Active speed limit sources (including cached values):")
                for i, source in enumerate(sources, 1):
                    print(f"  {i}. {source}")

                # Check if 60 km/h is showing
                for source in sources:
                    if "60" in source or "60.0" in source:
                        print(f"\n⚠️  FOUND 60 km/h in: {source}")
                        if "CACHED" in source:
                            print("   ⚠️  THIS IS A CACHED VALUE!")
                            print("   The UI is likely displaying the cached 'speedLimitLast' value")
                            print("   even though the current speed limit is 0")
                        else:
                            print("   This is likely what the UI is displaying!")

            # Check TSR message status (if we can access raw CAN data)
            print(f"\n{'TSR MESSAGE STATUS':-^100}")
            print("Note: TSR (Traffic Sign Recognition) data comes from ADAS_TSR_STATUS CAN message")
            print("      If carState.speedLimit is 0, TSR message is either:")
            print("      - Not available (message not received)")
            print("      - Typ1 != 1 (not a speed limit sign)")
            print("      - Typ1_value == 0 or out of range (invalid speed value)")
            print("      - Typ1 == 2 (end of speed limit sign)")

            print("\n" + "=" * 100)
            print()

    except KeyboardInterrupt:
        print("\n\nExiting...")

if __name__ == "__main__":
    main()

