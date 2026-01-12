#!/usr/bin/env python3
"""
Debug script for radar-vision matching on VinFast
Shows radar tracks, vision leads, and matching probabilities
"""

import math
import cereal.messaging as messaging
from cereal import car, log
from openpilot.common.params import Params
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA, laplacian_pdf

def calculate_match_probability(radar_dRel, radar_yRel, radar_vRel, vision_x, vision_y, vision_v,
                                vision_xStd, vision_yStd, vision_vStd, v_ego, is_vinfast=False):
    """Calculate matching probability between radar track and vision lead"""
    offset_vision_dist = vision_x[0] - RADAR_TO_CAMERA

    # For VinFast: Weight radar more by using 1.5x std dev (makes matching more lenient)
    if is_vinfast:
        prob_d = laplacian_pdf(radar_dRel, offset_vision_dist, vision_xStd[0] * 1.5)
        prob_y = laplacian_pdf(radar_yRel, -vision_y[0], vision_yStd[0] * 1.5)
        prob_v = laplacian_pdf(radar_vRel + v_ego, vision_v[0], vision_vStd[0] * 1.5)
    else:
        prob_d = laplacian_pdf(radar_dRel, offset_vision_dist, vision_xStd[0])
        prob_y = laplacian_pdf(radar_yRel, -vision_y[0], vision_yStd[0])
        prob_v = laplacian_pdf(radar_vRel + v_ego, vision_v[0], vision_vStd[0])

    return prob_d * prob_y * prob_v, prob_d, prob_y, prob_v

def check_sanity(radar_dRel, radar_yRel, radar_vRel, vision_x, vision_y, vision_v, v_ego, is_vinfast=False):
    """Check if match passes sanity checks"""
    offset_vision_dist = vision_x[0] - RADAR_TO_CAMERA

    if is_vinfast:
        # Updated thresholds to trust radar more: 50% distance tolerance, 20 m/s velocity tolerance
        dist_sane = abs(radar_dRel - offset_vision_dist) < max([(offset_vision_dist)*.50, 10.0])
        vel_sane = (abs(radar_vRel + v_ego - vision_v[0]) < 20) or (v_ego + radar_vRel > 1.5)
        yrel_sane = abs(radar_yRel - (-vision_y[0])) < max([abs(vision_y[0]) * 0.5 + 0.5, 1.5])
    else:
        dist_sane = abs(radar_dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
        vel_sane = (abs(radar_vRel + v_ego - vision_v[0]) < 10) or (v_ego + radar_vRel > 3)
        yrel_sane = abs(radar_yRel - (-vision_y[0])) < max([abs(vision_y[0]) * 0.5 + 0.5, 1.5])

    return dist_sane, vel_sane, yrel_sane, offset_vision_dist

def format_distance(d):
    """Format distance in meters"""
    return f"{d:6.2f}m"

def format_velocity(v):
    """Format velocity in m/s"""
    return f"{v:6.2f}m/s"

def format_probability(p):
    """Format probability"""
    return f"{p:.4f}"

def main():
    print("=" * 100)
    print("Radar-Vision Matching Debug Tool for VinFast")
    print("=" * 100)
    print()

    # Get car params
    params = Params()
    CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
    is_vinfast = CP.brand == "vinfast"
    print(f"Car: {CP.brand} ({CP.carFingerprint})")
    print(f"RADAR_TO_CAMERA offset: {RADAR_TO_CAMERA}m")
    print()

    # Subscribe to messages
    sm = messaging.SubMaster(['radarState', 'modelV2', 'carState'], poll='radarState')

    frame_count = 0
    last_print_time = 0

    print("Waiting for messages... (Press Ctrl+C to exit)")
    print()

    try:
        while True:
            sm.update(1000)

            if not sm.updated['radarState'] and not sm.updated['modelV2']:
                continue

            frame_count += 1
            current_time = sm.logMonoTime.get('radarState', 0) / 1e9

            # Only print every 0.5 seconds to avoid spam
            if current_time - last_print_time < 0.5:
                continue
            last_print_time = current_time

            # Clear screen (optional - comment out if you want to see history)
            # print("\033[2J\033[H", end="")

            print("=" * 100)
            print(f"Frame: {frame_count} | Time: {current_time:.2f}s")
            print("=" * 100)

            # Get car state
            v_ego = sm['carState'].vEgo if sm.valid['carState'] else 0.0
            print(f"\nVehicle Speed: {format_velocity(v_ego)} ({v_ego * 3.6:.1f} km/h)")

            # Get radar state
            radar_state = sm['radarState']
            lead_one = radar_state.leadOne

            print(f"\n{'RADAR STATE':-^100}")
            print(f"Lead One Status: {lead_one.status}")
            if lead_one.status:
                print(f"  dRel: {format_distance(lead_one.dRel)} | yRel: {format_distance(lead_one.yRel)} | vRel: {format_velocity(lead_one.vRel)}")
                print(f"  vLead: {format_velocity(lead_one.vLead)} | aLeadK: {lead_one.aLeadK:.2f} m/s²")
                print(f"  Radar Track: {lead_one.radar} | Track ID: {lead_one.radarTrackId}")
                print(f"  Model Prob: {lead_one.modelProb:.3f}")

            lead_two = radar_state.leadTwo
            if lead_two.status:
                print(f"\nLead Two Status: {lead_two.status}")
                print(f"  dRel: {format_distance(lead_two.dRel)} | yRel: {format_distance(lead_two.yRel)} | vRel: {format_velocity(lead_two.vRel)}")

            # Get vision leads
            model_v2 = sm['modelV2']
            leads_v3 = model_v2.leadsV3

            print(f"\n{'VISION LEADS':-^100}")
            print(f"Number of vision leads: {len(leads_v3)}")

            if len(leads_v3) > 0:
                # Capnp lists don't support slicing, so iterate with a counter
                max_leads = min(3, len(leads_v3))
                for i in range(max_leads):
                    lead = leads_v3[i]
                    print(f"\nVision Lead {i}:")
                    print(f"  Prob: {lead.prob:.3f}")
                    if len(lead.x) > 0:
                        print(f"  x[0]: {format_distance(lead.x[0])} | xStd[0]: {format_distance(lead.xStd[0])}")
                        print(f"  y[0]: {format_distance(lead.y[0])} | yStd[0]: {format_distance(lead.yStd[0])}")
                        print(f"  v[0]: {format_velocity(lead.v[0])} | vStd[0]: {format_velocity(lead.vStd[0])}")
                        print(f"  Offset vision dist (x - RADAR_TO_CAMERA): {format_distance(lead.x[0] - RADAR_TO_CAMERA)}")

            # Get radar tracks from liveTracks (if available)
            # Note: We need to get raw radar data, which might not be directly available
            # For now, we'll analyze the matching between leadOne and vision leads

            if lead_one.status and len(leads_v3) > 0:
                vision_lead = leads_v3[0]
                if len(vision_lead.x) > 0:
                    print(f"\n{'MATCHING ANALYSIS':-^100}")
                    print(f"Comparing Radar Lead One with Vision Lead 0:")

                    # Calculate matching probability
                    total_prob, prob_d, prob_y, prob_v = calculate_match_probability(
                        lead_one.dRel, lead_one.yRel, lead_one.vRel,
                        vision_lead.x, vision_lead.y, vision_lead.v,
                        vision_lead.xStd, vision_lead.yStd, vision_lead.vStd,
                        v_ego, is_vinfast
                    )

                    print(f"  Total Probability: {format_probability(total_prob)}")
                    print(f"    - Distance prob: {format_probability(prob_d)}")
                    print(f"    - Lateral prob:  {format_probability(prob_y)}")
                    print(f"    - Velocity prob: {format_probability(prob_v)}")

                    # Check sanity
                    dist_sane, vel_sane, yrel_sane, offset_dist = check_sanity(
                        lead_one.dRel, lead_one.yRel, lead_one.vRel,
                        vision_lead.x, vision_lead.y, vision_lead.v,
                        v_ego, is_vinfast
                    )

                    print(f"\n  Sanity Checks:")
                    dist_threshold = max([offset_dist*.50 if is_vinfast else offset_dist*.25, 10.0 if is_vinfast else 5.0])
                    print(f"    Distance: {dist_sane} (|{format_distance(lead_one.dRel)} - {format_distance(offset_dist)}| < {format_distance(dist_threshold)})")
                    print(f"    Velocity: {vel_sane} (|{format_velocity(lead_one.vRel + v_ego)} - {format_velocity(vision_lead.v[0])}| < 20 or v_ego+vRel > 1.5)")
                    lateral_threshold = max([abs(vision_lead.y[0])*0.5+0.5, 1.5])
                    print(f"    Lateral:  {yrel_sane} (|{format_distance(lead_one.yRel)} - {format_distance(-vision_lead.y[0])}| < {format_distance(lateral_threshold)})")

                    match_passed = dist_sane and vel_sane and yrel_sane
                    print(f"\n  Match Result: {'✓ PASSED' if match_passed else '✗ FAILED'}")

                    if not match_passed:
                        print(f"  Reasons:")
                        if not dist_sane:
                            print(f"    - Distance mismatch: {abs(lead_one.dRel - offset_dist):.2f}m difference")
                        if not vel_sane:
                            print(f"    - Velocity mismatch: {abs(lead_one.vRel + v_ego - vision_lead.v[0]):.2f}m/s difference")
                        if not yrel_sane:
                            print(f"    - Lateral mismatch: {abs(lead_one.yRel - (-vision_lead.y[0])):.2f}m difference (adjacent lane?)")

            # Show adjacent lane detection
            if len(leads_v3) > 0 and lead_one.status:
                vision_lead = leads_v3[0]
                if len(vision_lead.y) > 0:
                    lateral_offset = abs(lead_one.yRel - (-vision_lead.y[0]))
                    print(f"\n{'ADJACENT LANE CHECK':-^100}")
                    print(f"Lateral offset: {format_distance(lateral_offset)}")
                    if lateral_offset > 1.5:
                        print(f"⚠️  WARNING: Large lateral offset - likely adjacent lane vehicle!")
                    elif lateral_offset > 1.0:
                        print(f"⚠️  CAUTION: Moderate lateral offset - may be adjacent lane")
                    else:
                        print(f"✓ Lateral offset within acceptable range")

            print("\n" + "=" * 100)
            print()

    except KeyboardInterrupt:
        print("\n\nExiting...")

if __name__ == "__main__":
    main()

