#!/usr/bin/env python3
"""
Radar interface for VinFast VF8 MRR (Multi-Range Radar) with SCAM integration.
Parses CAN-FD messages from bus 1 (radar bus).
"""

import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.vinfast.values import DBC, CANBUS
from openpilot.common.swaglog import cloudlog


# Radar object blocks (CAN FD) containing fused radar/camera objects.
# Addresses 0x410-0x419 (decimal 1040-1049), two objects per frame.
RADAR_OD_MSGS = list(range(0x410, 0x41A))


def _create_radar_can_parser(car_fingerprint):
    """Create CAN parser for VinFast MRR radar messages."""
    try:
        if Bus.radar not in DBC[car_fingerprint]:
            cloudlog.warning(f"VinFast Radar: Bus.radar not in DBC for {car_fingerprint}")
            return None

        # Parse radar messages at 18 Hz (actual rate from radar)
        # Parse radar object blocks (camera/radar fused) at 18 Hz.
        # Each frame carries two objects.
        # Use slightly lower rate (18Hz) to match actual radar output and avoid false "low rate" warnings
        messages = [(addr, 18) for addr in RADAR_OD_MSGS]


        return CANParser(DBC[car_fingerprint][Bus.radar], messages, CANBUS.radar)
    except Exception as e:
        cloudlog.error(f"VinFast Radar: Failed to create CAN parser: {e}")
        return None


class RadarInterface(RadarInterfaceBase):
    def __init__(self, CP, CP_SP):
        super().__init__(CP, CP_SP)

        self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP.carFingerprint)
        # Trigger on first OD block
        self.trigger_msg = RADAR_OD_MSGS[0]
        self.updated_messages = set()
        self.track_id = 0
        self.diagnostic_counter = 0
        self.last_diagnostic_log = 0
        self.bus1_msg_count_total = 0
        self.bus1_msg_count_window = 0
        self.last_msg_count_reset = 0
        self.last_bus_reinit_attempt = 0
        self.consecutive_empty_updates = 0
        self.last_object_log = 0

        if self.rcp is None:
            cloudlog.warning("VinFast Radar: Interface disabled (radarUnavailable=True)")
        else:
            cloudlog.info(f"VinFast Radar: Initialized parser for bus {CANBUS.radar}, trigger msg: {self.trigger_msg}")

    def update(self, can_strings):
        """Update radar data from CAN messages."""
        if self.rcp is None:
            return super().update(None)

        # Lightweight message counting (only when needed for diagnostics)
        import time
        current_time = time.time()

        # Only do detailed counting/logging every 30 seconds to avoid performance impact
        # With ~2400 msgs/sec, processing all messages every update causes lag
        do_detailed_diagnostics = (current_time - self.last_diagnostic_log) >= 30.0

        bus1_msg_count_this_update = 0
        bus1_addrs = set()  # Always initialize, but only populate when needed
        radar_addrs_all_buses = set()  # Always initialize, but only populate when needed

        if do_detailed_diagnostics:
            # Only do full message processing for diagnostics every 30 seconds
            if can_strings:
                for entry in can_strings:
                    if isinstance(entry, (list, tuple)) and len(entry) >= 2:
                        frames = entry[1] if len(entry) > 1 else []
                        for frame in frames:
                            if len(frame) >= 3:
                                addr, dat, src = frame[0], frame[1], frame[2]

                                # Check if this is a radar message on any bus
                                if addr in [400, 401, 402, 403]:
                                    radar_addrs_all_buses.add((addr, src))

                                if src == CANBUS.radar:
                                    bus1_msg_count_this_update += 1
                                    self.bus1_msg_count_window += 1
                                    self.bus1_msg_count_total += 1
                                    bus1_addrs.add(addr)
        else:
            # Lightweight counting: just count bus 1 messages without full processing
            if can_strings:
                for entry in can_strings:
                    if isinstance(entry, (list, tuple)) and len(entry) >= 2:
                        frames = entry[1] if len(entry) > 1 else []
                        for frame in frames:
                            if len(frame) >= 3 and frame[2] == CANBUS.radar:
                                bus1_msg_count_this_update += 1
                                self.bus1_msg_count_window += 1
                                self.bus1_msg_count_total += 1

        # Track if bus 1 is receiving messages
        if bus1_msg_count_this_update == 0:
            self.consecutive_empty_updates += 1
        else:
            self.consecutive_empty_updates = 0

        # If bus 1 has been empty for 2 seconds (200 updates at 100Hz), log warning
        # This is a sign that bus 1 might have been disabled
        # Note: Configuration should persist in panda firmware, but if bus gets disabled,
        # we'll detect it here and the user can check logs
        if (self.consecutive_empty_updates > 200 and
            current_time - self.last_bus_reinit_attempt > 10.0):
            self.last_bus_reinit_attempt = current_time
            cloudlog.error(f"VinFast Radar: Bus {CANBUS.radar} has been empty for {self.consecutive_empty_updates} updates (~{self.consecutive_empty_updates/100:.1f}s)")
            cloudlog.error(f"VinFast Radar: Bus may be disabled. Check if power saving or safety mode disabled it. Configuration should persist, but bus transceiver may be off.")

        # Reset window counter every second for rate calculation
        msg_rate = 0
        if current_time - self.last_msg_count_reset >= 1.0:
            self.last_msg_count_reset = current_time
            msg_rate = self.bus1_msg_count_window
            self.bus1_msg_count_window = 0

        # Detailed diagnostics only every 30 seconds to avoid performance impact
        if do_detailed_diagnostics:
            self.last_diagnostic_log = current_time
            radar_addrs_seen = sorted([a for a in bus1_addrs if a in RADAR_OD_MSGS])
            cloudlog.info(f"VinFast Radar: Bus {CANBUS.radar} - {msg_rate} msgs/sec, total={self.bus1_msg_count_total}, radar addresses (0x410-0x419): {radar_addrs_seen if radar_addrs_seen else 'NONE'}")

            # Removed debug warning about missing expected addresses
            # if self.rcp is not None and hasattr(self.rcp, 'addresses') and self.rcp.addresses:
            #     missing_addrs = [a for a in RADAR_OD_MSGS if a not in bus1_addrs]
            #     if missing_addrs:
            #         cloudlog.warning(f"VinFast Radar: Missing expected addresses on bus {CANBUS.radar}: {missing_addrs}")

        # Check again that rcp is not None before using it
        if self.rcp is None:
            return super().update(None)

        try:
            vls = self.rcp.update(can_strings)
        except Exception as e:
            cloudlog.error(f"VinFast Radar: Error updating CAN parser: {e}")
            return super().update(None)
        self.updated_messages.update(vls)

        # Always return a RadarData object to maintain communication rate
        # Even if trigger message not received, return empty data to avoid "low communication rate" errors
        ret = structs.RadarData()

        # Wait for trigger message before processing, but still return empty RadarData
        if self.trigger_msg not in self.updated_messages:
            # Return empty RadarData instead of None to maintain communication rate
            # This prevents "low communication rate" errors when messages are slightly delayed
            return ret

        if not self.rcp.can_valid:
            ret.errors.canError = True
            if current_time - self.last_diagnostic_log > 4.0:  # Avoid duplicate logs
                cloudlog.warning("VinFast Radar: CAN bus invalid - no valid messages received")

        # Process radar messages
        self._update_radar_points(ret)

        # Optional periodic debug: log closest few objects for fusion tuning
        if current_time - self.last_object_log >= 1.0 and self.pts:
            closest = sorted(self.pts.values(), key=lambda p: p.dRel)[:6]
            summary = [{"dRel": round(p.dRel, 2), "vRel": round(p.vRel, 2)} for p in closest]
            cloudlog.debug(f"VinFast Radar: objects={len(self.pts)} closest={summary}")
            self.last_object_log = current_time

        self.updated_messages.clear()
        ret.points = list(self.pts.values())
        return ret

    def _update_radar_points(self, ret):
        """Extract radar points from parsed messages."""

        def _first_match(msg, prefix, field, default=None):
            """Return the first signal value whose key starts with prefix+field."""
            for k, v in msg.items():
                if k.startswith(f"{prefix}{field}"):
                    return v
            return default

        # Process OD blocks (0x410-0x419) carrying fused radar/camera objects.
        # Each message has two objects (objXX prefix inside the frame).
        # We extract basic longitudinal distance and relative velocity for fusion.
        for addr in RADAR_OD_MSGS:
            if addr not in self.updated_messages or addr not in self.rcp.addresses:
                continue
            try:
                msg = self.rcp.vl[addr]
                # Determine object prefixes in this frame (two per frame)
                # e.g., 1040 carries obj01/obj02, 1041 carries obj03/obj04, etc.
                base = (addr - 0x410) * 2 + 1  # 1-indexed object numbering
                prefixes = [f"obj{base:02d}_", f"obj{base+1:02d}_"]

                for pref in prefixes:
                    obj_id = _first_match(msg, pref, "ID", default=None)
                    dx = _first_match(msg, pref, "Dx", default=None)
                    # Some DBCs use Rel_Vx; keep fallback to 0.0
                    vrel = _first_match(msg, pref, "Rel_Vx", default=0.0)
                    exist_prob = _first_match(msg, pref, "ExistProb", default=1.0)
                    motion_status = _first_match(msg, pref, "Motion_Status", default=None)
                    dx_std = _first_match(msg, pref, "DxStdDev", default=None)
                    vx_std = _first_match(msg, pref, "Rel_VxStdDev", default=None)

                    # Extract azimuth angles and lateral velocity
                    phi_left = _first_match(msg, pref, "PhiLeft", default=None)
                    phi_right = _first_match(msg, pref, "PhiRight", default=None)
                    vy_rel = _first_match(msg, pref, "Rel_Vy", default=None)
                    vy_std = _first_match(msg, pref, "Rel_VyStdDev", default=None)

                    # Basic quality filters to avoid ghost / too-close artifacts
                    if dx is None or dx > 300:
                        continue

                    # Stricter filtering for very close objects (< 2m) - often ghosts, especially at low speed
                    # Based on radard.py: "Radar points closer than 0.75, are almost always glitches"
                    # But we need some margin, so filter < 0.75m completely, and apply extra checks for 0.75-2.0m
                    if dx < 0.75:
                        continue  # Definitely reject - almost always glitches

                    # For objects between 0.75-2.0m, apply stricter filtering
                    # These are often false positives at low speeds
                    if dx < 2.0:
                        # Require high existence probability for close objects
                        if exist_prob is None or exist_prob < 0.7:
                            continue
                        # Require low uncertainty for close objects
                        if dx_std is not None and dx_std > 1.0:
                            continue  # Very strict for close objects
                        if vx_std is not None and vx_std > 2.0:
                            continue  # Very strict velocity uncertainty
                        # Reject close objects with suspicious velocities (likely ghosts)
                        if abs(vrel) > 1.0:
                            continue  # Close objects shouldn't have high relative velocity

                    if exist_prob is not None and exist_prob < 0.5:
                        continue
                    if motion_status is not None and motion_status == 0:
                        continue  # static/invalid object
                    if dx_std is not None and dx_std > 30:
                        continue
                    if vx_std is not None and vx_std > 15:
                        continue
                    if vy_std is not None and vy_std > 10:
                        continue  # Reject objects with high lateral velocity uncertainty

                    # Calculate lateral distance from azimuth angle
                    # Use average of PhiLeft and PhiRight if both available, otherwise use one
                    azimuth = None
                    if phi_left is not None and phi_right is not None:
                        azimuth = (phi_left + phi_right) / 2.0
                    elif phi_left is not None:
                        azimuth = phi_left
                    elif phi_right is not None:
                        azimuth = phi_right

                    # Calculate lateral distance from azimuth and longitudinal distance
                    yrel = 0.0
                    if azimuth is not None and dx is not None:
                        # yrel = dx * sin(azimuth) for small angles, or dx * tan(azimuth)
                        # For small angles, sin(azimuth) ≈ azimuth, but use proper calculation
                        yrel = dx * math.sin(azimuth)

                        # Filter objects that are too far laterally (outside reasonable field of view)
                        # Typical radar FOV is about ±60 degrees (±1.05 radians)
                        # Filter objects beyond ±3m laterally when close, or ±10m when far
                        max_lateral_dist = 3.0 if dx < 20.0 else 10.0
                        if abs(yrel) > max_lateral_dist:
                            continue

                        # Also check azimuth angle directly - reject objects outside ±60 degrees
                        max_azimuth = math.radians(60)  # ±60 degrees
                        if abs(azimuth) > max_azimuth:
                            continue
                    else:
                        # If azimuth not available, use Rel_Vy to estimate if object is moving laterally
                        # Objects with high lateral velocity but low longitudinal velocity are likely side objects
                        if vy_rel is not None and abs(vy_rel) > 2.0 and abs(vrel) < 1.0:
                            continue  # Likely a side object, not relevant for lead tracking

                    # Additional post-yrel filtering for objects between 2.0-3.5m - common ghost distance
                    # Check after yrel is calculated to catch ghosts with lateral offset
                    if 2.0 <= dx < 3.5:
                        # Reject objects with high negative relative velocity (moving away fast)
                        # Real objects this close shouldn't be moving away at high speed
                        if vrel < -2.0:
                            continue
                        # Reject objects with significant lateral offset (>0.8m) when close
                        # Real lead vehicles should be more centered (within ±0.8m)
                        if abs(yrel) > 0.8:
                            continue
                        # Require reasonable existence probability
                        if exist_prob is not None and exist_prob < 0.6:
                            continue
                        # Require reasonable uncertainty
                        if dx_std is not None and dx_std > 2.0:
                            continue
                        if vx_std is not None and vx_std > 3.0:
                            continue

                    # Reject likely ghosts right on top of ego when relative speed is tiny
                    if dx < 7.0 and abs(vrel) < 0.3:
                        continue

                    # Reject tracks with suspiciously high velocities that are likely ghosts
                    # Many radar tracks show vRel=5+ m/s when vision sees stationary objects
                    # Filter out tracks with |vRel| > 3 m/s unless they're far away (>20m)
                    # This helps reduce false positives that prevent vision matching
                    if abs(vrel) > 3.0 and dx < 20.0:
                        continue

                    # Additional lateral velocity check - reject objects with excessive lateral movement
                    if vy_rel is not None and abs(vy_rel) > 5.0:
                        continue  # Object moving too fast laterally, likely not a lead vehicle

                    # Use stable track ID based on address and object index in frame
                    # This ensures same object gets same trackId across frames
                    obj_idx = 0 if pref == prefixes[0] else 1
                    track_id = (addr << 8) | obj_idx

                    # If object ID is available and valid, prefer it for better tracking
                    # But combine with address to ensure uniqueness across frames
                    if obj_id is not None and isinstance(obj_id, (int, float)) and 0 <= obj_id < 256:
                        # Use object ID but make it unique per address to avoid collisions
                        track_id = (addr << 8) | (int(obj_id) & 0xFF)

                    if track_id not in self.pts:
                        self.pts[track_id] = structs.RadarData.RadarPoint()
                        self.pts[track_id].trackId = track_id

                    self.pts[track_id].measured = True
                    self.pts[track_id].dRel = dx
                    self.pts[track_id].yRel = yrel
                    self.pts[track_id].vRel = vrel
            except Exception as e:
                cloudlog.debug(f"VinFast Radar: Error processing OD message {addr}: {e}")

        # Debug: log closest point occasionally to spot ghosts
        if self.pts and (len(self.pts) % 50 == 0):
            closest = min(self.pts.values(), key=lambda p: p.dRel)
            cloudlog.debug(f"VinFast Radar: Closest object dRel={closest.dRel:.1f} vRel={closest.vRel:.2f}")

        # Clean up old points that are no longer present
        # Keep points for a few cycles in case of temporary message loss
        # This is handled by the base class, but we can add additional cleanup here if needed
