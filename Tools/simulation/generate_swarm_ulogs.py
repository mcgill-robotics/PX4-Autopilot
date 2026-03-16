#!/usr/bin/env python3
"""Generate synthetic ULog files for testing mavsim-viewer swarm replay.

Creates N drone ULog files with crafted trajectories in valid ULog binary
format, without requiring any real PX4 simulation.
"""

import argparse
import json
import math
import os
import struct


# ---------------------------------------------------------------------------
# ULog binary writer
# ---------------------------------------------------------------------------

ULOG_MAGIC = b'\x55\x4c\x6f\x67\x01\x12\x35'
ULOG_VERSION = 1


class ULogWriter:
    """Write a minimal but valid ULog binary file."""

    def __init__(self, path: str, timestamp_us: int = 0):
        self._path = path
        self._fp = open(path, 'wb')
        self._msg_id_counter = 0
        self._subscriptions: dict[str, int] = {}  # topic_name -> msg_id
        self._write_header(timestamp_us)
        self._write_flag_bits()

    # -- low-level helpers --------------------------------------------------

    def _write_raw(self, data: bytes):
        self._fp.write(data)

    def _write_message(self, msg_type: int, payload: bytes):
        """Write a ULog message: uint16 size + uint8 type + payload."""
        self._write_raw(struct.pack('<HB', len(payload), msg_type))
        self._write_raw(payload)

    def _write_header(self, timestamp_us: int):
        self._write_raw(ULOG_MAGIC)
        self._write_raw(struct.pack('<BQ', ULOG_VERSION, timestamp_us))

    def _write_flag_bits(self):
        # 'B' message: 8 compat + 8 incompat + 3*uint64 appended offsets
        payload = b'\x00' * 8 + b'\x00' * 8 + struct.pack('<QQQ', 0, 0, 0)
        self._write_message(0x42, payload)

    # -- definition messages ------------------------------------------------

    def write_format(self, fmt_string: str):
        """Write a Format ('F') definition message.

        fmt_string example: "vehicle_local_position:uint64_t timestamp;float x;float y"
        """
        payload = fmt_string.encode('ascii')
        self._write_message(0x46, payload)

    def write_info(self, key: str, value: str):
        """Write an Info ('I') message with a string value."""
        key_with_type = f"char[{len(value)}] {key}"
        key_bytes = key_with_type.encode('ascii')
        value_bytes = value.encode('ascii')
        payload = struct.pack('<B', len(key_bytes)) + key_bytes + value_bytes
        self._write_message(0x49, payload)

    def add_subscription(self, topic_name: str, multi_id: int = 0) -> int:
        """Write an AddSubscription ('A') message. Returns the assigned msg_id."""
        msg_id = self._msg_id_counter
        self._msg_id_counter += 1
        self._subscriptions[topic_name] = msg_id
        name_bytes = topic_name.encode('ascii')
        payload = struct.pack('<BH', multi_id, msg_id) + name_bytes
        self._write_message(0x41, payload)
        return msg_id

    def write_data(self, topic_name: str, data_bytes: bytes):
        """Write a Data ('D') message for a subscribed topic.

        data_bytes must be the fully serialized payload (starting with
        uint64 timestamp) matching the format definition.
        """
        msg_id = self._subscriptions[topic_name]
        payload = struct.pack('<H', msg_id) + data_bytes
        self._write_message(0x44, payload)

    def close(self):
        self._fp.close()


# ---------------------------------------------------------------------------
# Topic serialization helpers
# ---------------------------------------------------------------------------

def pack_vehicle_local_position(timestamp_us: int, x: float, y: float, z: float,
                                 vx: float, vy: float, vz: float,
                                 ref_lat: float, ref_lon: float, ref_alt: float) -> bytes:
    return struct.pack('<Qffffffddf',
                       timestamp_us, x, y, z, vx, vy, vz,
                       ref_lat, ref_lon, ref_alt)


def pack_vehicle_attitude(timestamp_us: int, q: tuple[float, ...]) -> bytes:
    return struct.pack('<Qffff', timestamp_us, q[0], q[1], q[2], q[3])


def pack_vehicle_global_position(timestamp_us: int, lat: float, lon: float, alt: float) -> bytes:
    return struct.pack('<Qddf', timestamp_us, lat, lon, alt)


def pack_vehicle_status(timestamp_us: int, vehicle_type: int, nav_state: int) -> bytes:
    return struct.pack('<QBB', timestamp_us, vehicle_type, nav_state)


# ---------------------------------------------------------------------------
# Format definition strings (must match pack functions above)
# ---------------------------------------------------------------------------

FMT_LOCAL_POS = (
    "vehicle_local_position:"
    "uint64_t timestamp;"
    "float x;float y;float z;"
    "float vx;float vy;float vz;"
    "double ref_lat;double ref_lon;float ref_alt"
)

FMT_ATTITUDE = (
    "vehicle_attitude:"
    "uint64_t timestamp;"
    "float[4] q"
)

FMT_GLOBAL_POS = (
    "vehicle_global_position:"
    "uint64_t timestamp;"
    "double lat;double lon;float alt"
)

FMT_STATUS = (
    "vehicle_status:"
    "uint64_t timestamp;"
    "uint8_t vehicle_type;uint8_t nav_state"
)

# ---------------------------------------------------------------------------
# Trajectory math
# ---------------------------------------------------------------------------

REF_LAT = 47.397742
REF_LON = 8.545594
REF_ALT = 488.0

NAV_TAKEOFF = 2
NAV_OFFBOARD = 14
NAV_LAND = 18


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def smooth_interp(t: float) -> float:
    """Cosine interpolation factor 0..1."""
    return 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, t)))


def grid_position(drone_id: int) -> tuple[float, float]:
    """4x4 grid, 3m spacing, centered on origin."""
    row = drone_id // 4
    col = drone_id % 4
    x = (row - 1.5) * 3.0
    y = (col - 1.5) * 3.0
    return x, y


def circle_position(drone_id: int, num_drones: int, radius: float = 30.0) -> tuple[float, float]:
    angle = 2.0 * math.pi * drone_id / num_drones
    return radius * math.cos(angle), radius * math.sin(angle)


def local_to_global(x: float, y: float, z: float) -> tuple[float, float, float]:
    lat = REF_LAT + (x / 111320.0)
    lon = REF_LON + (y / (111320.0 * math.cos(math.radians(REF_LAT))))
    alt = REF_ALT - z
    return lat, lon, alt


# ---------------------------------------------------------------------------
# Phase computations
# ---------------------------------------------------------------------------

TARGET = (200.0, 0.0, 0.0)  # NED target for kamikaze


def compute_state(drone_id: int, t: float, num_drones: int, _duration: float):
    """Return (x, y, z, vx, vy, vz, yaw, nav_state) for a drone at time t.

    Returns None if the drone hasn't started yet or has impacted.
    """
    start_time = drone_id * 0.5  # staggered start
    if t < start_time:
        return None

    # Failure drone 12 -- crash at t=70, log ends
    if drone_id == 12 and t > 70.0:
        return None

    # Default nav state
    nav_state = NAV_OFFBOARD
    yaw = 0.0
    vx, vy, vz = 0.0, 0.0, 0.0

    gx, gy = grid_position(drone_id)
    cx, cy = circle_position(drone_id, num_drones)

    if t < 10.0:
        # Phase 1: staggered start, on ground rising slightly
        frac = smooth_interp((t - start_time) / max(0.01, 10.0 - start_time))
        x, y = gx * frac, gy * frac
        z = 0.0
        nav_state = NAV_TAKEOFF

    elif t < 25.0:
        # Phase 2: grid takeoff, ascend to -20 NED
        frac = smooth_interp((t - 10.0) / 15.0)
        x, y = gx, gy
        z = -20.0 * frac
        vz = -20.0 / 15.0 if frac < 1.0 else 0.0
        nav_state = NAV_TAKEOFF

    elif t < 40.0:
        # Phase 3: grid -> ring morph
        frac = smooth_interp((t - 25.0) / 15.0)
        x = gx + (cx - gx) * frac
        y = gy + (cy - gy) * frac
        z = -20.0
        vx = (cx - gx) / 15.0
        vy = (cy - gy) / 15.0
        nav_state = NAV_OFFBOARD

    elif t < 55.0:
        # Phase 4: ring cruise CW at 3 m/s
        angular_speed = 3.0 / 30.0  # omega = v/r
        base_angle = 2.0 * math.pi * drone_id / num_drones
        angle = base_angle - angular_speed * (t - 40.0)  # CW -> negative
        x = 30.0 * math.cos(angle)
        y = 30.0 * math.sin(angle)
        z = -20.0
        vx = 30.0 * angular_speed * math.sin(angle)
        vy = -30.0 * angular_speed * math.cos(angle)
        yaw = math.atan2(-y, -x)  # face center
        nav_state = NAV_OFFBOARD

    else:
        # Phase 5/6: Kamikaze dive toward target
        # Starting position at t=55
        base_angle = 2.0 * math.pi * drone_id / num_drones
        angular_speed = 3.0 / 30.0
        angle55 = base_angle - angular_speed * 15.0
        start_x = 30.0 * math.cos(angle55)
        start_y = 30.0 * math.sin(angle55)
        start_z = -20.0

        dx = TARGET[0] - start_x
        dy = TARGET[1] - start_y
        dz = TARGET[2] - start_z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        speed = 12.0  # m/s max
        nx, ny, nz = dx / dist, dy / dist, dz / dist

        dt = t - 55.0
        x = start_x + nx * speed * dt
        y = start_y + ny * speed * dt
        z = start_z + nz * speed * dt
        vx = nx * speed
        vy = ny * speed
        vz = nz * speed
        yaw = math.atan2(ny, nx)
        nav_state = NAV_LAND

        # Impact: close to target or above ground
        cur_dist = math.sqrt((TARGET[0] - x) ** 2 + (TARGET[1] - y) ** 2 + (TARGET[2] - z) ** 2)
        if cur_dist < 2.0 or z > -1.0:
            return None

    # Drone 12 failure behavior (attitude wobble from t=60)
    if drone_id == 12 and t >= 60.0:
        wobble_t = t - 60.0
        wobble_amp = min(wobble_t * 0.1, 0.8)  # growing oscillation
        roll = wobble_amp * math.sin(wobble_t * 5.0)
        pitch = wobble_amp * math.cos(wobble_t * 4.0)
        # drift sideways
        x += wobble_t * 2.0
        y += wobble_t * 1.5
        z += wobble_t * 0.5  # descending
        # Build quaternion with roll/pitch/yaw
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        q = (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
        return (x, y, z, vx, vy, vz, q, nav_state)

    return (x, y, z, vx, vy, vz, quat_from_yaw(yaw), nav_state)


# ---------------------------------------------------------------------------
# Generate a single drone ULog
# ---------------------------------------------------------------------------

def generate_drone_ulog(drone_id: int, num_drones: int, duration: float,
                        output_dir: str, scenario: str):
    filename = os.path.join(output_dir, f"drone_{drone_id:02d}.ulg")
    start_us = drone_id * 500_000  # stagger by 0.5s in timestamps

    writer = ULogWriter(filename, timestamp_us=start_us)

    # Write format definitions
    writer.write_format(FMT_LOCAL_POS)
    writer.write_format(FMT_ATTITUDE)
    writer.write_format(FMT_GLOBAL_POS)
    writer.write_format(FMT_STATUS)

    # Write info messages
    writer.write_info("sys_name", "PX4")
    writer.write_info("ver_hw", "SITL")
    writer.write_info("scenario", scenario)
    writer.write_info("drone_id", str(drone_id))

    # Add subscriptions
    writer.add_subscription("vehicle_local_position")
    writer.add_subscription("vehicle_attitude")
    writer.add_subscription("vehicle_global_position")
    writer.add_subscription("vehicle_status")

    # Determine rates per drone (edge cases)
    pos_dt = 0.02  # 50 Hz
    att_dt = 0.01  # 100 Hz
    if drone_id == 13:
        pos_dt = 0.1  # 10 Hz low rate
    if drone_id == 14:
        att_dt = 0.005  # 200 Hz high rate

    # Generate data
    # We iterate at the finest resolution needed and emit when due
    fine_dt = min(pos_dt, att_dt, 0.001)
    # Use the finest rate needed for this drone
    fine_dt = min(pos_dt, att_dt)

    status_interval = 1.0  # 1 Hz
    next_pos_t = 0.0
    next_att_t = 0.0
    next_status_t = 0.0
    last_nav_state = -1

    t = 0.0
    samples_written = 0
    while t <= duration:
        state = compute_state(drone_id, t, num_drones, duration)

        # Drone 15 data gap at t=50-52
        in_gap = (drone_id == 15 and 50.0 <= t <= 52.0)

        if state is not None and not in_gap:
            x, y, z, vx, vy, vz, q, nav_state = state
            ts = int(t * 1_000_000)

            # Position data
            if t >= next_pos_t:
                writer.write_data("vehicle_local_position",
                                  pack_vehicle_local_position(
                                      ts, x, y, z, vx, vy, vz,
                                      REF_LAT, REF_LON, REF_ALT))
                lat, lon, alt = local_to_global(x, y, z)
                writer.write_data("vehicle_global_position",
                                  pack_vehicle_global_position(ts, lat, lon, alt))
                next_pos_t = t + pos_dt
                samples_written += 2

            # Attitude data
            if t >= next_att_t:
                writer.write_data("vehicle_attitude",
                                  pack_vehicle_attitude(ts, q))
                next_att_t = t + att_dt
                samples_written += 1

            # Status data
            if t >= next_status_t or nav_state != last_nav_state:
                writer.write_data("vehicle_status",
                                  pack_vehicle_status(ts, 2, nav_state))
                next_status_t = t + status_interval
                last_nav_state = nav_state
                samples_written += 1

        t += fine_dt

    writer.close()
    return samples_written


# ---------------------------------------------------------------------------
# Scenario metadata
# ---------------------------------------------------------------------------

def write_scenario_json(output_dir: str, scenario: str, num_drones: int, duration: float):
    meta = {
        "scenario": scenario,
        "num_drones": num_drones,
        "duration_s": duration,
        "origin": {
            "lat": REF_LAT,
            "lon": REF_LON,
            "alt": REF_ALT,
        },
        "phases": [
            {"name": "staggered_start", "start_s": 0, "end_s": 10},
            {"name": "grid_takeoff", "start_s": 10, "end_s": 25},
            {"name": "grid_to_ring", "start_s": 25, "end_s": 40},
            {"name": "ring_cruise", "start_s": 40, "end_s": 55},
            {"name": "kamikaze_dive", "start_s": 55, "end_s": 75},
        ],
        "edge_cases": {
            "drone_12": "failure at t=60, attitude oscillation, crash at t=70",
            "drone_13": "low-rate position data (10Hz)",
            "drone_14": "high-rate attitude data (200Hz)",
            "drone_15": "2-second data gap at t=50-52",
        },
        "files": [f"drone_{i:02d}.ulg" for i in range(num_drones)],
    }
    path = os.path.join(output_dir, "scenario.json")
    with open(path, 'w') as f:
        json.dump(meta, f, indent=2)
    print(f"  Wrote {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Generate synthetic ULog files for swarm replay testing.")
    parser.add_argument("--output-dir", default="synthetic_swarm_ulogs",
                        help="Output directory (default: synthetic_swarm_ulogs)")
    parser.add_argument("--num-drones", type=int, default=16,
                        help="Number of drones (default: 16)")
    parser.add_argument("--duration", type=float, default=90.0,
                        help="Scenario duration in seconds (default: 90)")
    parser.add_argument("--scenario", default="hawk_descent",
                        help="Scenario name (default: hawk_descent)")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    print(f"Generating {args.num_drones} drone ULog files for scenario '{args.scenario}'")
    print(f"  Duration: {args.duration}s")
    print(f"  Output:   {os.path.abspath(args.output_dir)}/")
    print()

    for i in range(args.num_drones):
        samples = generate_drone_ulog(i, args.num_drones, args.duration,
                                       args.output_dir, args.scenario)
        tag = ""
        if i == 12:
            tag = " [failure drone]"
        elif i == 13:
            tag = " [low-rate pos]"
        elif i == 14:
            tag = " [high-rate att]"
        elif i == 15:
            tag = " [data gap]"
        print(f"  drone_{i:02d}.ulg  ({samples} messages){tag}")

    write_scenario_json(args.output_dir, args.scenario, args.num_drones, args.duration)
    print()
    print(f"Done. {args.num_drones} ULog files written to {os.path.abspath(args.output_dir)}/")


if __name__ == "__main__":
    main()
