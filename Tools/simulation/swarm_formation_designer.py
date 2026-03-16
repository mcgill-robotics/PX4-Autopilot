#!/usr/bin/env python3
"""
Swarm Formation Designer - Create and preview formation YAML files for PX4 SIH swarm simulation.

Generates swarm_formation.yaml files with preset formation patterns (grid, circle, line,
v, diamond, random) for use with sih_swarm_run.sh and swarm_ros2 coordinator.

Usage examples:
    # 16-drone grid with 2m spacing, preview in terminal
    python3 swarm_formation_designer.py --pattern grid --count 16 --spacing 2.0 --preview

    # 8-drone circle, radius 10m, write to file
    python3 swarm_formation_designer.py --pattern circle --count 8 --radius 10 -o formation.yaml

    # V formation with custom origin and target
    python3 swarm_formation_designer.py --pattern v --count 7 --spacing 3.0 \\
        --origin-lat 37.4 --origin-lon -122.0 --target-north 150 --target-east 50
"""

import argparse
import math
import random
import sys
from typing import List, Tuple

# Defaults matching PX4 SIH test site (Zurich area)
DEFAULT_LAT = 47.397742
DEFAULT_LON = 8.545594
DEFAULT_ALT = 488.0
DEFAULT_ALTITUDE = 20.0
DEFAULT_SPACING = 1.0
DEFAULT_RADIUS = 5.0
DEFAULT_COUNT = 16
DEFAULT_TARGET_NORTH = 200.0
DEFAULT_TARGET_EAST = 0.0
MIN_SEPARATION = 0.5  # meters, minimum allowed distance between any two drones


def generate_grid(count: int, spacing: float, altitude: float) -> List[dict]:
    """Arrange drones in a square grid pattern (ceil(sqrt(count)) columns)."""
    cols = math.ceil(math.sqrt(count))
    positions = []
    for i in range(count):
        row = i // cols
        col = i % cols
        positions.append({
            "id": i,
            "x": round(row * spacing, 2),
            "y": round(col * spacing, 2),
            "z": round(-altitude, 2),
        })
    return positions


def generate_circle(count: int, radius: float, altitude: float) -> List[dict]:
    """Arrange drones evenly spaced around a circle."""
    positions = []
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        positions.append({
            "id": i,
            "x": round(radius * math.cos(angle), 2),
            "y": round(radius * math.sin(angle), 2),
            "z": round(-altitude, 2),
        })
    return positions


def generate_line(count: int, spacing: float, altitude: float) -> List[dict]:
    """Arrange drones in a line along the east (y) axis, centered at origin."""
    positions = []
    offset = (count - 1) * spacing / 2.0
    for i in range(count):
        positions.append({
            "id": i,
            "x": 0.0,
            "y": round(i * spacing - offset, 2),
            "z": round(-altitude, 2),
        })
    return positions


def generate_v(count: int, spacing: float, altitude: float) -> List[dict]:
    """Classic V formation with leader at front (northernmost position).

    Leader is id 0 at the tip. Remaining drones alternate left/right,
    each row stepping back (south) and outward (east/west).
    """
    positions = [{"id": 0, "x": 0.0, "y": 0.0, "z": round(-altitude, 2)}]
    wing_index = 1
    for i in range(1, count):
        side = 1 if (i % 2 == 1) else -1
        row = (i + 1) // 2
        positions.append({
            "id": wing_index,
            "x": round(-row * spacing, 2),       # step back (south)
            "y": round(side * row * spacing, 2),  # spread east/west
            "z": round(-altitude, 2),
        })
        wing_index += 1
    return positions


def generate_diamond(count: int, spacing: float, altitude: float) -> List[dict]:
    """Diamond/rhombus formation.

    Builds concentric diamond rings outward from the center. Ring k has 4*k
    positions (except the center which has 1). Drones fill from center outward.
    """
    # Generate diamond coordinates ring by ring
    coords: List[Tuple[float, float]] = [(0.0, 0.0)]
    ring = 1
    while len(coords) < count:
        # Each ring has 4 sides, each side has 'ring' segments
        for side in range(4):
            for step in range(ring):
                if side == 0:   # top-right edge
                    x = (ring - step) * spacing
                    y = step * spacing
                elif side == 1: # bottom-right edge
                    x = -step * spacing
                    y = (ring - step) * spacing
                elif side == 2: # bottom-left edge
                    x = -(ring - step) * spacing
                    y = -step * spacing
                else:           # top-left edge
                    x = step * spacing
                    y = -(ring - step) * spacing
                coords.append((round(x, 2), round(y, 2)))
                if len(coords) >= count:
                    break
            if len(coords) >= count:
                break
        ring += 1

    positions = []
    for i in range(count):
        positions.append({
            "id": i,
            "x": coords[i][0],
            "y": coords[i][1],
            "z": round(-altitude, 2),
        })
    return positions


def generate_random(count: int, spacing: float, altitude: float) -> List[dict]:
    """Random positions within a bounding box, enforcing minimum separation.

    The bounding box size scales with count and spacing. Uses rejection
    sampling to guarantee minimum separation between all drones.
    """
    box_size = max(spacing * math.sqrt(count) * 2, spacing * 4)
    positions = []
    coords: List[Tuple[float, float]] = []
    max_attempts = count * 1000

    rng = random.Random(42)  # deterministic for reproducibility
    attempts = 0
    while len(coords) < count and attempts < max_attempts:
        x = round(rng.uniform(-box_size / 2, box_size / 2), 2)
        y = round(rng.uniform(-box_size / 2, box_size / 2), 2)
        too_close = False
        for cx, cy in coords:
            if math.sqrt((x - cx) ** 2 + (y - cy) ** 2) < MIN_SEPARATION:
                too_close = True
                break
        if not too_close:
            coords.append((x, y))
        attempts += 1

    if len(coords) < count:
        print(
            f"Warning: could only place {len(coords)}/{count} drones "
            f"with minimum separation {MIN_SEPARATION}m in bounding box {box_size:.1f}m",
            file=sys.stderr,
        )

    for i, (x, y) in enumerate(coords):
        positions.append({
            "id": i,
            "x": x,
            "y": y,
            "z": round(-altitude, 2),
        })
    return positions


PATTERN_GENERATORS = {
    "grid": generate_grid,
    "circle": generate_circle,
    "line": generate_line,
    "v": generate_v,
    "diamond": generate_diamond,
    "random": generate_random,
}


def validate_no_overlap(positions: List[dict]) -> bool:
    """Check that no two drones occupy the same horizontal position (within MIN_SEPARATION)."""
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            dx = positions[i]["x"] - positions[j]["x"]
            dy = positions[i]["y"] - positions[j]["y"]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < MIN_SEPARATION:
                print(
                    f"Error: drones {positions[i]['id']} and {positions[j]['id']} "
                    f"are only {dist:.2f}m apart (minimum: {MIN_SEPARATION}m)",
                    file=sys.stderr,
                )
                return False
    return True


def build_formation_dict(
    positions: List[dict],
    origin_lat: float,
    origin_lon: float,
    origin_alt: float,
    target_north: float,
    target_east: float,
) -> dict:
    """Build the formation dictionary matching swarm_formation.yaml structure."""
    return {
        "origin": {
            "lat": origin_lat,
            "lon": origin_lon,
            "alt": origin_alt,
        },
        "formation": positions,
        "target": {
            "x": target_north,
            "y": target_east,
            "z": 0.0,
        },
    }


def format_yaml(data: dict) -> str:
    """Format the formation data as YAML with flow-style formation entries.

    Produces output matching the established swarm_formation.yaml style with
    inline dicts for formation entries and block style for origin/target.
    """
    lines = [
        "# Formation config for SIH swarm simulation",
        "# Generated by swarm_formation_designer.py",
        "",
        "origin:",
        f"  lat: {data['origin']['lat']}",
        f"  lon: {data['origin']['lon']}",
        f"  alt: {data['origin']['alt']}",
        "",
        "formation:",
    ]

    # Determine padding width for aligned id field
    max_id = max(p["id"] for p in data["formation"])
    id_width = len(str(max_id))

    for pos in data["formation"]:
        id_str = str(pos["id"]).rjust(id_width)
        lines.append(
            f"  - {{id: {id_str}, "
            f"x: {pos['x']}, "
            f"y: {pos['y']}, "
            f"z: {pos['z']}}}"
        )

    lines.extend([
        "",
        "target:",
        f"  x: {data['target']['x']}",
        f"  y: {data['target']['y']}",
        f"  z: {data['target']['z']}",
        "",
    ])
    return "\n".join(lines)


def ascii_preview(positions: List[dict], target_north: float, target_east: float) -> str:
    """Render an ASCII top-down map showing drone positions and target.

    North is up, East is right. Axes are labeled. The target is shown as 'T'
    and drones as their id number (or '*' if id >= 10).
    """
    width = 60
    height = 30

    # Collect all points (drones + target)
    all_x = [p["x"] for p in positions] + [target_north]
    all_y = [p["y"] for p in positions] + [target_east]

    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)

    # Add margin
    range_x = max_x - min_x if max_x != min_x else 1.0
    range_y = max_y - min_y if max_y != min_y else 1.0
    margin = 0.1
    min_x -= range_x * margin
    max_x += range_x * margin
    min_y -= range_y * margin
    max_y += range_y * margin
    range_x = max_x - min_x
    range_y = max_y - min_y

    def to_grid(north: float, east: float) -> Tuple[int, int]:
        col = int((east - min_y) / range_y * (width - 1))
        row = int((max_x - north) / range_x * (height - 1))  # north is up
        col = max(0, min(width - 1, col))
        row = max(0, min(height - 1, row))
        return row, col

    # Initialize grid
    grid = [[" " for _ in range(width)] for _ in range(height)]

    # Place target
    tr, tc = to_grid(target_north, target_east)
    grid[tr][tc] = "T"

    # Place drones (overwrite target if on same cell)
    for pos in positions:
        r, c = to_grid(pos["x"], pos["y"])
        label = str(pos["id"]) if pos["id"] < 10 else "*"
        grid[r][c] = label

    # Build output
    out = []
    out.append(f"  Formation Preview ({len(positions)} drones)")
    out.append(f"  North ^  (x range: {min_x:.1f} to {max_x:.1f} m)")
    out.append("  " + "-" * (width + 2))
    for row in grid:
        out.append("  |" + "".join(row) + "|")
    out.append("  " + "-" * (width + 2))
    out.append(f"  East ->  (y range: {min_y:.1f} to {max_y:.1f} m)")
    out.append("")
    out.append("  Legend: 0-9 = drone id, * = drone id >= 10, T = target")
    return "\n".join(out)


def parse_args(argv: "List[str] | None" = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create and preview swarm formation YAML files for PX4 SIH simulation.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  %(prog)s --pattern grid --count 16 --spacing 2 --preview\n"
            "  %(prog)s --pattern circle --count 8 --radius 10 -o my_formation.yaml\n"
            "  %(prog)s --pattern v --count 7 --spacing 3 --preview -o v_formation.yaml\n"
        ),
    )
    parser.add_argument(
        "--pattern",
        choices=list(PATTERN_GENERATORS.keys()),
        default="grid",
        help="Formation pattern (default: grid)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=DEFAULT_COUNT,
        help=f"Number of drones (default: {DEFAULT_COUNT})",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=DEFAULT_SPACING,
        help=f"Spacing between drones in meters (default: {DEFAULT_SPACING})",
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=DEFAULT_ALTITUDE,
        help=f"Flight altitude in meters AGL (default: {DEFAULT_ALTITUDE})",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=DEFAULT_RADIUS,
        help=f"Radius for circle pattern in meters (default: {DEFAULT_RADIUS})",
    )
    parser.add_argument(
        "--origin-lat",
        type=float,
        default=DEFAULT_LAT,
        help=f"Origin latitude (default: {DEFAULT_LAT})",
    )
    parser.add_argument(
        "--origin-lon",
        type=float,
        default=DEFAULT_LON,
        help=f"Origin longitude (default: {DEFAULT_LON})",
    )
    parser.add_argument(
        "--origin-alt",
        type=float,
        default=DEFAULT_ALT,
        help=f"Origin altitude AMSL in meters (default: {DEFAULT_ALT})",
    )
    parser.add_argument(
        "--target-north",
        type=float,
        default=DEFAULT_TARGET_NORTH,
        help=f"Target position north of origin in meters (default: {DEFAULT_TARGET_NORTH})",
    )
    parser.add_argument(
        "--target-east",
        type=float,
        default=DEFAULT_TARGET_EAST,
        help=f"Target position east of origin in meters (default: {DEFAULT_TARGET_EAST})",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show ASCII preview of the formation",
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        default=None,
        help="Output YAML file path (default: print to stdout)",
    )

    args = parser.parse_args(argv)

    if args.count < 1:
        parser.error("--count must be at least 1")
    if args.spacing <= 0:
        parser.error("--spacing must be positive")
    if args.altitude <= 0:
        parser.error("--altitude must be positive")
    if args.radius <= 0:
        parser.error("--radius must be positive")

    return args


def main(argv: "List[str] | None" = None) -> int:
    args = parse_args(argv)

    # Select generator and dispatch with appropriate arguments
    generator = PATTERN_GENERATORS[args.pattern]
    if args.pattern == "circle":
        positions = generator(args.count, args.radius, args.altitude)
    else:
        positions = generator(args.count, args.spacing, args.altitude)

    # Validate
    if not validate_no_overlap(positions):
        return 1

    # Build output
    data = build_formation_dict(
        positions,
        args.origin_lat,
        args.origin_lon,
        args.origin_alt,
        args.target_north,
        args.target_east,
    )
    yaml_text = format_yaml(data)

    # Preview
    if args.preview:
        print(ascii_preview(positions, args.target_north, args.target_east))
        print()

    # Output
    if args.output:
        with open(args.output, "w") as f:
            f.write(yaml_text)
        print(f"Wrote {len(positions)} drone formation to {args.output}")
    else:
        if not args.preview:
            print(yaml_text, end="")
        else:
            print(yaml_text, end="")

    return 0


if __name__ == "__main__":
    sys.exit(main())
