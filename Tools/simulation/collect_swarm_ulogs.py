#!/usr/bin/env python3
"""Collect ULog files from SIH swarm simulation instances.

Gathers .ulg files from per-instance log directories and copies them
into a single output folder with filenames that identify the source instance.
"""

import argparse
import glob
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path


def find_repo_root() -> Path:
    """Walk up from script location to find the repository root."""
    path = Path(__file__).resolve().parent
    while path != path.parent:
        if (path / "ROMFS").is_dir() and (path / "src").is_dir():
            return path
        path = path.parent
    return Path(__file__).resolve().parent.parent.parent


def parse_instance_number(filepath: str) -> int | None:
    """Extract the instance number from a path containing instance_N."""
    parts = Path(filepath).parts
    for part in parts:
        if part.startswith("instance_"):
            try:
                return int(part.split("_", 1)[1])
            except (ValueError, IndexError):
                continue
    return None


def collect_ulogs(build_dir: Path, output_dir: Path, max_instances: int | None) -> None:
    """Find and copy ULog files from swarm instance directories."""
    pattern = str(build_dir / "instance_*" / "log" / "**" / "*.ulg")
    ulog_files = sorted(glob.glob(pattern, recursive=True))

    if not ulog_files:
        print(f"No ULog files found matching: {pattern}")
        print()
        print("Troubleshooting:")
        print(f"  - Verify build directory exists: {build_dir}")
        print(f"  - Check for instance_* subdirectories in {build_dir}")
        print("  - Ensure the swarm simulation has run and produced logs")
        sys.exit(1)

    # Filter by instance count if specified.
    if max_instances is not None:
        ulog_files = [
            f for f in ulog_files
            if (n := parse_instance_number(f)) is not None and n < max_instances
        ]

    if not ulog_files:
        assert max_instances is not None
        print(f"No ULog files found for the requested instance range (0..{max_instances - 1}).")
        sys.exit(1)

    output_dir.mkdir(parents=True, exist_ok=True)

    total_size = 0
    instances_seen: set[int] = set()
    copied = 0

    for src in ulog_files:
        instance = parse_instance_number(src)
        if instance is None:
            print(f"  skipping (could not determine instance): {src}")
            continue

        instances_seen.add(instance)
        original_name = Path(src).name
        dest_name = f"drone_{instance:02d}_{original_name}"
        dest_path = output_dir / dest_name

        shutil.copy2(src, dest_path)
        file_size = os.path.getsize(src)
        total_size += file_size
        copied += 1
        print(f"  [{instance:02d}] {original_name} ({file_size / 1024:.1f} KB)")

    print()
    print("Summary")
    print("-" * 40)
    print(f"  Files collected : {copied}")
    print(f"  Instances       : {sorted(instances_seen)}")
    print(f"  Total size      : {total_size / (1024 * 1024):.2f} MB")
    print(f"  Output directory: {output_dir.resolve()}")


def main() -> None:
    repo_root = find_repo_root()
    default_build = str(repo_root / "build" / "px4_sitl_sih")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    parser = argparse.ArgumentParser(
        description="Collect ULog files from SIH swarm simulation instances.",
    )
    parser.add_argument(
        "--build-dir",
        default=default_build,
        help=f"Path to the build directory (default: {default_build})",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Output directory (default: swarm_ulogs_YYYYMMDD_HHMMSS)",
    )
    parser.add_argument(
        "--instances",
        type=int,
        default=None,
        help="Number of instances to collect from (default: auto-detect)",
    )

    args = parser.parse_args()

    build_dir = Path(args.build_dir)
    if not build_dir.is_dir():
        print(f"Error: build directory does not exist: {build_dir}")
        sys.exit(1)

    if args.output_dir is not None:
        output_dir = Path(args.output_dir)
    else:
        output_dir = Path(f"swarm_ulogs_{timestamp}")

    print(f"Collecting ULog files from: {build_dir}")
    print(f"Output directory: {output_dir}")
    print()

    collect_ulogs(build_dir, output_dir, args.instances)


if __name__ == "__main__":
    main()
