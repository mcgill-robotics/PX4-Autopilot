#!/bin/bash
# Launch N SIH quadrotor instances with XRCE-DDS agent.
# Usage: ./sih_swarm_run.sh [CONFIG_YAML] [SPEED_FACTOR]
#
# Requires: build/px4_sitl_sih (make px4_sitl_sih sihsim_quadx_vision)

set -euo pipefail

ulimit -n 4096

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../"
build_path="${src_path}/build/px4_sitl_sih"

CONFIG_YAML="${1:-${SCRIPT_DIR}/swarm_formation.yaml}"
SPEED_FACTOR="${2:-1}"

if [ ! -f "$CONFIG_YAML" ]; then
    echo "ERROR: config not found: $CONFIG_YAML"
    exit 1
fi

if [ ! -d "$build_path" ]; then
    echo "ERROR: build directory not found: $build_path"
    echo "Run: make px4_sitl_sih"
    exit 1
fi

# --- Parse YAML config via Python ---
eval "$(python3 -c "
import yaml, sys
with open('${CONFIG_YAML}') as f:
    cfg = yaml.safe_load(f)
origin = cfg['origin']
print(f\"ORIGIN_LAT={origin['lat']}\")
print(f\"ORIGIN_LON={origin['lon']}\")
print(f\"ORIGIN_ALT={origin['alt']}\")
target = cfg['target']
print(f\"TARGET_X={target['x']}\")
print(f\"TARGET_Y={target['y']}\")
print(f\"TARGET_Z={target['z']}\")
drones = cfg['formation']
print(f\"DRONE_COUNT={len(drones)}\")
for i, d in enumerate(drones):
    print(f\"DRONE_{i}_ID={d['id']}\")
    print(f\"DRONE_{i}_X={d['x']}\")
    print(f\"DRONE_{i}_Y={d['y']}\")
    print(f\"DRONE_{i}_Z={d['z']}\")
")"

echo "Swarm config: $DRONE_COUNT drones, origin ($ORIGIN_LAT, $ORIGIN_LON, ${ORIGIN_ALT}m)"
echo "Target: ($TARGET_X, $TARGET_Y, $TARGET_Z)"
echo "Speed factor: $SPEED_FACTOR"
echo ""

# --- Cleanup ---
CHILD_PIDS=()
AGENT_PID=""

cleanup() {
    echo ""
    echo "Shutting down..."
    for pid in "${CHILD_PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    if [ -n "$AGENT_PID" ]; then
        kill "$AGENT_PID" 2>/dev/null || true
    fi
    pkill -x px4 2>/dev/null || true
    wait 2>/dev/null
    echo "All processes stopped."
}

trap cleanup SIGINT SIGTERM

# --- Kill existing instances ---
echo "Killing running instances..."
pkill -x px4 2>/dev/null || true
sleep 1

# --- Start XRCE-DDS agent (if available) ---
if command -v MicroXRCEAgent &>/dev/null; then
    echo "Starting MicroXRCEAgent on UDP port 8888..."
    MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
    AGENT_PID=$!
    echo "XRCE-DDS agent PID: $AGENT_PID"
else
    echo "WARNING: MicroXRCEAgent not found on host."
    echo "  Start it separately (e.g. from Docker container):"
    echo "  MicroXRCEAgent udp4 -p 8888"
fi
echo ""

# --- Port map ---
printf "%-10s %-12s %-14s %-14s\n" "Instance" "MAV_SYS_ID" "DDS Namespace" "Offboard Port"
printf "%-10s %-12s %-14s %-14s\n" "--------" "----------" "-------------" "-------------"
n=0
while [ $n -lt "$DRONE_COUNT" ]; do
    sysid=$((n + 1))
    printf "%-10s %-12s %-14s %-14s\n" "$n" "$sysid" "px4_${n}" "$((14540 + n))"
    n=$((n + 1))
done
echo ""

# --- Launch instances ---
export PX4_SIM_MODEL=sihsim_quadx_vision

n=0
while [ $n -lt "$DRONE_COUNT" ]; do
    eval "FX=\$DRONE_${n}_X"
    eval "FY=\$DRONE_${n}_Y"
    eval "FZ=\$DRONE_${n}_Z"

    working_dir="$build_path/instance_$n"
    [ ! -d "$working_dir" ] && mkdir -p "$working_dir"

    export PX4_HOME_LAT="$ORIGIN_LAT"
    export PX4_HOME_LON="$ORIGIN_LON"
    export PX4_HOME_ALT="$ORIGIN_ALT"
    export PX4_FORMATION_X="$FX"
    export PX4_FORMATION_Y="$FY"
    export PX4_FORMATION_Z="$FZ"
    export PX4_TARGET_X="$TARGET_X"
    export PX4_TARGET_Y="$TARGET_Y"
    export PX4_TARGET_Z="$TARGET_Z"
    export PX4_SIM_SPEED_FACTOR="$SPEED_FACTOR"
    # Force consistent DDS namespace for all instances (including 0)
    export PX4_UXRCE_DDS_NS="px4_$n"

    pushd "$working_dir" &>/dev/null
    echo "Starting instance $n (formation: $FX, $FY, $FZ)"
    "$build_path/bin/px4" -i "$n" -d "$build_path/etc" >out.log 2>err.log &
    CHILD_PIDS+=($!)
    popd &>/dev/null

    n=$((n + 1))
done

echo ""
echo "All $DRONE_COUNT instances launched. Press Ctrl-C to stop."
wait
