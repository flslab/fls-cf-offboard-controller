#!/usr/bin/env bash
# Local container simulation: loopback UDP only, no host network or USB devices.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
output=${1:?Pass a NEW absolute output directory}
shift
# Remaining scenario options pass through verbatim, including --minimum-surface-gap
# in metres (CAD close-pass mode only; its default remains 0.03 m).
[[ "$output" = /* && ! -e "$output" ]] || exit 2
crazysim=${COLLISION_SIM_CRAZYSIM:-"$(dirname "$repo")/CrazySim"}
assets=${COLLISION_SIM_ASSETS:-/Users/shuqinzhu/Desktop/fls_drone}
firmware=${COLLISION_SIM_FIRMWARE:-"$repo/autoresearch/classic-260925-0959/firmware"}
sim_image=${COLLISION_SIM_IMAGE:-fls-master-sitl-debug:local}
container_name="virtual-collision-$(date +%s)-$$"
mkdir -p "$output"
trap 'docker rm -f "$container_name" >/dev/null 2>&1 || true' EXIT
if ! docker image inspect "$sim_image" --format '{{.Id}}' > "$output/docker_image.txt" 2> "$output/docker_image_lookup.log"; then
  # Docker Desktop can briefly report a missing local tag while warming up.
  sleep .5
  docker image inspect "$sim_image" --format '{{.Id}}' > "$output/docker_image.txt" 2>> "$output/docker_image_lookup.log"
fi
docker run --rm --name "$container_name" \
  -v "$crazysim:/workspace:ro" -v "$assets:/fls-assets:ro" \
  -v "$repo:/offboard:ro" -v "$firmware:/master-sitl:ro" \
  -v "$output:/artifacts" -w /offboard \
  "$sim_image" python -B -m Interaction.simulate_virtual_collision_gazebo \
  --output /artifacts/gazebo --collision "$@" > "$output/runner.log" 2>&1
cat "$output/gazebo/report.json"
