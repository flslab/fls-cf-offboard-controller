#!/usr/bin/env bash
# Simulation only: reduced integration plus full FLS170 firmware/PID/KF capture.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
output=${1:?Pass a NEW absolute output directory}
[[ "$output" = /* && ! -e "$output" ]] || exit 2
mkdir -p "$output"
python_bin=${IMU_SIM_PYTHON:-"$repo/venv/bin/python"}
crazysim=${IMU_SIM_CRAZYSIM:-"$(dirname "$repo")/CrazySim"}
assets=${IMU_SIM_ASSETS:-/Users/shuqinzhu/Desktop/fls_drone}
firmware="$repo/autoresearch/classic-260925-0959/firmware"
container="imu-validation-$(date +%s)-$$"
trap 'docker rm -f "$container" >/dev/null 2>&1 || true' EXIT
cd "$repo"
"$python_bin" -B -m Interaction.simulate_estimator_imu --output "$output/reduced"
mkdir "$output/gazebo"
docker image inspect fls-master-sitl-debug:local --format '{{.Id}}' > "$output/docker_image.txt"
docker run --rm --name "$container" \
  -v "$crazysim:/workspace:ro" -v "$assets:/fls-assets:ro" \
  -v "$repo:/offboard:ro" -v "$firmware:/master-sitl:ro" \
  -v "$output/gazebo:/artifacts" -w /offboard \
  fls-master-sitl-debug:local python -B -m Interaction.simulate_estimator_imu_gazebo \
  > "$output/gazebo_runner.log" 2>&1
printf 'Simulation artifacts: %s\n' "$output"
