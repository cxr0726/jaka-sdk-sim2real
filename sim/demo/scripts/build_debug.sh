#!/usr/bin/env bash
set -euo pipefail

demo_root="$(realpath "$(cd "$(dirname "$0")/.." && pwd)")"
workspace_root="$(realpath "${demo_root}/../..")"
sdk_root="${KPI_MUJOCO_SIM_SDK_ROOT:-${workspace_root}/sim/sdk}"
build_dir="${KPI_MUJOCO_DEMO_BUILD_DIR:-${demo_root}/build_debug}"
config_python="${KPI_MUJOCO_CONFIG_PYTHON:-/usr/bin/python3}"
ros_setup="${KPI_MUJOCO_ROS_SETUP:-/opt/ros/humble/setup.bash}"

if [[ -f "$ros_setup" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$ros_setup"
  set -u
fi

mkdir -p "$build_dir" "${demo_root}/log"

"$config_python" - "${demo_root}/config/KpiConfig.yaml" "${demo_root}/config/KpiConfig.sim.yaml" "${build_dir}/KpiConfig.debug.yaml" <<'PY'
import sys
from pathlib import Path

import yaml

base_path = Path(sys.argv[1])
overlay_path = Path(sys.argv[2])
out_path = Path(sys.argv[3])

with base_path.open("r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f)
with overlay_path.open("r", encoding="utf-8") as f:
    overlay = yaml.safe_load(f) or {}

key_map = {
    "kps": "kp",
    "kds": "kd",
    "kps_pr": "kp_pitch_roll",
    "kds_pr": "kd_pitch_roll",
}
allowed_keys = {
    "kp",
    "kd",
    "kp_pitch_roll",
    "kd_pitch_roll",
    "init_base_pos",
    "init_base_quat",
}

for src_key, value in overlay.items():
    dst_key = key_map.get(src_key, src_key)
    if dst_key in allowed_keys:
        cfg[dst_key] = value

out_path.parent.mkdir(parents=True, exist_ok=True)
with out_path.open("w", encoding="utf-8") as f:
    yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=False)
PY

cmake -S "$demo_root" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DPython3_EXECUTABLE="$config_python" \
  -DKPI_MUJOCO_SIM_SDK_ROOT="$sdk_root"

cmake --build "$build_dir" --target KpiMujocoSimDemo -j"$(nproc)"
