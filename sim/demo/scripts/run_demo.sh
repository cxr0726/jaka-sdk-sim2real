#!/usr/bin/env bash
set -euo pipefail

demo_root="$(realpath "$(cd "$(dirname "$0")/.." && pwd)")"
sdk_root="${KPI_MUJOCO_SIM_SDK_ROOT:-$(realpath "${demo_root}/../sdk")}"
build_dir="${KPI_MUJOCO_DEMO_BUILD_DIR:-${demo_root}/build}"
build_type="${KPI_MUJOCO_DEMO_BUILD_TYPE:-Release}"
ros_setup="${KPI_MUJOCO_ROS_SETUP:-/opt/ros/humble/setup.bash}"
config_python="${KPI_MUJOCO_CONFIG_PYTHON:-/usr/bin/python3}"

sdk_lib="${sdk_root}/lib/release/libkpi_mujoco_sim.so"
xml_path="${KPI_MUJOCO_XML:-${sdk_root}/assets/khan_mini/Khan_mini_contact.xml}"
overlay_config="${KPI_MUJOCO_SIM_CONFIG:-${demo_root}/config/KpiConfig.sim.yaml}"
base_config="${KPI_MUJOCO_BASE_CONFIG:-${demo_root}/config/KpiConfig.yaml}"
run_config="${KPI_MUJOCO_CONFIG:-${build_dir}/KpiConfig.demo.yaml}"
default_log="${KPI_MUJOCO_LOG:-${build_dir}/sim_feedback.csv}"

# Resolve an explicit model before validation and logging. Custom XMLs may use
# their own compiler meshdir; MuJoCo validates their referenced assets.
requested_args=("$@")
for ((i=0; i<${#requested_args[@]}; ++i)); do
  if [[ "${requested_args[i]}" == "--xml" ]]; then
    if (( i+1 >= ${#requested_args[@]} )); then
      echo "--xml requires a path" >&2; exit 2
    fi
    xml_path="${requested_args[i+1]}"
  fi
done

if [[ -f "$ros_setup" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$ros_setup"
  set -u
fi

for required in \
  "$sdk_lib" \
  "${sdk_root}/third_party/mujoco/lib/libmujoco.so" \
  "${sdk_root}/third_party/glfw/lib/libglfw.so" \
  "$xml_path" \
  "$overlay_config" \
  "$base_config" \
  "${demo_root}/ext_libs/onnxruntime-linux-x64-1.24.4/lib/libonnxruntime.so"; do
  if [[ ! -e "$required" ]]; then
    echo "[kpi_mujoco_sim_demo] error: required file or directory not found: $required" >&2
    exit 1
  fi
done

mkdir -p "$build_dir" "${demo_root}/log"

"$config_python" - "$base_config" "$overlay_config" "$run_config" <<'PY'
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
  -DCMAKE_BUILD_TYPE="$build_type" \
  -DPython3_EXECUTABLE="$config_python" \
  -DKPI_MUJOCO_SIM_SDK_ROOT="$sdk_root"

cmake --build "$build_dir" --target KpiMujocoSimDemo -j"$(nproc)"

args=("$@")
has_config=0
has_xml=0
has_log=0
has_mode=0
has_duration=0
for arg in "${args[@]}"; do
  [[ "$arg" == "--config" ]] && has_config=1
  [[ "$arg" == "--xml" ]] && has_xml=1
  [[ "$arg" == "--log" ]] && has_log=1
  [[ "$arg" == "--mode" ]] && has_mode=1
  [[ "$arg" == "--duration" ]] && has_duration=1
done

run_args=()
if [[ "$has_config" -eq 0 ]]; then
  run_args+=(--config "$run_config")
fi
if [[ "$has_xml" -eq 0 ]]; then
  run_args+=(--xml "$xml_path")
fi
if [[ "$has_log" -eq 0 ]]; then
  run_args+=(--log "$default_log")
fi
if [[ "$has_mode" -eq 0 ]]; then
  run_args+=(--mode loco)
fi
if [[ "$has_duration" -eq 0 ]]; then
  run_args+=(--duration 5)
fi
run_args+=("${args[@]}")

export LD_LIBRARY_PATH="${sdk_root}/lib/release:${sdk_root}/third_party/mujoco/lib:${sdk_root}/third_party/glfw/lib:${demo_root}/ext_libs/onnxruntime-linux-x64-1.24.4/lib:/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"

echo "[kpi_mujoco_sim_demo] SDK_ROOT=${sdk_root}"
echo "[kpi_mujoco_sim_demo] config=${run_config}"
echo "[kpi_mujoco_sim_demo] xml=${xml_path}"
echo "[kpi_mujoco_sim_demo] running: ${build_dir}/KpiMujocoSimDemo ${run_args[*]}"
exec "${build_dir}/KpiMujocoSimDemo" "${run_args[@]}"
