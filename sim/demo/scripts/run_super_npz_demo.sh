#!/usr/bin/env bash
set -euo pipefail

demo_root="$(realpath "$(cd "$(dirname "$0")/.." && pwd)")"
python="${KPI_SUPER_NPZ_PYTHON:-${demo_root}/.venv-pub-motion/bin/python}"
replay_dir="${KPI_SUPER_NPZ_RUN_DIR:-${demo_root}/build/super_npz_replay}"
ready_timeout="${KPI_SUPER_NPZ_READY_TIMEOUT:-60}"
publisher_startup_sleep="${KPI_SUPER_NPZ_PUBLISHER_STARTUP_SLEEP:-0.25}"
npz="${demo_root}/scripts/newsmallstep.npz"
layout="auto"
height_offset="${KPI_SUPER_NPZ_HEIGHT_OFFSET:-0}"
target_limits="${KPI_SUPER_NPZ_TARGET_LIMITS:-none}"
policy="${KPI_SUPER_NPZ_POLICY:-}"
motion_port="${KPI_SUPER_NPZ_MOTION_PORT:-28721}"
state_port="${KPI_SUPER_NPZ_STATE_PORT:-28731}"
viewer_enabled="${KPI_SUPER_NPZ_VIEWER:-1}"
forward_args=()
if [[ "$#" -gt 0 && "$1" != -* ]]; then
  npz="$1"
  shift
fi
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --viewer) viewer_enabled=1; shift ;;
    --no-viewer) viewer_enabled=0; shift ;;
    --npz-layout|--height-offset|--target-limits|--policy)
      option="$1"
      if [[ "$#" -lt 2 ]]; then echo "$option requires a value" >&2; exit 2; fi
      case "$option" in
        --npz-layout) layout="$2" ;;
        --height-offset) height_offset="$2" ;;
        --target-limits) target_limits="$2" ;;
        --policy) policy="$2" ;;

      esac
      shift 2 ;;
    --mode|--direct-fsm|--config|--anchor|--reference-body-index)
      echo "$1 is managed by this SuperMimic launcher" >&2; exit 2 ;;
    -h|--help)
      echo "Usage: $0 [motion.npz] [--policy model.onnx] [--viewer|--no-viewer] [--npz-layout auto|isaac|jaka] [--height-offset meters] [--target-limits none|real] [C++ options]"
      echo "Starts SuperMimic control first, waits for its valid state, then publishes frame 0 at 50 Hz."
      echo "Defaults: viewer on, source-layout auto, fixed base reference (index 0), local motion/state ports 28721/28731."
      echo "Height offset defaults to 0 for all NPZ layouts; --height-offset translates the publisher's world Z."
      echo "KPI_SUPER_NPZ_VIEWER=0 disables the viewer; --duration limits simulation time."
      echo "Default XML: Khan_mini_contact.xml (CAD contact envelope, whole-foot mesh)."
      echo "Pass --xml path/to/model.xml to select another contact model."
      echo "--policy (or KPI_SUPER_NPZ_POLICY) overrides only this replay's SuperMimic model; otherwise uses KpiConfig.yaml."
      exit 0 ;;
    *) forward_args+=("$1"); shift ;;
  esac
done
set -- "${forward_args[@]}"
npz="$(realpath "$npz")"
if [[ -n "$policy" ]]; then
  if [[ ! -f "$policy" ]]; then
    echo "Policy file not found: $policy" >&2
    exit 1
  fi
  policy="$(realpath "$policy")"
fi
state_connect="tcp://127.0.0.1:${state_port}"
motion_bind="tcp://127.0.0.1:${motion_port}"

if [[ ! -x "$python" ]]; then
  echo "Missing publisher environment: $python" >&2
  exit 1
fi
if [[ ! -f "$npz" ]]; then
  echo "Motion file not found: $npz" >&2
  exit 1
fi
mkdir -p "$replay_dir"
# Protect the generated YAML/logs from a second launcher using this directory.
exec {replay_lock_fd}>"${replay_dir}/.launcher.lock"
if ! flock -n "$replay_lock_fd"; then
  echo "Another replay is using $replay_dir; choose a different KPI_SUPER_NPZ_RUN_DIR" >&2
  exit 1
fi

# Describe validates without opening sockets or consuming frames. Named Khan
# training joints already use JAKA order, unlike the older Isaac NPZ files.
"$python" "${demo_root}/scripts/pub_motion_super_npz.py" "$npz" \
  --layout "$layout" --height-offset="$height_offset" --describe > "${replay_dir}/motion_info.json"

# All replay overrides are local; the remote Pico configuration stays intact.
"$python" - "${demo_root}/config/KpiConfig.yaml" "${replay_dir}/KpiConfig.base.yaml" \
  "${replay_dir}/motion_info.json" "$motion_port" "$state_port" "$target_limits" "$policy" <<'PY'
import json
import sys
from pathlib import Path
import yaml

motion_port, state_port = map(int, sys.argv[4:6])
if not (1 <= motion_port <= 65535 and 1 <= state_port <= 65535 and motion_port != state_port):
    raise SystemExit("Motion/state ports must be distinct integers in 1..65535")
config = yaml.safe_load(Path(sys.argv[1]).read_text())
info = json.loads(Path(sys.argv[3]).read_text())
super_config = config["jakaminimimic_superzmq"]
limit_mode = sys.argv[6]
if limit_mode not in ("none", "real"):
    raise SystemExit("--target-limits must be none or real")
super_config["clip_target_positions"] = limit_mode == "real"
if sys.argv[7]:
    policy = Path(sys.argv[7]).resolve()
    if not policy.is_file():
        raise SystemExit(f"Policy file not found: {policy}")
    super_config["policy_path"] = str(policy)

super_config["zmq_connect"] = f"tcp://127.0.0.1:{motion_port}"
config["state_zmq_bind"] = f"tcp://127.0.0.1:{state_port}"
# The legacy SuperMimic buffer has neither a source-clock configuration nor an
# implicit height transform. The publisher already validated/applied the offset.
super_config.pop("reference_height_offset", None)
super_config.pop("clock", None)
Path(sys.argv[2]).write_text(yaml.safe_dump(config, sort_keys=False))
print(f"[super_npz_demo] layout={info['layout']} frames={info['frames']} fps={info['fps']} "
      f"reference=base_link[0] publisher_height_offset={info['reference_height_offset']:g}m target_limits={limit_mode} "
      f"policy={super_config['policy_path']}")
PY

publisher_pid=""
sim_pid=""
cleanup() {
  if [[ -n "$publisher_pid" ]] && kill -0 "$publisher_pid" 2>/dev/null; then
    kill "$publisher_pid" 2>/dev/null || true
    wait "$publisher_pid" 2>/dev/null || true
  fi
  if [[ -n "$sim_pid" ]]; then
    # run_demo.sh may still be compiling.  It runs in its own session so this
    # also stops its build children, without touching unrelated simulators.
    kill -TERM -- "-${sim_pid}" 2>/dev/null || true
    wait "$sim_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "[super_npz_demo] motion=$npz"
echo "[super_npz_demo] starting controller first; waiting for valid state on ${state_connect}"
echo "[super_npz_demo] simulator log: ${replay_dir}/sim.log"
: > "${replay_dir}/sim.log"
: > "${replay_dir}/publisher.log"

viewer_args=()
if [[ "$viewer_enabled" != "0" ]]; then
  viewer_args+=(--viewer)
fi

# Do not consume motion frames while CMake builds or INIT enters the ZMQ FSM.
# stdbuf makes the readiness marker visible even when stdout is piped to tee.
setsid env \
  KPI_MUJOCO_BASE_CONFIG="${replay_dir}/KpiConfig.base.yaml" \
  KPI_MUJOCO_CONFIG="${replay_dir}/KpiConfig.demo.yaml" \
  KPI_MUJOCO_LOG="${replay_dir}/sim_feedback.csv" \
  stdbuf -oL -eL "${demo_root}/scripts/run_demo.sh" \
  "${viewer_args[@]}" --realtime --duration 0 --mode jakaminimimic_superzmq "$@" \
  > >(tee "${replay_dir}/sim.log") 2>&1 &
sim_pid=$!

# Require this SuperMimic process to enter its FSM and emit its own bind
# marker as well as a complete finite state, avoiding an unrelated old publisher.
"$python" - "$sim_pid" "${replay_dir}/sim.log" "$state_connect" "$ready_timeout" <<'PY'
import json
import math
import os
import sys
import time
from pathlib import Path

import zmq

sim_pid = int(sys.argv[1])
log_path = Path(sys.argv[2])
timeout = float(sys.argv[4])
if not math.isfinite(timeout) or timeout <= 0:
    raise SystemExit("KPI_SUPER_NPZ_READY_TIMEOUT must be a positive finite number")
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.LINGER, 0)
socket.setsockopt(zmq.SUBSCRIBE, b"")
socket.connect(sys.argv[3])
deadline = time.monotonic() + timeout
try:
    while time.monotonic() < deadline:
        try:
            os.kill(sim_pid, 0)
        except ProcessLookupError:
            raise SystemExit(f"Controller exited before readiness; see {log_path}")
        if not socket.poll(100):
            continue
        try:
            state = json.loads(socket.recv())
            q = state["joint_pos"]
            quat = state["body_quat_w"][0]
            valid = (
                isinstance(q, list) and len(q) == 27
                and isinstance(quat, list) and len(quat) == 4
                and all(math.isfinite(float(x)) for x in q + quat)
                and sum(float(x) ** 2 for x in quat) > 0.5
            )
        except (ValueError, TypeError, KeyError, IndexError):
            continue
        log = log_path.read_text(errors="replace")
        bound = "[StatePublisher] bound to " in log
        entered = "[FSMJakaMiniSuperZmq] action_dim=27" in log
        if valid and bound and entered:
            os.kill(sim_pid, 0)
            print(f"[super_npz_demo] controller ready: valid ZMQ state seq={state.get('seq')}", flush=True)
            break
    else:
        raise SystemExit(f"Controller readiness timed out after {timeout:g}s; see {log_path}")
finally:
    socket.close(0)
    context.term()
PY

if ! kill -0 "$sim_pid" 2>/dev/null; then
  echo "[super_npz_demo] controller exited before motion publication" >&2
  exit 1
fi

"$python" -u "${demo_root}/scripts/pub_motion_super_npz.py" "$npz" \
  --layout "$layout" --height-offset="$height_offset" --bind "$motion_bind" --hz 50 \
  --startup-sleep "$publisher_startup_sleep" \
  > "${replay_dir}/publisher.log" 2>&1 &
publisher_pid=$!

echo "[super_npz_demo] publisher_pid=$publisher_pid, endpoint=${motion_bind}"
echo "[super_npz_demo] publisher log: ${replay_dir}/publisher.log"
echo "[super_npz_demo] publishing from frame 0 after ${publisher_startup_sleep}s connection warm-up; holds last frame at end"

# The publisher keeps the final reference alive until the controller exits.
# Publisher failures remain fatal rather than silently leaving an empty stream.
while kill -0 "$sim_pid" 2>/dev/null; do
  if [[ -n "$publisher_pid" ]] && ! kill -0 "$publisher_pid" 2>/dev/null; then
    publisher_status=0
    wait "$publisher_pid" || publisher_status=$?
    publisher_pid=""
    echo "[super_npz_demo] publisher exited unexpectedly (exit ${publisher_status}); see ${replay_dir}/publisher.log" >&2
    if [[ "$publisher_status" -eq 0 ]]; then publisher_status=1; fi
    exit "$publisher_status"
  fi
  sleep 0.1
done

sim_status=0
wait "$sim_pid" || sim_status=$?
exit "$sim_status"
