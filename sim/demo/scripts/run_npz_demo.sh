#!/usr/bin/env bash
set -euo pipefail

demo_root="$(realpath "$(cd "$(dirname "$0")/.." && pwd)")"
python="${KPI_NPZ_PYTHON:-${demo_root}/.venv-pub-motion/bin/python}"
replay_dir="${KPI_NPZ_RUN_DIR:-${demo_root}/build/npz_replay}"
ready_timeout="${KPI_NPZ_READY_TIMEOUT:-60}"
state_connect="${KPI_NPZ_STATE_CONNECT:-tcp://127.0.0.1:28711}"
publisher_startup_sleep="${KPI_NPZ_PUBLISHER_STARTUP_SLEEP:-0.25}"
npz="${1:-${demo_root}/scripts/newsmallstep.npz}"
if [[ "$#" -gt 0 ]]; then
  shift
fi
npz="$(realpath "$npz")"

if [[ ! -x "$python" ]]; then
  echo "Missing publisher environment: $python" >&2
  exit 1
fi
if [[ ! -f "$npz" ]]; then
  echo "Motion file not found: $npz" >&2
  exit 1
fi
mkdir -p "$replay_dir"

# Keep the remote Pico configuration intact; use a separate local replay config.
"$python" - "${demo_root}/config/KpiConfig.yaml" "${replay_dir}/KpiConfig.base.yaml" <<'PY'
import sys
from pathlib import Path

import yaml

config = yaml.safe_load(Path(sys.argv[1]).read_text())
config["jakaminimimic_zmq"]["zmq_connect"] = "tcp://127.0.0.1:28701"
Path(sys.argv[2]).write_text(yaml.safe_dump(config, sort_keys=False))
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

echo "[npz_demo] motion=$npz"
echo "[npz_demo] starting controller first; waiting for valid state on ${state_connect}"
echo "[npz_demo] simulator log: ${replay_dir}/sim.log"
: > "${replay_dir}/sim.log"
: > "${replay_dir}/publisher.log"

viewer_args=()
if [[ "${KPI_NPZ_VIEWER:-1}" != "0" ]]; then
  viewer_args+=(--viewer)
fi

# Do not consume motion frames while CMake builds or INIT enters the ZMQ FSM.
# stdbuf makes the readiness marker visible even when stdout is piped to tee.
setsid env \
  KPI_MUJOCO_BASE_CONFIG="${replay_dir}/KpiConfig.base.yaml" \
  KPI_MUJOCO_CONFIG="${replay_dir}/KpiConfig.demo.yaml" \
  KPI_MUJOCO_LOG="${replay_dir}/sim_feedback.csv" \
  stdbuf -oL -eL "${demo_root}/scripts/run_demo.sh" \
  "${viewer_args[@]}" --realtime --duration 0 --mode jakaminimimic_zmq "$@" \
  > >(tee "${replay_dir}/sim.log") 2>&1 &
sim_pid=$!

# StatePublisher emits only from FSM_JAKAMINIMIMIC_ZMQ.  Require this run's bind
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
    raise SystemExit("KPI_NPZ_READY_TIMEOUT must be a positive finite number")
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
        bound = "[StatePublisher] bound to " in log_path.read_text(errors="replace")
        if valid and bound:
            os.kill(sim_pid, 0)
            print(f"[npz_demo] controller ready: valid ZMQ state seq={state.get('seq')}", flush=True)
            break
    else:
        raise SystemExit(f"Controller readiness timed out after {timeout:g}s; see {log_path}")
finally:
    socket.close(0)
    context.term()
PY

if ! kill -0 "$sim_pid" 2>/dev/null; then
  echo "[npz_demo] controller exited before motion publication" >&2
  exit 1
fi

"$python" -u "${demo_root}/scripts/pub_motion_npz.py" "$npz" \
  --bind tcp://127.0.0.1:28701 --hz 50 --keep-upper-body \
  --startup-sleep "$publisher_startup_sleep" \
  > "${replay_dir}/publisher.log" 2>&1 &
publisher_pid=$!

echo "[npz_demo] publisher_pid=$publisher_pid, endpoint=tcp://127.0.0.1:28701"
echo "[npz_demo] publisher log: ${replay_dir}/publisher.log"
echo "[npz_demo] publishing from frame 0 after ${publisher_startup_sleep}s connection warm-up"

# A completed single-pass motion keeps the controller running on its last frame.
# Publisher failures remain fatal rather than silently leaving an empty stream.
while kill -0 "$sim_pid" 2>/dev/null; do
  if [[ -n "$publisher_pid" ]] && ! kill -0 "$publisher_pid" 2>/dev/null; then
    publisher_status=0
    wait "$publisher_pid" || publisher_status=$?
    publisher_pid=""
    if [[ "$publisher_status" -ne 0 ]]; then
      echo "[npz_demo] publisher failed (exit ${publisher_status}); see ${replay_dir}/publisher.log" >&2
      exit "$publisher_status"
    fi
    echo "[npz_demo] motion finished; controller continues holding the last reference until viewer closes or Ctrl+C"
  fi
  sleep 0.1
done

sim_status=0
wait "$sim_pid" || sim_status=$?
exit "$sim_status"
