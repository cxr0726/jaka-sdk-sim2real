#!/usr/bin/env bash
set -euo pipefail

demo_root="$(realpath "$(cd "$(dirname "$0")/.." && pwd)")"
python="${KPI_RECORD_PYTHON:-${demo_root}/.venv-pub-motion/bin/python}"
replay_dir="${KPI_RECORD_RUN_DIR:-${demo_root}/build/record_replay}"
ready_timeout="${KPI_RECORD_READY_TIMEOUT:-60}"
state_connect="${KPI_RECORD_STATE_CONNECT:-tcp://127.0.0.1:28711}"
publisher_startup_sleep="${KPI_RECORD_PUBLISHER_STARTUP_SLEEP:-0.25}"
anchor="${KPI_RECORD_ANCHOR:-waist}"
forward_args=()
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --anchor)
      if [[ "$#" -lt 2 ]]; then
        echo "--anchor requires waist or base" >&2
        exit 2
      fi
      anchor="$2"
      shift 2
      ;;
    --anchor=*)
      anchor="${1#*=}"
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [episode directory or Parquet/NPZ file] [--anchor waist|base] [C++ options]"
      echo "Default anchor: waist. Base mode copies base_link pose to all 28 body slots."
      exit 0
      ;;
    *)
      forward_args+=("$1")
      shift
      ;;
  esac
done
case "$anchor" in
  waist|base) ;;
  *) echo "Invalid anchor '$anchor'; expected waist or base" >&2; exit 2 ;;
esac
set -- "${forward_args[@]}"
if [[ ! -x "$python" ]]; then
  echo "Missing record publisher environment: $python" >&2
  exit 1
fi
# Resolve the first available episode; this recording starts at episode_0001.
record="${demo_root}/scripts/20261022-221234"
if [[ "$#" -gt 0 && "$1" != -* ]]; then
  record="$1"
  shift
fi
if [[ -d "$record" ]]; then
  if [[ -f "${record}/data.parquet" ]]; then
    record="${record}/data.parquet"
  else
    record="$("$python" - "$record" <<'PY_SELECT'
from pathlib import Path
import sys
files = sorted(Path(sys.argv[1]).rglob("data.parquet"))
if not files:
    raise SystemExit("No data.parquet episodes found in the selected directory")
print(files[0])
PY_SELECT
)"
  fi
fi
record="$(realpath "$record")"

if [[ ! -f "$record" ]]; then
  echo "Motion file not found: $record" >&2
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

echo "[record_demo] record=$record"
echo "[record_demo] anchor=$anchor (copied to all 28 body slots)"
echo "[record_demo] starting controller first; waiting for valid state on ${state_connect}"
echo "[record_demo] simulator log: ${replay_dir}/sim.log"
: > "${replay_dir}/sim.log"
: > "${replay_dir}/publisher.log"

viewer_args=()
if [[ "${KPI_RECORD_VIEWER:-1}" != "0" ]]; then
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
    raise SystemExit("KPI_RECORD_READY_TIMEOUT must be a positive finite number")
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
            print(f"[record_demo] controller ready: valid ZMQ state seq={state.get('seq')}", flush=True)
            break
    else:
        raise SystemExit(f"Controller readiness timed out after {timeout:g}s; see {log_path}")
finally:
    socket.close(0)
    context.term()
PY

if ! kill -0 "$sim_pid" 2>/dev/null; then
  echo "[record_demo] controller exited before motion publication" >&2
  exit 1
fi

publisher_args=(--bind tcp://127.0.0.1:28701 --anchor "$anchor" --hz "${KPI_RECORD_HZ:-50}" --speed "${KPI_RECORD_SPEED:-1}" --startup-sleep "$publisher_startup_sleep")
if [[ "${KPI_RECORD_LOOP:-0}" == "1" ]]; then
  publisher_args+=(--loop)
fi
"$python" -u "${demo_root}/scripts/pub_motion_record.py" "$record" \
  "${publisher_args[@]}" \
  > "${replay_dir}/publisher.log" 2>&1 &
publisher_pid=$!

echo "[record_demo] publisher_pid=$publisher_pid, endpoint=tcp://127.0.0.1:28701"
echo "[record_demo] publisher log: ${replay_dir}/publisher.log"
echo "[record_demo] publishing from frame 0 after ${publisher_startup_sleep}s connection warm-up"

# A completed single-pass motion keeps the controller running on its last frame.
# Publisher failures remain fatal rather than silently leaving an empty stream.
while kill -0 "$sim_pid" 2>/dev/null; do
  if [[ -n "$publisher_pid" ]] && ! kill -0 "$publisher_pid" 2>/dev/null; then
    publisher_status=0
    wait "$publisher_pid" || publisher_status=$?
    publisher_pid=""
    if [[ "$publisher_status" -ne 0 ]]; then
      echo "[record_demo] publisher failed (exit ${publisher_status}); see ${replay_dir}/publisher.log" >&2
      exit "$publisher_status"
    fi
    echo "[record_demo] motion finished; controller continues holding the last reference until viewer closes or Ctrl+C"
  fi
  sleep 0.1
done

sim_status=0
wait "$sim_pid" || sim_status=$?
exit "$sim_status"
