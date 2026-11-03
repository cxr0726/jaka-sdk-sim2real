#!/usr/bin/env python3
"""
Replay a motion NPZ file over ZMQ PUB to drive FSM_JAKAMINIMIMIC_ZMQ.

Usage:
  python scripts/pub_motion_npz.py config/mimic_command/motion_mf.npz
  python scripts/pub_motion_npz.py motion.npz --bind tcp://*:28701 --hz 50 --loop
"""

from __future__ import annotations

import argparse
import json
import time
import uuid
from pathlib import Path

import numpy as np
import zmq

# Reorder from Isaac Lab joint order to MuJoCo joint order.
# Inverse of kMujocoToIsaacIndex in RealtimeMotionBuffer.cpp:258-261.
ISAAC_TO_MUJOCO = np.array(
    [0, 3, 8, 13, 17, 21, 1, 4, 9, 14, 18, 22, 2, 5, 10, 15, 19, 23, 25, 7, 12, 16, 20, 24, 26, 6, 11],
    dtype=np.intp,
)


# Default upper-body joint positions (Isaac Lab order)
# Upper body joints (15 joints): waist_yaw, arms (12), neck (2).
# Lower body (12 leg joints: 0, 1, 3, 4, 8, 9, 13, 14, 17, 18, 21, 22) retain motion data.
DEFAULT_UPPER_BODY_QPOS_27 = {
    2: 0.0,     # waist_yaw_joint
    5: 0.0,     # Left_shoulder_pitch_joint
    6: 0.0,     # Neck_yaw_joint
    7: 0.0,     # Right_shoulder_pitch_joint
    10: -1.57,  # Left_shoulder_roll_joint
    11: 0.0,    # Neck_pitch_joint
    12: -1.57,  # Right_shoulder_roll_joint
    15: 0.0,    # Left_shoulder_yaw_joint
    16: 0.0,    # Right_shoulder_yaw_joint
    19: 1.57,   # Left_elbow_joint
    20: 1.57,   # Right_elbow_joint
    23: 0.0,    # Left_wrist_roll_joint
    24: 0.0,    # Right_wrist_roll_joint
    25: 0.3,    # Left_wrist_yaw_joint
    26: 0.3,    # Right_wrist_yaw_joint
}

DEFAULT_UPPER_BODY_QPOS_23 = {
    # 2: 0.0,     # waist_yaw_joint
    # 5: 0.0,     # Left_shoulder_pitch_joint
    # 6: 0.0,     # Right_shoulder_pitch_joint
    # 9: -1.57,   # Left_shoulder_roll_joint
    # 10: -1.57,  # Right_shoulder_roll_joint
    # 13: 0.0,    # Left_shoulder_yaw_joint
    # 14: 0.0,    # Right_shoulder_yaw_joint
    17: 1.57,   # Left_elbow_joint
    18: 1.57,   # Right_elbow_joint
    21: 0.0,    # Left_wrist_roll_joint
    22: 0.0,    # Right_wrist_roll_joint
}


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Replay NPZ motion via ZMQ PUB")
    p.add_argument("npz", type=Path, help="Path to .npz motion file")
    p.add_argument("--bind", default="tcp://*:28701", help="ZMQ PUB bind address")
    p.add_argument("--hz", type=float, default=0, help="Publish rate (0 = use NPZ fps, fallback 50)")
    p.add_argument("--loop", action="store_true", help="Loop the motion indefinitely")
    p.add_argument("--startup-sleep", type=float, default=1.0, help="Sleep before first publish")
    p.add_argument(
        "--keep-upper-body",
        action="store_true",
        help="Keep raw upper body joint pos from NPZ instead of overriding with default qpos",
    )
    return p


def load_motion(npz_path: Path) -> dict:
    data = np.load(npz_path)
    required = {"joint_pos", "body_pos_w", "body_quat_w"}
    missing = required - set(data.keys())
    if missing:
        raise KeyError(f"NPZ missing keys: {missing}")
    fps = int(data.get("fps", np.int64(50)).flat[0])
    return {
        "fps": fps,
        "num_frames": data["joint_pos"].shape[0],
        "joint_pos": data["joint_pos"].astype(np.float32),
        "body_pos_w": data["body_pos_w"].astype(np.float32),
        "body_quat_w": data["body_quat_w"].astype(np.float32),
    }


def frame_to_json(
    frame_idx: int,
    motion: dict,
    override_upper_body: bool = True,
    seq: int | None = None,
    stream_id: str | None = None,
) -> str:
    joint_pos = motion["joint_pos"][frame_idx].copy()
    if override_upper_body:
        if len(joint_pos) == 27:
            for idx, val in DEFAULT_UPPER_BODY_QPOS_27.items():
                joint_pos[idx] = val
        elif len(joint_pos) == 23:
            for idx, val in DEFAULT_UPPER_BODY_QPOS_23.items():
                joint_pos[idx] = val

    body_pos_w = motion["body_pos_w"][frame_idx].tolist()
    body_quat_w = motion["body_quat_w"][frame_idx].tolist()

    # In Isaac Lab body order, waist_yaw_link is at index 3.
    # RealtimeMotionBuffer.cpp:242 reads body index 13 (MuJoCo body order).
    # Copy the root body to index 13 so the receiver always finds it.
    body_pos_w[13] = body_pos_w[3]
    body_quat_w[13] = body_quat_w[3]

    payload = {
        "publish_t_ns": time.time_ns(),
        "joint_pos": joint_pos[ISAAC_TO_MUJOCO].tolist() if len(joint_pos) == 27 else joint_pos.tolist(),
        "body_pos_w": body_pos_w,
        "body_quat_w": body_quat_w,
    }
    if seq is not None:
        payload["seq"] = int(seq)
    if stream_id is not None:
        payload["stream_id"] = stream_id
    return json.dumps(payload, separators=(",", ":"))


def main() -> None:
    args = build_argparser().parse_args()

    motion = load_motion(args.npz)
    hz = args.hz if args.hz > 0 else motion["fps"]
    period_s = 1.0 / hz
    num_frames = motion["num_frames"]

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.bind(args.bind)

    override_upper = not args.keep_upper_body
    print(f"[pub_motion_npz] file={args.npz} frames={num_frames} hz={hz:.1f} bind={args.bind}")
    if override_upper:
        print("[pub_motion_npz] Upper-body joints overridden with default qpos (pass --keep-upper-body to disable)")
    else:
        print("[pub_motion_npz] Upper-body joints preserved from NPZ motion")
    if args.loop:
        print("[pub_motion_npz] looping indefinitely (Ctrl+C to stop)")
    print(f"[pub_motion_npz] startup sleep {args.startup_sleep}s...")
    time.sleep(args.startup_sleep)

    frame = 0
    seq = 0
    stream_id = uuid.uuid4().hex
    try:
        while True:
            msg = frame_to_json(
                frame, motion, override_upper_body=override_upper,
                seq=seq, stream_id=stream_id,
            )
            sock.send_string(msg, flags=zmq.NOBLOCK)
            seq += 1

            frame += 1
            if frame >= num_frames:
                if args.loop:
                    frame = 0
                    print("[pub_motion_npz] loop restart")
                else:
                    print("[pub_motion_npz] done")
                    break

            time.sleep(period_s)
    except KeyboardInterrupt:
        print("[pub_motion_npz] interrupted")
    finally:
        sock.close(0)


if __name__ == "__main__":
    main()
