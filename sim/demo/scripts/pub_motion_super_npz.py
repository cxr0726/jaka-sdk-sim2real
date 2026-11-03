#!/usr/bin/env python3
"""Publish legacy Isaac or named Khan NPZ motions in the canonical JAKA wire order.

Named NPZ columns are resolved by name, independently of their array order.
Unnamed 27-joint/28-body files default to legacy Isaac layout. Other unnamed
layouts require an explicit declaration. All NPZ body quaternions are wxyz;
source_quat_order describes the converter's input, not its NPZ output.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time
import uuid

import numpy as np


JAKA_JOINT_NAMES = (
    "Left_hip_pitch_joint", "Left_hip_roll_joint", "Left_hip_yaw_joint", "Left_knee_joint",
    "Left_ankle_pitch_joint", "Left_ankle_roll_joint",
    "Right_hip_pitch_joint", "Right_hip_roll_joint", "Right_hip_yaw_joint", "Right_knee_joint",
    "Right_ankle_pitch_joint", "Right_ankle_roll_joint", "waist_yaw_joint",
    "Left_shoulder_pitch_joint", "Left_shoulder_roll_joint", "Left_shoulder_yaw_joint",
    "Left_elbow_joint", "Left_wrist_roll_joint", "Left_wrist_yaw_joint",
    "Right_shoulder_pitch_joint", "Right_shoulder_roll_joint", "Right_shoulder_yaw_joint",
    "Right_elbow_joint", "Right_wrist_roll_joint", "Right_wrist_yaw_joint",
    "Neck_yaw_joint", "Neck_pitch_joint",
)

ISAAC_JOINT_NAMES = (
    "Left_hip_pitch_joint", "Right_hip_pitch_joint", "waist_yaw_joint",
    "Left_hip_roll_joint", "Right_hip_roll_joint", "Left_shoulder_pitch_joint",
    "Neck_yaw_joint", "Right_shoulder_pitch_joint", "Left_hip_yaw_joint",
    "Right_hip_yaw_joint", "Left_shoulder_roll_joint", "Neck_pitch_joint",
    "Right_shoulder_roll_joint", "Left_knee_joint", "Right_knee_joint",
    "Left_shoulder_yaw_joint", "Right_shoulder_yaw_joint", "Left_ankle_pitch_joint",
    "Right_ankle_pitch_joint", "Left_elbow_joint", "Right_elbow_joint",
    "Left_ankle_roll_joint", "Right_ankle_roll_joint", "Left_wrist_roll_joint",
    "Right_wrist_roll_joint", "Left_wrist_yaw_joint", "Right_wrist_yaw_joint",
)


@dataclass(frozen=True)
class Motion:
    layout: str
    fps: float
    reference_height_offset: float
    joint_pos: np.ndarray
    body_pos_w: np.ndarray
    body_quat_w: np.ndarray

    @property
    def frames(self) -> int:
        return int(self.joint_pos.shape[0])

    @property
    def duration_s(self) -> float:
        return (self.frames - 1) / self.fps

    def describe(self) -> dict:
        return {"layout": self.layout, "frames": self.frames, "fps": self.fps,
                "reference_height_offset": self.reference_height_offset}


def _names(values, count: int, label: str) -> list[str]:
    values = np.asarray(values)
    if values.shape != (count,):
        raise ValueError(f"{label} must have shape ({count},), got {values.shape}")
    names = []
    for value in values.tolist():
        if isinstance(value, bytes):
            value = value.decode("utf-8")
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{label} must contain nonempty strings")
        names.append(value.strip().casefold())
    if len(set(names)) != count:
        raise ValueError(f"{label} contains duplicate names")
    return names


def _joint_gather(source_names: list[str]) -> list[int]:
    target = [name.casefold() for name in JAKA_JOINT_NAMES]
    if set(source_names) != set(target):
        raise ValueError("joint_names must name exactly the 27 Khan/JAKA joints; "
                         f"missing={sorted(set(target) - set(source_names))}, "
                         f"unknown={sorted(set(source_names) - set(target))}")
    return [source_names.index(name) for name in target]


def load_motion(path: str | Path, layout: str = "auto") -> Motion:
    """Validate the whole clip and normalize exactly once to JAKA wire arrays.

    Only body slot 0 (base) is meaningful for SuperMimic. Other
    slots are zero-position/identity-quaternion placeholders, not fabricated FK.
    No joint positions or root heights are changed.
    """
    if layout not in ("auto", "isaac", "jaka"):
        raise ValueError("layout must be auto, isaac, or jaka")
    with np.load(path, allow_pickle=False) as data:
        missing = {"joint_pos", "body_pos_w", "body_quat_w"} - set(data.files)
        if missing:
            raise ValueError(f"NPZ missing required arrays: {sorted(missing)}")
        q = np.asarray(data["joint_pos"], dtype=np.float64)
        pos = np.asarray(data["body_pos_w"], dtype=np.float64)
        quat = np.asarray(data["body_quat_w"], dtype=np.float64)
        if q.ndim != 2 or q.shape[1] != 27 or q.shape[0] < 1:
            raise ValueError(f"joint_pos must have shape (T,27), T >= 1; got {q.shape}")
        frames = q.shape[0]
        if pos.ndim != 3 or pos.shape[0] != frames or pos.shape[2] != 3:
            raise ValueError(f"body_pos_w must have shape ({frames},B,3); got {pos.shape}")
        bodies = pos.shape[1]
        if quat.shape != (frames, bodies, 4):
            raise ValueError(f"body_quat_w must have shape ({frames},{bodies},4); got {quat.shape}")
        for name, value in (("joint_pos", q), ("body_pos_w", pos), ("body_quat_w", quat)):
            if not np.isfinite(value).all():
                raise ValueError(f"{name} contains NaN or infinity")
        norms = np.linalg.norm(quat, axis=-1, keepdims=True)
        if not np.isfinite(norms).all() or (norms < 1e-8).any():
            raise ValueError("body_quat_w contains a zero or invalid quaternion")
        quat = quat / norms
        fps_value = np.asarray(data["fps"] if "fps" in data else 50.0)
        if fps_value.size != 1:
            raise ValueError("fps must contain exactly one number")
        fps = float(fps_value.reshape(-1)[0])
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("fps must be finite and positive")

        has_joints, has_bodies = "joint_names" in data, "body_names" in data
        if has_joints != has_bodies:
            raise ValueError("Partial NPZ name metadata: joint_names and body_names must both be present")
        if has_joints:
            joint_names = _names(data["joint_names"], 27, "joint_names")
            body_names = _names(data["body_names"], bodies, "body_names")
            gather = _joint_gather(joint_names)
            if "base_link" not in body_names:
                raise ValueError("body_names must contain base_link for SuperMimic")
            base = body_names.index("base_link")
            effective_layout, height_offset = "jaka", 0.0
        else:
            if layout == "auto":
                if bodies != 28:
                    raise ValueError("Cannot infer unnamed NPZ layout: auto accepts only legacy "
                                     "27-joint/28-body Isaac files; supply joint_names/body_names "
                                     "or explicitly select --layout isaac|jaka")
                layout = "isaac"
            if layout == "isaac":
                base = 0
                gather = _joint_gather([name.casefold() for name in ISAAC_JOINT_NAMES])
                effective_layout, height_offset = "isaac", 0.05
            else:
                base = 0
                gather = list(range(27))
                effective_layout, height_offset = "jaka", 0.0
            if bodies <= base:
                raise ValueError(f"Unnamed {layout} layout requires base body index 0; got {bodies} bodies")

    wire_pos = np.zeros((frames, 28, 3), dtype=np.float64)
    wire_quat = np.zeros((frames, 28, 4), dtype=np.float64)
    wire_quat[..., 0] = 1.0
    wire_pos[:, 0] = pos[:, base]
    wire_quat[:, 0] = quat[:, base]
    return Motion(effective_layout, fps, height_offset, q[:, gather].copy(), wire_pos, wire_quat)


def _slerp(left: np.ndarray, right: np.ndarray, fraction: float) -> np.ndarray:
    """Batched shortest-path SLERP with a stable normalized-linear limit."""
    dot = np.sum(left * right, axis=-1, keepdims=True)
    right = np.where(dot < 0.0, -right, right)
    dot = np.clip(np.abs(dot), 0.0, 1.0)
    theta = np.arccos(dot)
    denominator = np.sin(theta)
    safe_denominator = np.where(denominator > 1e-8, denominator, 1.0)
    spherical = (np.sin((1.0 - fraction) * theta) * left
                 + np.sin(fraction * theta) * right) / safe_denominator
    linear = (1.0 - fraction) * left + fraction * right
    result = np.where(dot > 0.9995, linear, spherical)
    return result / np.linalg.norm(result, axis=-1, keepdims=True)


def sample_motion(motion: Motion, elapsed_s: float, loop: bool = False):
    """Return (q, body_pos, body_quat, finished), sampled in source-time seconds.

    Nonlooping clips clamp to the exact last pose indefinitely. A loop retains
    the last sample for its final 1/fps interval, then wraps to the first sample.
    """
    if not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError("elapsed_s must be finite and nonnegative")
    sample_s = elapsed_s % (motion.frames / motion.fps) if loop else elapsed_s
    coordinate = min(sample_s * motion.fps, motion.frames - 1)
    first = int(math.floor(coordinate))
    second = min(first + 1, motion.frames - 1)
    fraction = coordinate - first
    q = (1.0 - fraction) * motion.joint_pos[first] + fraction * motion.joint_pos[second]
    pos = (1.0 - fraction) * motion.body_pos_w[first] + fraction * motion.body_pos_w[second]
    quat = _slerp(motion.body_quat_w[first], motion.body_quat_w[second], fraction)
    return q, pos, quat, not loop and elapsed_s >= motion.duration_s


def make_payload(motion: Motion, elapsed_s: float, *, motion_t_ns: int, seq: int,
                 stream_id: str, source_clock_id: str, loop: bool = False):
    q, pos, quat, finished = sample_motion(motion, elapsed_s, loop=loop)
    payload = {
        "source_time_field": "motion_t_ns", "source_clock_id": source_clock_id,
        "stream_id": stream_id, "seq": seq, "motion_t_ns": motion_t_ns,
        "joint_pos": q.tolist(), "body_pos_w": pos.tolist(), "body_quat_w": quat.tolist(),
    }
    # Stamp publication after sampling and constructing the pose payload. This
    # wall clock is diagnostic only; the receiver uses the declared monotonic clock.
    payload["publish_t_ns"] = time.time_ns()
    return payload, finished


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("npz", type=Path)
    parser.add_argument("--layout", choices=("auto", "isaac", "jaka"), default="auto")
    parser.add_argument("--describe", action="store_true", help="Validate and print JSON without importing ZMQ or opening a socket")
    parser.add_argument("--bind", default="tcp://*:28701")
    parser.add_argument("--hz", type=float, default=50.0, help="Publish frequency; source motion always runs at its own NPZ fps")
    parser.add_argument("--startup-sleep", type=float, default=0.25)
    end = parser.add_mutually_exclusive_group()
    end.add_argument("--loop", action="store_true")
    end.add_argument("--exit-on-end", action="store_true", help="Exit after publishing the last pose instead of holding it")
    return parser


def main(argv=None) -> int:
    parser = build_argparser()
    args = parser.parse_args(argv)
    if not math.isfinite(args.hz) or args.hz <= 0:
        parser.error("--hz must be finite and positive")
    if not math.isfinite(args.startup_sleep) or args.startup_sleep < 0:
        parser.error("--startup-sleep must be finite and nonnegative")
    try:
        motion = load_motion(args.npz, args.layout)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    if args.describe:
        print(json.dumps(motion.describe(), allow_nan=False))
        return 0

    # Keep --describe and the numerical tests usable with Python + NumPy only.
    import zmq

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SNDHWM, 1)
    socket.setsockopt(zmq.CONFLATE, 1)
    try:
        socket.bind(args.bind)
        print(f"[pub_motion_super_npz] {json.dumps(motion.describe())} publish_hz={args.hz:g} bind={args.bind}", flush=True)
        time.sleep(args.startup_sleep)
        stream_id = uuid.uuid4().hex
        source_clock_id = "npz-monotonic:" + stream_id
        started_ns = time.monotonic_ns()
        next_ns = started_ns
        period_ns = max(1, round(1e9 / args.hz))
        seq, finished_reported = 0, False
        while True:
            remaining_ns = next_ns - time.monotonic_ns()
            if remaining_ns > 0:
                time.sleep(remaining_ns / 1e9)
            sampled_ns = time.monotonic_ns()
            payload, finished = make_payload(
                motion, (sampled_ns - started_ns) / 1e9, motion_t_ns=sampled_ns,
                seq=seq, stream_id=stream_id, source_clock_id=source_clock_id, loop=args.loop,
            )
            try:
                socket.send_string(json.dumps(payload, separators=(",", ":"), allow_nan=False), flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            seq += 1
            if finished:
                if not finished_reported:
                    behavior = "exiting" if args.exit_on_end else "holding final pose with live timestamps"
                    print(f"[pub_motion_super_npz] finished; {behavior}", flush=True)
                    finished_reported = True
                if args.exit_on_end:
                    break
            next_ns += period_ns
            # After a scheduling pause, skip missed slots rather than burst old poses.
            now_ns = time.monotonic_ns()
            if next_ns < now_ns:
                next_ns = now_ns + period_ns
    except KeyboardInterrupt:
        pass
    finally:
        socket.close(0)
        context.term()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
