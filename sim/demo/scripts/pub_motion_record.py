#!/usr/bin/env python3
"""
Replay a recorded teleoperation episode (Parquet or NPZ) over ZMQ PUB to drive
FSM_JAKAMINIMIMIC_ZMQ / RealtimeMotionBuffer in simulation or on real robot.

Usage:
  # 1. 直接读取实机录制的 Parquet 文件向外发布 (50Hz 平滑插值；--loop 可持续循环):
  python scripts/pub_motion_record.py 20261016-180555/level-0/data/chunk-000/episode_000001.parquet --loop

  # 2. 读取并同时转换为 .npz 格式保存:
  python scripts/pub_motion_record.py episode_000001.parquet --save-npz motion_record.npz

  # 3. 仅转换为 .npz 文件不启动发布:
  python scripts/pub_motion_record.py episode_000001.parquet --save-npz motion_record.npz --export-only

  # 4. 以原始录制帧率 (30Hz) 发布 (不进行 50Hz 插值):
  python scripts/pub_motion_record.py episode_000001.parquet --no-interpolate --loop

  # 5. 发布 base_link pose（默认发布 waist_yaw_Link）:
  python scripts/pub_motion_record.py episode_000001.parquet --anchor base
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import zmq

# -----------------------------------------------------------------------------
# 关节映射关系说明:
# -----------------------------------------------------------------------------
# 1. 实机录制的 Parquet 中的 actions[:27] 已经是 MuJoCo / JAKA 顺序:
#    左腿6 (0~5) | 右腿6 (6~11) | 腰1 (12) | 左臂6 (13~18) | 右臂6 (19~24) | 颈2 (25~26)
# 2. RealtimeMotionBuffer.cpp (line 302) 在收到 JSON 后，会自动通过 kMujocoToIsaacIndex
#    将 MuJoCo 序转换为神经网络策略所需的 Isaac Lab 序。
# 3. 因此，与 pub_motion_npz.py (原输入为 Isaac 序需转 MuJoCo 序) 不同，
#    本脚本从 Parquet 读取的 joint_pos 本身就是 MuJoCo 序，无需再做逆重排！
# -----------------------------------------------------------------------------

# MuJoCo 顺序 -> Isaac Lab 顺序 (与 RealtimeMotionBuffer.cpp:302 一致)
MUJOCO_TO_ISAAC = np.array(
    [0, 6, 12, 1, 7, 13, 25, 19, 2, 8, 14, 26, 20, 3, 9, 15, 21, 4, 10, 16, 22, 5, 11, 17, 23, 18, 24],
    dtype=np.intp,
)

# Isaac Lab 顺序 -> MuJoCo 顺序 (MUJOCO_TO_ISAAC 的逆映射)
ISAAC_TO_MUJOCO = np.array(
    [0, 3, 8, 13, 17, 21, 1, 4, 9, 14, 18, 22, 2, 5, 10, 15, 19, 23, 25, 7, 12, 16, 20, 24, 26, 6, 11],
    dtype=np.intp,
)

ANCHOR_BODY_NAMES = {"waist": "waist_yaw_Link", "base": "base_link"}


def _relative_times(values, *, nanoseconds: bool, name: str) -> np.ndarray:
    # Subtract the epoch in int64 BEFORE converting ns to floating point seconds.
    if nanoseconds:
        values = np.asarray(values, dtype=np.int64)
        times = (values - values[0]).astype(np.float64) * 1e-9
    else:
        values = np.asarray(values, dtype=np.float64)
        times = values - values[0]
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError(f"{name} must contain finite, strictly increasing timestamps")
    return times


def _finish_motion(motion: dict) -> dict:
    n = len(motion["joint_pos"])
    if n == 0:
        raise ValueError("Motion has no frames")
    for key, shape in (("joint_pos", (n, 27)), ("anchor_pos_w", (n, 3)), ("anchor_quat_w", (n, 4))):
        if motion[key].shape != shape or not np.isfinite(motion[key]).all():
            raise ValueError(f"Invalid {key}: expected finite values with shape {shape}")
    if motion["timestamps"].shape != (n,):
        raise ValueError("Timestamp count does not match motion frames")
    if np.any(np.linalg.norm(motion["anchor_quat_w"], axis=1) < 1e-6):
        raise ValueError("Anchor quaternion has zero norm")
    if n > 1:
        motion["fps"] = (n - 1) / float(motion["timestamps"][-1] - motion["timestamps"][0])
    return motion


def load_parquet_or_npz(file_path: Path, anchor: str = "waist") -> dict:
    """Read a reference in JAKA joint order, selecting the requested link pose."""
    if anchor not in ANCHOR_BODY_NAMES:
        raise ValueError(f"Unsupported anchor: {anchor}; expected waist or base")
    anchor_body_name = ANCHOR_BODY_NAMES[anchor]
    file_path = Path(file_path)
    if not file_path.is_file():
        raise FileNotFoundError(f"File not found: {file_path}")
    suffix = file_path.suffix.lower()
    if suffix == ".parquet":
        import pyarrow.parquet as pq

        # actions[33:40] stores waist pose. Base pose requires the full body columns.
        parquet = pq.ParquetFile(file_path)
        names = parquet.schema_arrow.names
        time_key = next((k for k in ("timestamp_ns", "timestamp", "motion_publish_t_ns") if k in names), None)
        columns = ["actions"] + ([time_key] if time_key else [])
        if anchor == "base":
            body_columns = ["motion_body_pos_w", "motion_body_quat_w"]
            if missing := set(body_columns) - set(names):
                raise ValueError(f"--anchor base requires Parquet columns: {sorted(missing)}")
            columns += body_columns
        table = parquet.read(columns=columns)
        actions = np.asarray(table["actions"].to_pylist(), dtype=np.float32)
        if actions.ndim != 2 or actions.shape[0] == 0 or actions.shape[1] < 40:
            raise ValueError(f"Expected nonempty actions[N, >=40], got {actions.shape}")
        n = len(actions)
        if anchor == "base":
            pos = np.asarray(table["motion_body_pos_w"].to_pylist(), dtype=np.float32)
            quat = np.asarray(table["motion_body_quat_w"].to_pylist(), dtype=np.float32)
            if (pos.ndim != 2 or quat.ndim != 2 or pos.shape[1] < 3
                    or pos.shape[1] % 3 or quat.shape[1] % 4
                    or pos.shape[1] // 3 != quat.shape[1] // 4):
                raise ValueError("Invalid flattened Parquet body position/quaternion arrays")
            anchor_pos = pos.reshape(n, -1, 3)[:, 0].copy()
            anchor_quat = quat.reshape(n, -1, 4)[:, 0].copy()
        else:
            anchor_pos = actions[:, 33:36].copy()
            anchor_quat = actions[:, 36:40].copy()
        motion = {
            "source": "parquet", "fps": 30.0,
            "anchor_body_name": anchor_body_name,
            "joint_pos": actions[:, :27].copy(),
            "anchor_pos_w": anchor_pos,
            "anchor_quat_w": anchor_quat,
            "actions_full": actions,
        }
        if time_key:
            values = table[time_key].to_pylist()
            motion["timestamps"] = _relative_times(values, nanoseconds=time_key.endswith("_ns"), name=time_key)
            if time_key.endswith("_ns"):
                motion["recorded_timestamp_ns"] = np.asarray(values, dtype=np.int64)
        else:
            print("[pub_motion_record] WARNING: no recorded timestamps; assuming 30 Hz")
            motion["timestamps"] = np.arange(n, dtype=np.float64) / 30.0
        motion["time_source"] = time_key or "assumed_30hz"
        return _finish_motion(motion)

    if suffix == ".npz":
        with np.load(file_path, allow_pickle=False) as data:
            required = {"joint_pos", "body_pos_w", "body_quat_w"}
            if missing := required - set(data.files):
                raise KeyError(f"NPZ missing keys: {missing}")
            raw_recording = "recv_t_ns" in data and "publish_t_ns" in data
            fps = float(data["fps"].flat[0]) if "fps" in data else 50.0
            if not np.isfinite(fps) or fps <= 0:
                raise ValueError("NPZ fps must be positive and finite")
            joint_pos = data["joint_pos"].astype(np.float32)
            pos = data["body_pos_w"].astype(np.float32)
            quat = data["body_quat_w"].astype(np.float32)
            # motion_raw.npz is JAKA order. Exported records explicitly carry order.
            # Unmarked training motion NPZ files use the Isaac convention.
            order = str(data["joint_order"].item()) if "joint_order" in data else ("mujoco" if raw_recording else "isaac")
            if order == "isaac":
                joint_pos = joint_pos[:, ISAAC_TO_MUJOCO]
                anchor_index = 3
            elif order == "mujoco":
                anchor_index = 13
            else:
                raise ValueError(f"Unsupported NPZ joint_order: {order}")
            if anchor == "base":
                anchor_index = 0
            # Exports contain a single selected pose repeated in every body slot.
            # They cannot supply the other link's pose, even though its index exists.
            pose_format = str(data["body_pose_format"].item()) if "body_pose_format" in data else ""
            repeated_pose = (
                pos.ndim == 2
                or (pos.ndim == 3 and pos.shape[1] > 0
                    and np.all(pos == pos[:, :1]) and np.all(quat == quat[:, :1]))
            )
            if pose_format == "anchor_repeated" or repeated_pose:
                # Older exports had no label and always contained waist pose.
                stored_anchor = str(data["anchor_body_name"].item()) if "anchor_body_name" in data else "waist_yaw_Link"
                if stored_anchor != anchor_body_name:
                    raise ValueError(
                        f"NPZ contains only {stored_anchor} pose; cannot select {anchor_body_name}. "
                        "Use the original Parquet or motion_raw.npz with full body poses."
                    )
                anchor_index = 0
            if pos.ndim == 3:
                anchor_pos, anchor_quat = pos[:, anchor_index], quat[:, anchor_index]
            else:
                anchor_pos, anchor_quat = pos, quat
            n = len(joint_pos)
            if n == 0:
                raise ValueError("Motion has no frames")
            time_key = next((k for k in ("timestamps", "publish_t_ns", "recv_t_ns") if k in data), None)
            motion = {
                "source": "raw_npz" if raw_recording else "npz", "fps": fps,
                "anchor_body_name": anchor_body_name,
                "joint_pos": joint_pos, "anchor_pos_w": anchor_pos,
                "anchor_quat_w": anchor_quat, "body_pos_w": pos, "body_quat_w": quat,
                "time_source": time_key or "fps",
            }
            if time_key:
                values = data[time_key]
                motion["timestamps"] = _relative_times(values, nanoseconds=time_key.endswith("_ns"), name=time_key)
                if time_key.endswith("_ns"):
                    motion["recorded_timestamp_ns"] = values.astype(np.int64)
            else:
                motion["timestamps"] = np.arange(n, dtype=np.float64) / fps
        return _finish_motion(motion)
    raise ValueError(f"Unsupported file format '{suffix}', expected .parquet or .npz")


def interpolate_motion(motion: dict, target_hz: float, speed: float = 1.0) -> dict:
    """
    Interpolate motion trajectory to target_hz with proper linear/slerp interpolation.
    """
    from scipy.interpolate import interp1d
    from scipy.spatial.transform import Rotation, Slerp

    timestamps = motion["timestamps"]
    joint_pos = motion["joint_pos"]
    pos_w = motion["anchor_pos_w"]
    quat_w = motion["anchor_quat_w"]  # wxyz

    n_frames = len(timestamps)
    if n_frames <= 1:
        return motion

    # Invalid clocks must be reported, rather than silently inventing 0.1 ms gaps.
    t_fixed = _relative_times(timestamps, nanoseconds=False, name=motion.get("time_source", "timestamps"))

    t_start, t_end = t_fixed[0], t_fixed[-1]
    duration = (t_end - t_start) / max(speed, 1e-4)
    target_dt = 1.0 / target_hz
    num_target_frames = max(2, int(round(duration * target_hz)) + 1)
    t_new = np.linspace(t_start, t_end, num_target_frames)

    # 关节角度与位置线性插值
    f_joints = interp1d(t_fixed, joint_pos, axis=0, kind="linear", fill_value="extrapolate")
    joint_pos_new = f_joints(t_new).astype(np.float32)

    f_pos = interp1d(t_fixed, pos_w, axis=0, kind="linear", fill_value="extrapolate")
    pos_w_new = f_pos(t_new).astype(np.float32)

    # 四元数 Slerp 插值 (注意 scipy Rotation 使用 xyzw 顺序)
    # 输入为 wxyz -> 转为 xyzw
    quat_xyzw = quat_w[:, [1, 2, 3, 0]]
    # 归一化四元数
    norms = np.linalg.norm(quat_xyzw, axis=1, keepdims=True)
    norms[norms < 1e-6] = 1.0
    quat_xyzw /= norms

    rots = Rotation.from_quat(quat_xyzw)
    slerp = Slerp(t_fixed, rots)
    rots_new = slerp(t_new)
    quat_new_xyzw = rots_new.as_quat()
    # 转回 wxyz
    quat_w_new = quat_new_xyzw[:, [3, 0, 1, 2]].astype(np.float32)

    return {
        "source": motion["source"],
        "anchor_body_name": motion.get("anchor_body_name", "waist_yaw_Link"),
        "time_source": motion.get("time_source", "timestamps"),
        "fps": target_hz,
        "timestamps": np.linspace(0.0, duration, num_target_frames),
        "joint_pos": joint_pos_new,
        "anchor_pos_w": pos_w_new,
        "anchor_quat_w": quat_w_new,
    }


def export_to_npz(motion: dict, output_path: Path) -> Path:
    """Save processed motion dictionary into .npz format."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    n_frames = motion["joint_pos"].shape[0]
    joint_pos = motion["joint_pos"].astype(np.float32)
    pos_w = motion["anchor_pos_w"].astype(np.float32)
    quat_w = motion["anchor_quat_w"].astype(np.float32)  # wxyz
    timestamps = motion["timestamps"].astype(np.float64)
    fps = float(motion["fps"])

    # Anchor placeholders for this tracker; these are not 28-body FK trajectories.
    body_pos_w = np.zeros((n_frames, 28, 3), dtype=np.float32)
    body_quat_w = np.zeros((n_frames, 28, 4), dtype=np.float32)
    for b in range(28):
        body_pos_w[:, b, :] = pos_w
        body_quat_w[:, b, :] = quat_w

    # Differentiate on the actual recorded axis, including its sampling jitter.
    joint_vel = np.gradient(joint_pos, timestamps, axis=0).astype(np.float32) if n_frames > 1 else np.zeros_like(joint_pos)
    body_lin_vel_w = np.gradient(body_pos_w, timestamps, axis=0).astype(np.float32) if n_frames > 1 else np.zeros_like(body_pos_w)
    body_ang_vel_w = np.zeros((n_frames, 28, 3), dtype=np.float32)

    # 对应 Isaac Lab 顺序的关节角度 (供需要 Isaac 序的工具使用)
    joint_pos_isaac = joint_pos[:, MUJOCO_TO_ISAAC].astype(np.float32)

    save_dict = {
        "fps": np.array([fps], dtype=np.float64),
        "joint_pos": joint_pos,                 # MuJoCo 顺序 (27)
        "joint_pos_isaac": joint_pos_isaac,     # Isaac Lab 顺序 (27)
        "joint_vel": joint_vel,
        "body_pos_w": body_pos_w,               # (N, 28, 3)
        "body_quat_w": body_quat_w,             # (N, 28, 4) wxyz
        "body_lin_vel_w": body_lin_vel_w,
        "body_ang_vel_w": body_ang_vel_w,
        "timestamps": timestamps,
        "joint_order": np.array("mujoco"),
        "anchor_body_name": np.array(motion.get("anchor_body_name", "waist_yaw_Link")),
        "body_pose_format": np.array("anchor_repeated"),
    }
    if "recorded_timestamp_ns" in motion:
        save_dict["recorded_timestamp_ns"] = motion["recorded_timestamp_ns"]
    if "actions_full" in motion:
        save_dict["actions"] = motion["actions_full"]

    np.savez_compressed(output_path, **save_dict)
    return output_path


def build_json_payload(frame_idx: int, motion: dict) -> str:
    """
    Format single frame into JSON string expected by RealtimeMotionBuffer.cpp:
    {
        "joint_pos": [27 floats in MuJoCo order],
        "body_pos_w": [[x, y, z] * 28],
        "body_quat_w": [[w, x, y, z] * 28]
    }
    """
    joint_pos = motion["joint_pos"][frame_idx].tolist()
    anchor_pos = motion["anchor_pos_w"][frame_idx].tolist()
    anchor_quat = motion["anchor_quat_w"][frame_idx].tolist()  # wxyz

    # Every slot contains the selected link pose: --anchor base or --anchor waist.
    payload = {
        "joint_pos": [round(float(v), 5) for v in joint_pos],
        "body_pos_w": [[round(float(v), 5) for v in anchor_pos]] * 28,
        "body_quat_w": [[round(float(v), 5) for v in anchor_quat]] * 28,
        "publish_t_ns": time.time_ns(),
    }
    return json.dumps(payload, separators=(",", ":"))


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Publish recorded teleoperation episode (Parquet/NPZ) via ZMQ PUB for MuJoCo mimic"
    )
    p.add_argument(
        "input",
        type=Path,
        help="Path to .parquet or .npz recorded motion file",
    )
    p.add_argument(
        "--bind",
        default="tcp://*:28701",
        help="ZMQ PUB bind address (default: tcp://*:28701, matching jakaminimimic_zmq)",
    )
    p.add_argument(
        "--anchor",
        choices=tuple(ANCHOR_BODY_NAMES),
        default="waist",
        help="Link pose to copy into all 28 body slots: waist (default) or base",
    )
    p.add_argument(
        "--hz",
        type=float,
        default=None,
        help="Publish Hz (default 50; with --no-interpolate defaults to recorded fps; 0 = recorded fps)",
    )
    p.add_argument(
        "--no-interpolate",
        action="store_true",
        help="Disable interpolation; play raw recorded frames at requested --hz",
    )
    p.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (e.g. 1.0 = normal, 0.5 = half speed)",
    )
    p.add_argument(
        "--loop",
        action="store_true",
        help="Loop playback indefinitely (Ctrl+C to stop)",
    )
    p.add_argument(
        "--startup-sleep",
        type=float,
        default=1.0,
        help="Sleep before first publish to allow subscriber handshake (default: 1.0s)",
    )
    p.add_argument(
        "--save-npz",
        type=Path,
        default=None,
        nargs="?",
        const="",
        help="Optional path to save/export motion to .npz file (if flag passed without value, saves with same name)",
    )
    p.add_argument(
        "--export-only",
        action="store_true",
        help="Only export to .npz and exit immediately without publishing",
    )
    return p


def main() -> None:
    args = build_argparser().parse_args()

    # 1. 加载动作数据
    print(f"[pub_motion_record] Loading motion from: {args.input}")
    motion = load_parquet_or_npz(args.input, anchor=args.anchor)
    raw_frames = motion["joint_pos"].shape[0]
    raw_fps = motion["fps"]
    duration = (motion["timestamps"][-1] - motion["timestamps"][0]) if raw_frames > 1 else 0.0
    print(
        f"[pub_motion_record] Loaded {raw_frames} frames ({duration:.6f}s, recorded_fps={raw_fps:.4f}) "
        f"from {motion['source']}, time_source={motion['time_source']}, "
        f"anchor={motion['anchor_body_name']}"
    )
    if not np.isfinite(args.speed) or args.speed <= 0:
        raise ValueError("--speed must be positive and finite")
    if not np.isfinite(args.startup_sleep) or args.startup_sleep < 0:
        raise ValueError("--startup-sleep must be nonnegative and finite")

    # 2. 导出 NPZ (如果指定了 --save-npz 或 --export-only)
    if args.save_npz is not None or args.export_only:
        if args.save_npz == Path("") or args.save_npz is None:
            out_npz = args.input.with_suffix(".npz")
        else:
            out_npz = args.save_npz
        saved_path = export_to_npz(motion, out_npz)
        print(f"[pub_motion_record] Successfully exported NPZ to: {saved_path}")
        if args.export_only:
            print("[pub_motion_record] --export-only specified, exiting.")
            return

    # 3. 决定发布帧率与插值
    target_hz = (raw_fps if args.no_interpolate else 50.0) if args.hz is None else (raw_fps if args.hz == 0 else args.hz)
    if not np.isfinite(target_hz) or target_hz <= 0:
        raise ValueError("--hz must be nonnegative and finite")
    do_interp = not args.no_interpolate

    if do_interp:
        print(
            f"[pub_motion_record] Interpolating trajectory: {raw_fps:.1f}Hz -> {target_hz:.1f}Hz "
            f"(speed={args.speed}x)..."
        )
        motion = interpolate_motion(motion, target_hz=target_hz, speed=args.speed)
    else:
        target_hz *= args.speed
        print(f"[pub_motion_record] Using original frames at {target_hz:.4f}Hz (speed={args.speed}x)")

    num_frames = motion["joint_pos"].shape[0]
    period_s = 1.0 / target_hz

    # 4. 初始化 ZMQ PUB
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.bind(args.bind)

    print(
        f"[pub_motion_record] Bound to {args.bind} | Ready to publish {num_frames} frames "
        f"at {target_hz:.1f}Hz (period={period_s*1000:.1f}ms)"
    )
    print(f"[pub_motion_record] Joint order: MuJoCo (27 DoF, RealtimeMotionBuffer compatible)")
    print(f"[pub_motion_record] Anchor: {motion['anchor_body_name']} (pose copied to all 28 body slots)")
    if args.loop:
        print("[pub_motion_record] Looping indefinitely enabled (Ctrl+C to stop)")
    print(f"[pub_motion_record] Waiting {args.startup_sleep}s for subscribers to connect...")
    time.sleep(args.startup_sleep)

    # 5. Fresh publish_t_ns above belongs to this replay, never the old recording clock.
    #    Recorded timestamps control trajectory interpolation, not the receiver's epoch.
    frame = 0
    loop_count = 0
    t_start = time.monotonic()
    next_time = t_start

    print("\n[pub_motion_record] >>> Publishing started from frame 0 <<<\n")
    try:
        while True:
            msg = build_json_payload(frame, motion)
            sock.send_string(msg, flags=zmq.NOBLOCK)

            frame += 1
            if frame % int(max(1, target_hz)) == 0 or frame == num_frames:
                pct = (frame / num_frames) * 100
                print(
                    f"\r[pub_motion_record] [Loop {loop_count}] Frame {frame}/{num_frames} ({pct:.1f}%) "
                    f"root_h={motion['anchor_pos_w'][frame-1][2]:.3f}m",
                    end="",
                    flush=True,
                )

            if frame >= num_frames:
                print(f"\n[pub_motion_record] [Loop {loop_count}] Finished playing {num_frames} frames.")
                if args.loop:
                    frame = 0
                    loop_count += 1
                    next_time = time.monotonic()
                else:
                    print("[pub_motion_record] All frames completed.")
                    break

            # 高精度防漂移定时
            next_time += period_s
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -period_s:
                # 若系统卡顿落后较多，重置基准时间避免追赶突发
                next_time = time.monotonic()

    except KeyboardInterrupt:
        print("\n[pub_motion_record] Stopped by user (Ctrl+C).")
    finally:
        sock.close(0)
        print("[pub_motion_record] ZMQ socket closed.")


if __name__ == "__main__":
    main()
