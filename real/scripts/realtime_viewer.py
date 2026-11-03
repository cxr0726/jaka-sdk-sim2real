#!/usr/bin/env python3
"""
Real-time Kinematic Viewer for JAKA Humanoid Robot via ZMQ.

Unlike `./scripts/run_demo.sh --viewer --duration 0 --mode jakaminimimic_zmq`
which runs full WBC physics simulation and RL policy inference, this viewer
is a lightweight, pure-kinematic visualizer.

位姿解析模式:
  1. 默认模式 (未开启 --root):
     在实机录制的 Parquet 文件 (及 pico 远控数据) 中，anchor pos 与 anchor quat 对应
     的是腰部连杆 `waist_yaw_Link` 的世界系位姿。
     本脚本通过正运动学几何关系利用输入的 waist_yaw 关节角度与 anchor 位姿反解出 `base_link` 的位姿:
       R_base = R_waist * inv(R_joint(q_waist_yaw)) * inv(R_offset)
       p_base = p_waist - R_base * p_waist_rel
     然后赋给 `data.qpos[0:3]` 与 `data.qpos[3:7]`。

  2. 全身 Root 模式 (开启 --root):
     当开启 `--root` 时，视 ZMQ payload 中的 body_pos_w 和 body_quat_w 为真正的全身 28 个
     link 的独立位姿，此时第 0 个元素即为机器人根节点 `base_link` 的真实世界系位姿。
     脚本将直接提取 `body_pos_w[0]` 和 `body_quat_w[0]` 赋值给:
       data.qpos[0:3] = body_pos_w[0]
       data.qpos[3:7] = body_quat_w[0]
     无需任何几何推导与反解。

Usage:
  # 1. 默认实时接收模式 (解算腰部->基座位姿，订阅 tcp://127.0.0.1:28701):
  python scripts/realtime_viewer.py

  # 2. 开启 --root 模式 (直接取 body_pos_w[0] 与 body_quat_w[0] 赋给 base_link):
  python scripts/realtime_viewer.py --root

  # 3. 指定远控机 IP (如多机联调):
  python scripts/realtime_viewer.py --connect tcp://10.5.5.120:28701 --root

  # 4. 锁定基座 X/Y 在原点 (原地观察运动):
  python scripts/realtime_viewer.py --lock-origin --root

  # 5. 离线文件直接回放可视化:
  python scripts/realtime_viewer.py --file 20261016-180555/level-0/data/chunk-000/episode_000001.parquet --loop
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation
import zmq

# -----------------------------------------------------------------------------
# 27 关节列表 (MuJoCo / JAKA 顺序，与 Khan_mini_sym.xml 及 pub_motion_record 一致)
# -----------------------------------------------------------------------------
JAKA_JOINT_NAMES = [
    # 左腿 (6)
    "Left_hip_pitch_joint",
    "Left_hip_roll_joint",
    "Left_hip_yaw_joint",
    "Left_knee_joint",
    "Left_ankle_pitch_joint",
    "Left_ankle_roll_joint",
    # 右腿 (6)
    "Right_hip_pitch_joint",
    "Right_hip_roll_joint",
    "Right_hip_yaw_joint",
    "Right_knee_joint",
    "Right_ankle_pitch_joint",
    "Right_ankle_roll_joint",
    # 腰部 (1, 索引 12)
    "waist_yaw_joint",
    # 左臂 (6)
    "Left_shoulder_pitch_joint",
    "Left_shoulder_roll_joint",
    "Left_shoulder_yaw_joint",
    "Left_elbow_joint",
    "Left_wrist_roll_joint",
    "Left_wrist_yaw_joint",
    # 右臂 (6)
    "Right_shoulder_pitch_joint",
    "Right_shoulder_roll_joint",
    "Right_shoulder_yaw_joint",
    "Right_elbow_joint",
    "Right_wrist_roll_joint",
    "Right_wrist_yaw_joint",
    # 颈部 (2)
    "Neck_yaw_joint",
    "Neck_pitch_joint",
]


def find_default_xml() -> Path:
    """Auto-locate Khan_mini_sym.xml relative to script or repo root."""
    script_dir = Path(__file__).resolve().parent
    candidates = [
        script_dir.parents[1] / "sim/sdk/assets/khan_mini/Khan_mini_sym.xml",
        script_dir.parents[0] / "sim/sdk/assets/khan_mini/Khan_mini_sym.xml",
        Path("/home/irmv/Workspace/jaka_ws/jaka-humanoid-sdk/sim/sdk/assets/khan_mini/Khan_mini_sym.xml"),
    ]
    for c in candidates:
        if c.exists():
            return c
    raise FileNotFoundError(
        "Could not find Khan_mini_sym.xml. Please specify via --xml <path/to/Khan_mini_sym.xml>"
    )


def quat_to_euler_rpy(q: np.ndarray) -> Tuple[float, float, float]:
    """Convert wxyz quaternion to roll, pitch, yaw (in degrees)."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return float(np.degrees(roll)), float(np.degrees(pitch)), float(np.degrees(yaw))


class RobotKinematicsAdapter:
    """
    Maps incoming ZMQ motion frames into MuJoCo model coordinates.
    Supports either direct base_link assignment (--root) or solving base_link from waist_yaw_Link.
    """

    def __init__(self, model: mujoco.MjModel, lock_origin: bool = False, anchor_is_waist: bool = True):
        self.model = model
        self.lock_origin = lock_origin
        self.anchor_is_waist = anchor_is_waist

        # 查找 27 个主要关节的 qpos 偏移地址
        self.jnt_addrs: List[int] = []
        for name in JAKA_JOINT_NAMES:
            jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jnt_id == -1:
                print(f"[realtime_viewer] WARN: Joint '{name}' not found in XML model, will skip")
                self.jnt_addrs.append(-1)
            else:
                self.jnt_addrs.append(model.jnt_qposadr[jnt_id])

        # 查找踝关节四连杆 drive 关节 (若存在，随 pitch/roll 同步计算更新驱动杆位姿)
        self.drive_addrs = {}
        for dname in ["Left_drive1_joint", "Left_drive2_joint", "Right_drive1_joint", "Right_drive2_joint"]:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, dname)
            self.drive_addrs[dname] = model.jnt_qposadr[jid] if jid != -1 else None

        self.base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.waist_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "waist_yaw_Link")
        self.waist_jnt_idx = JAKA_JOINT_NAMES.index("waist_yaw_joint") if "waist_yaw_joint" in JAKA_JOINT_NAMES else 12

        self.has_freejoint = model.nq >= 7 and model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE

        # 从模型 XML 中读取 waist_yaw_Link 相对于父级 base_link 的几何偏置 (p_rel, R_rel)
        if self.waist_body_id != -1:
            self.p_waist_rel = np.array(model.body_pos[self.waist_body_id], dtype=np.float64)
            q_waist_rel_wxyz = np.array(model.body_quat[self.waist_body_id], dtype=np.float64)
            q_xyzw = q_waist_rel_wxyz[[1, 2, 3, 0]]
            norm_q = np.linalg.norm(q_xyzw)
            if norm_q > 1e-6:
                q_xyzw /= norm_q
            self.rot_waist_offset = Rotation.from_quat(q_xyzw)
            if self.anchor_is_waist:
                print(
                    f"[realtime_viewer] Kinematics init: waist_yaw_Link relative offset to base_link is "
                    f"pos={self.p_waist_rel.tolist()}"
                )
        else:
            self.p_waist_rel = np.array([0.0, 0.0, 0.22385], dtype=np.float64)
            self.rot_waist_offset = Rotation.identity()

    def solve_base_pose_from_waist(
        self,
        waist_pos: np.ndarray,
        waist_quat_wxyz: np.ndarray,
        waist_yaw_angle: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        根据 waist_yaw_Link 的世界坐标位姿及当前腰部关节角度 q_waist_yaw，反解出 base_link 的世界坐标位姿。

        正向链关系:
          T_world_waist = T_world_base * T_base_waist
          R_waist = R_base * R_offset * R_joint(q_waist_yaw)
          p_waist = p_base + R_base * p_waist_rel

        逆向解析解:
          R_base = R_waist * inv(R_joint(q_waist_yaw)) * inv(R_offset)
          p_base = p_waist - R_base * p_waist_rel
        """
        # 1. 构造 R_waist
        q_xyzw = np.array(waist_quat_wxyz, dtype=np.float64)[[1, 2, 3, 0]]
        norm = np.linalg.norm(q_xyzw)
        if norm > 1e-6:
            q_xyzw /= norm
        R_waist = Rotation.from_quat(q_xyzw)

        # 2. 构造腰关节旋转 R_joint (围绕 Z 轴)
        R_joint = Rotation.from_rotvec([0.0, 0.0, float(waist_yaw_angle)])

        # 3. 计算 base_link 的全局旋转
        R_base = R_waist * R_joint.inv() * self.rot_waist_offset.inv()
        q_base_wxyz = R_base.as_quat()[[3, 0, 1, 2]]

        # 4. 计算 base_link 的全局位置
        p_base = np.array(waist_pos, dtype=np.float64) - R_base.apply(self.p_waist_rel)

        return p_base, q_base_wxyz

    def apply_frame(
        self,
        data: mujoco.MjData,
        joint_pos: np.ndarray,
        anchor_pos: Optional[np.ndarray] = None,
        anchor_quat: Optional[np.ndarray] = None,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Apply joint angles and root pos/quat to data.qpos and update kinematics.
        Returns: (resolved_base_pos, resolved_base_quat_wxyz)
        """
        waist_yaw_angle = 0.0

        # 1. 设置 27 关节角度
        if joint_pos is not None:
            n = min(len(joint_pos), len(self.jnt_addrs))
            for i in range(n):
                addr = self.jnt_addrs[i]
                if addr != -1:
                    data.qpos[addr] = float(joint_pos[i])

            if len(joint_pos) > self.waist_jnt_idx:
                waist_yaw_angle = float(joint_pos[self.waist_jnt_idx])

            # 同步更新踝关节驱动连杆 (pitch/roll -> drive1/drive2)
            if len(joint_pos) >= 12:
                l_pitch, l_roll = float(joint_pos[4]), float(joint_pos[5])
                r_pitch, r_roll = float(joint_pos[10]), float(joint_pos[11])

                if self.drive_addrs.get("Left_drive1_joint") is not None:
                    data.qpos[self.drive_addrs["Left_drive1_joint"]] = l_pitch - l_roll
                if self.drive_addrs.get("Left_drive2_joint") is not None:
                    data.qpos[self.drive_addrs["Left_drive2_joint"]] = -l_pitch - l_roll
                if self.drive_addrs.get("Right_drive1_joint") is not None:
                    data.qpos[self.drive_addrs["Right_drive1_joint"]] = r_pitch - r_roll
                if self.drive_addrs.get("Right_drive2_joint") is not None:
                    data.qpos[self.drive_addrs["Right_drive2_joint"]] = -r_pitch - r_roll

        resolved_base_pos: Optional[np.ndarray] = None
        resolved_base_quat: Optional[np.ndarray] = None

        # 2. 设置 base_link 位姿 (若是 waist 模式则正运动学反解，若是 root/base 模式则直接赋值)
        if self.has_freejoint and anchor_pos is not None and anchor_quat is not None:
            if self.anchor_is_waist:
                resolved_base_pos, resolved_base_quat = self.solve_base_pose_from_waist(
                    anchor_pos, anchor_quat, waist_yaw_angle
                )
            else:
                # --root 模式: 直接使用传入的 base_link 位姿
                resolved_base_pos = np.array(anchor_pos, dtype=np.float64)
                resolved_base_quat = np.array(anchor_quat, dtype=np.float64)

            # 设置 base_link 位置
            if self.lock_origin:
                data.qpos[0] = 0.0
                data.qpos[1] = 0.0
                data.qpos[2] = float(resolved_base_pos[2])
            else:
                data.qpos[0] = float(resolved_base_pos[0])
                data.qpos[1] = float(resolved_base_pos[1])
                data.qpos[2] = float(resolved_base_pos[2])

            # 设置 base_link 姿态
            norm = np.linalg.norm(resolved_base_quat)
            if norm > 1e-4:
                q = resolved_base_quat / norm
                data.qpos[3] = q[0]
                data.qpos[4] = q[1]
                data.qpos[5] = q[2]
                data.qpos[6] = q[3]

        # 3. 执行前向运动学
        mujoco.mj_forward(self.model, data)

        return resolved_base_pos, resolved_base_quat


def parse_zmq_payload(
    payload_str: str, use_root: bool = False
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Parse JSON string from ZMQ into (joint_pos, anchor_pos, anchor_quat).
    When use_root=True, takes body_pos_w[0] and body_quat_w[0] directly (base_link).
    Otherwise, defaults to body_pos_w[13] and body_quat_w[13] (waist_yaw_Link).
    """
    try:
        data = json.loads(payload_str)
    except Exception:
        return None, None, None

    # 1. 关节角度 (支持 "joint_pos", "dof_pos", "qpos")
    joint_pos = None
    for k in ["joint_pos", "dof_pos", "qpos"]:
        if k in data:
            joint_pos = np.asarray(data[k], dtype=np.float32)
            break

    # 2. 刚体位置 (支持 "body_pos_w", "root_pos", "anchor_pos")
    anchor_pos = None
    for k in ["body_pos_w", "root_pos", "anchor_pos"]:
        if k in data:
            raw = np.asarray(data[k], dtype=np.float32)
            if raw.ndim == 2 and len(raw) > 0:
                # 当开启 --root 时直接取 body_pos_w[0] (base_link)
                # 否则取 body_pos_w[13] (waist_yaw_Link)
                idx = 0 if use_root else (13 if len(raw) > 13 else 0)
                anchor_pos = raw[idx]
                anchor_pos[2]+=0.05
            elif raw.ndim == 1:
                anchor_pos = raw[:3]
            break

    # 3. 刚体四元数 (支持 "body_quat_w", "root_quat", "anchor_quat")
    anchor_quat = None
    for k in ["body_quat_w", "root_quat", "anchor_quat"]:
        if k in data:
            raw = np.asarray(data[k], dtype=np.float32)
            if raw.ndim == 2 and len(raw) > 0:
                idx = 0 if use_root else (13 if len(raw) > 13 else 0)
                anchor_quat = raw[idx]
            elif raw.ndim == 1:
                anchor_quat = raw[:4]
            break

    return joint_pos, anchor_pos, anchor_quat


def run_offline_replay(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    adapter: RobotKinematicsAdapter,
    file_path: Path,
    loop: bool = True,
    speed: float = 1.0,
    use_root: bool = False,
) -> None:
    """Directly replay a .parquet or .npz motion file inside the viewer."""
    suffix = file_path.suffix.lower()
    print(f"[realtime_viewer] Loading file for offline playback: {file_path}")

    if suffix == ".parquet":
        import pyarrow.parquet as pq
        table = pq.read_table(str(file_path))
        actions = np.array(table["actions"].to_pylist(), dtype=np.float32)
        if "timestamp" in table.column_names:
            ts = np.array(table["timestamp"].to_pylist(), dtype=np.float64)
            dt = float(np.mean(np.diff(ts))) if len(ts) > 1 else 1.0 / 30.0
        else:
            dt = 1.0 / 30.0
        joint_pos_seq = actions[:, :27]
        pos_seq = actions[:, 33:36]
        quat_seq = actions[:, 36:40]
    elif suffix == ".npz":
        npz = np.load(file_path)
        joint_pos_seq = npz["joint_pos"]
        body_pos_w = npz["body_pos_w"]
        body_quat_w = npz["body_quat_w"]
        fps = float(npz.get("fps", np.array([50.0])).flat[0])
        dt = 1.0 / fps
        idx = 0 if use_root else (13 if body_pos_w.ndim == 3 and body_pos_w.shape[1] > 13 else 0)
        pos_seq = body_pos_w[:, idx] if body_pos_w.ndim == 3 else body_pos_w
        quat_seq = body_quat_w[:, idx] if body_quat_w.ndim == 3 else body_quat_w
    else:
        raise ValueError(f"Unsupported file {suffix}")

    total_frames = len(joint_pos_seq)
    print(f"[realtime_viewer] Loaded {total_frames} frames (dt={dt*1000:.1f}ms, speed={speed}x)")

    period = dt / max(speed, 1e-4)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = adapter.base_body_id

        frame = 0
        loop_cnt = 0
        while viewer.is_running():
            t_frame_start = time.monotonic()
            with viewer.lock():
                adapter.apply_frame(
                    data,
                    joint_pos=joint_pos_seq[frame],
                    anchor_pos=pos_seq[frame],
                    anchor_quat=quat_seq[frame],
                )
            viewer.sync()

            frame += 1
            if frame >= total_frames:
                if loop:
                    frame = 0
                    loop_cnt += 1
                else:
                    break

            elapsed = time.monotonic() - t_frame_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Real-time MuJoCo Kinematics Viewer for JAKA Robot (ZMQ Stream / File Replay)"
    )
    p.add_argument(
        "--connect",
        default="tcp://127.0.0.1:28701",
        help="ZMQ SUB connect address (default: tcp://127.0.0.1:28701, matching jakaminimimic_zmq)",
    )
    p.add_argument(
        "--root",
        action="store_true",
        help="开启此参数时，视 body_pos_w 和 body_quat_w 为真正的全身 28 个 link 位姿，"
             "直接提取 body_pos_w[0] 与 body_quat_w[0] 赋给 base_link (data.qpos[0:7])，"
             "不进行腰部到基座的正向逆解算。",
    )
    p.add_argument(
        "--xml",
        type=Path,
        default=None,
        help="Path to Khan_mini_sym.xml (default: auto-locate in repo)",
    )
    p.add_argument(
        "--anchor-body",
        choices=["waist", "base"],
        default="waist",
        help="Body that anchor_pos represents when --root is not set: 'waist' (default) or 'base'",
    )
    p.add_argument(
        "--file",
        type=Path,
        default=None,
        help="Optional: offline playback from .parquet or .npz directly",
    )
    p.add_argument(
        "--loop",
        action="store_true",
        help="Loop offline file playback indefinitely",
    )
    p.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier for offline mode (default: 1.0)",
    )
    p.add_argument(
        "--lock-origin",
        action="store_true",
        help="Lock robot base X/Y to (0, 0) in world grid to observe motion in-place",
    )
    p.add_argument(
        "--free-cam",
        action="store_true",
        help="Do not automatically track the robot base with the camera",
    )
    p.add_argument(
        "--fps",
        type=float,
        default=60.0,
        help="Target viewer sync rate in Hz (default: 60.0)",
    )
    return p


def main() -> None:
    args = build_argparser().parse_args()

    # 1. 加载 MuJoCo 模型
    xml_path = args.xml if args.xml is not None else find_default_xml()
    if not xml_path.exists():
        print(f"[realtime_viewer] ERROR: XML file not found at {xml_path}")
        sys.exit(1)

    print(f"[realtime_viewer] Loading MuJoCo model from: {xml_path}")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    # 初始化到站立姿态
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    # 确定是否启用 root 模式
    use_root = args.root or (args.anchor_body == "base")
    anchor_is_waist = not use_root

    adapter = RobotKinematicsAdapter(
        model, lock_origin=args.lock_origin, anchor_is_waist=anchor_is_waist
    )

    # 2. 如果指定了离线文件，进入离线回放模式
    if args.file is not None:
        run_offline_replay(
            model, data, adapter, args.file, loop=args.loop, speed=args.speed, use_root=use_root
        )
        return

    # 3. 否则进入 ZMQ 实时接收模式
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.CONFLATE, 1)  # 保持最新一帧，无延迟排队
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.connect(args.connect)

    print(f"[realtime_viewer] Connected to ZMQ: {args.connect}")
    if use_root:
        print(
            "[realtime_viewer] Mode: [--root ENABLED] -> Directly extracting body_pos_w[0] and body_quat_w[0] "
            "as base_link pose (data.qpos[0:7])"
        )
    else:
        print("[realtime_viewer] Mode: [WAIST ANCHOR] -> Solving base_link pose from waist_yaw_Link kinematics")
    if args.lock_origin:
        print("[realtime_viewer] Base X/Y locked to (0,0) (in-place view)")
    print("[realtime_viewer] Launching interactive viewer... (Press ESC in window to quit)")

    # 统计信息
    recv_packets = 0
    t_last_stat = time.monotonic()
    last_waist_pos: Optional[np.ndarray] = None
    last_base_pos: Optional[np.ndarray] = None
    last_base_quat: Optional[np.ndarray] = None

    period_s = 1.0 / max(args.fps, 10.0)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        if not args.free_cam and adapter.base_body_id != -1:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = adapter.base_body_id

        print("\n[realtime_viewer] >>> Viewer ready! Waiting for ZMQ packets... <<<\n")

        while viewer.is_running():
            t_loop_start = time.monotonic()

            # 非阻塞获取最新一帧 (CONFLATE=1 保证每次拿到的都是最新数据)
            has_new = False
            while True:
                try:
                    raw_msg = sock.recv_string(flags=zmq.NOBLOCK)
                    j_pos, r_pos, r_quat = parse_zmq_payload(raw_msg, use_root=use_root)
                    if j_pos is not None:
                        has_new = True
                        recv_packets += 1
                        last_waist_pos = r_pos
                        with viewer.lock():
                            b_pos, b_quat = adapter.apply_frame(
                                data, joint_pos=j_pos, anchor_pos=r_pos, anchor_quat=r_quat
                            )
                        last_base_pos = b_pos
                        last_base_quat = b_quat
                except zmq.Again:
                    break

            if has_new:
                viewer.sync()

            # 定时在终端打印状态 (每秒 1 次)
            now = time.monotonic()
            if now - t_last_stat >= 1.0:
                hz = recv_packets / (now - t_last_stat)
                recv_packets = 0
                t_last_stat = now

                if use_root:
                    base_str = (
                        f"base_link=[{last_base_pos[0]:.2f}, {last_base_pos[1]:.2f}, {last_base_pos[2]:.2f}]"
                        if last_base_pos is not None
                        else "base_link=N/A"
                    )
                    rpy_str = ""
                    if last_base_quat is not None:
                        r, p, y = quat_to_euler_rpy(last_base_quat)
                        rpy_str = f" rpy=[{r:.1f}°, {p:.1f}°, {y:.1f}°]"

                    print(
                        f"\r[realtime_viewer] Recv: {hz:4.1f} Hz | (root mode) {base_str}{rpy_str} | "
                        f"Status: {'Active' if hz > 0.5 else 'Waiting...'}",
                        end="",
                        flush=True,
                    )
                else:
                    waist_str = (
                        f"waist=[{last_waist_pos[0]:.2f}, {last_waist_pos[1]:.2f}, {last_waist_pos[2]:.2f}]"
                        if last_waist_pos is not None
                        else "waist=N/A"
                    )
                    base_str = (
                        f"base=[{last_base_pos[0]:.2f}, {last_base_pos[1]:.2f}, {last_base_pos[2]:.2f}]"
                        if last_base_pos is not None
                        else "base=N/A"
                    )
                    rpy_str = ""
                    if last_base_quat is not None:
                        r, p, y = quat_to_euler_rpy(last_base_quat)
                        rpy_str = f" rpy=[{r:.1f}°, {p:.1f}°, {y:.1f}°]"

                    print(
                        f"\r[realtime_viewer] Recv: {hz:4.1f} Hz | {waist_str} -> {base_str}{rpy_str} | "
                        f"Status: {'Active' if hz > 0.5 else 'Waiting...'}",
                        end="",
                        flush=True,
                    )

            # 控制渲染刷新频率
            elapsed = time.monotonic() - t_loop_start
            sleep_time = period_s - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    sock.close(0)
    print("\n[realtime_viewer] Viewer closed cleanly.")


if __name__ == "__main__":
    main()
