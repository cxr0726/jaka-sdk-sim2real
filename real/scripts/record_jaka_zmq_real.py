"""Standalone OpenHLM-style data record script for the REAL Jaka robot.

Runs from the KpiDeployReal checkout (this directory), independent of the SIMPLE
repo. All record helpers live in the sibling module ``jaka_record_lib.py``
(self-contained port of the SIMPLE-side helpers; no ``simple.*`` imports).

The real-robot provider is the C++ ``StatePublisher`` inside the 50 Hz low-level
controller on the robot (``src/jaka-humanoid-sdk/real``): it publishes the body
state (27 joints in MuJoCo/JAKA order + base quat) over ZMQ PUB
``tcp://<robot_ip>:28711``, inline & non-blocking every control tick. The pico
retarget hub keeps publishing the reference motion on the PC (``:28701``). The
head camera (``:28712``) is NOT wired on the real robot yet — use ``--no-camera``
to record state+action with a black placeholder image (format stays complete;
re-record with the real camera once the onboard publisher exists).

All streams keep only the newest frame (``zmq.CONFLATE=1``). State + motion are
polled inline in the record loop (OpenHLM body-state style); the camera runs a
dedicated receive thread with a lock-protected latest-frame cache (OpenHLM
``HeadZMQClient`` style), so later consumers (e.g. a live display) can share
the same camera cache without touching the record loop.

Usage (real robot, from the recording PC)::

    python scripts/record_jaka_zmq_real.py  --save-dir data/teleop_jaka_mf_real --desc "put the bottle on the mouse pad"  --state-zmq-connect tcp://<robot_ip>:28711  --no-camera

Runtime env requirements (pip packages): numpy, scipy, pyzmq, typer, PyYAML,
lerobot (lazy-imported on dataset creation). No mujoco/simple/decoupled_wbc.

Trigger: pico A-button ``toggle_data_collection`` on :28701 and/or keyboard ENTER.
Records at ``--frequency`` (default 30 Hz), latest-frame-of-each-stream, openhlm
format (state 30 / action 40 / head image).
"""
##python scripts/record_jaka_zmq_real.py  --save-dir data/teleop_jaka_mf_real --desc "put the bottle on the mouse pad"  --motion_zmq_connect tcp://10.5.5.120:28701  --no-camera

from __future__ import annotations

import json
import os
import struct
import sys
import threading
import time
from typing import Optional

import numpy as np
import typer
import zmq
from typing_extensions import Annotated

from jaka_record_lib import (
    JAKA_BODY_NAMES,
    JAKA_JOINT_NAMES,
    BODY_POS_W_KEY,
    BODY_QUAT_W_KEY,
    JOINT_POS_KEY,
    OPENHLM_ACTION_DIM,
    OPENHLM_STATE_DIM,
    PUBLISH_T_NS_KEY,
    SMPLX_T_NS_KEY,
    TOGGLE_DATA_COLLECTION_KEY,
    VIDEO_SHAPE,
    LatestMotionFrame,
    OpenHLMRootVel,
    build_openhlm_frame,
    finalize_openhlm_episode,
    init_openhlm_exporter,
    reference_action_live_latest,
)

_DEFAULT_POLICY_CONFIG = "scripts/pico_record.yaml"


# ---------------------------------------------------------------------------
# Latest-only clients: state/motion poll inline in the 30 Hz record loop
# (CONFLATE=1); the image client runs a dedicated receive thread with a
# lock-protected cache (OpenHLM HeadZMQClient style) so future consumers
# (live display / visualizer) can share the same camera cache.
# ---------------------------------------------------------------------------
class RealStateClient:
    """Latest-only SUB to the C++ StatePublisher (:28711).

    Payload (JSON, from StatePublisher.cpp): ``publish_t_ns`` / ``smplx_t_ns`` /
    ``paused`` / ``seq`` / ``joint_pos[27]`` (MuJoCo/JAKA order) /
    ``body_pos_w[28,3]`` / ``body_quat_w[28,4]`` (only [0] = base quat meaningful).
    """

    def __init__(self, connect: str):
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.CONFLATE, 1)  # socket 层只保留最新一条
        self._sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._sock.connect(connect)
        self._joint_pos: Optional[np.ndarray] = None
        self._base_quat_wxyz: Optional[np.ndarray] = None
        self._ts_ns: Optional[int] = None

    def poll(self) -> None:
        """取一次最新帧(无新帧则保留缓存)。主循环 30Hz 调用。"""
        try:
            raw = self._sock.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            return
        try:
            payload = json.loads(raw.decode("utf-8"))
        except Exception:
            return
        joint_pos = np.asarray(payload.get(JOINT_POS_KEY), dtype=np.float32)
        body_quat_w = np.asarray(payload.get(BODY_QUAT_W_KEY), dtype=np.float32)
        # 27 关节 + 二维 body 数组(与 SIMPLE 版 LatestStateClient 相同校验)
        if joint_pos.shape[0] != 27 or body_quat_w.ndim != 2 or body_quat_w.shape[0] == 0:
            return
        self._joint_pos = joint_pos.copy()
        self._base_quat_wxyz = body_quat_w[0].copy()  # base_link 四元数 wxyz
        self._ts_ns = int(payload.get(PUBLISH_T_NS_KEY, time.time_ns()))

    def latest(self):
        return self._joint_pos, self._base_quat_wxyz, self._ts_ns

    def close(self) -> None:
        try:
            self._sock.close(0)
        except Exception:
            pass


class RealImageClient:
    """Latest-only SUB to the head-camera stream (:28712) — OpenHLM style.

    Mirrors ``HeadZMQClient`` (gear_sonic/data_utils/vision_client.py): a
    dedicated daemon thread runs a tight ``recv(NOBLOCK)`` loop (CONFLATE=1,
    socket-level latest-only) and decodes into a lock-protected cache;
    consumers (recorder now, possibly a live display / visualizer later) just
    call :meth:`latest` from their own thread at any moment — the network I/O
    and decoding stay off the record loop, and the cache supports multiple
    readers.

    Wire format matches ``HeadZMQClient._decode_message``: ``[w,h,c]`` int32
    header + RGB payload (c ∈ {3,4}).
    """

    def __init__(self, connect: str):
        self._connect = connect
        self._lock = threading.Lock()
        self._latest_image: Optional[np.ndarray] = None
        self._ts_ms: Optional[int] = None
        self._running = True
        self._thread = threading.Thread(target=self._receive_process, daemon=True)
        self._thread.start()

    def _decode_message(self, msg: bytes) -> Optional[np.ndarray]:
        if len(msg) < 12:
            return None
        w, h, c = struct.unpack("iii", msg[:12])
        payload = msg[12:]
        if len(payload) != w * h * c or c not in (3, 4):
            return None
        img = np.frombuffer(payload, dtype=np.uint8).reshape((h, w, c))
        if c == 4:  # BGRA -> BGR
            img = img[..., :3]
        return img

    def _set_latest(self, image: np.ndarray) -> None:
        with self._lock:
            self._latest_image = image.copy()
            self._ts_ms = int(time.time() * 1000)

    def _receive_process(self) -> None:
        # 线程专属 context/socket(与 HeadZMQClient 一致): socket 只在接收线程内使用
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.CONFLATE, 1)  # 只保留最新一条
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.connect(self._connect)
        try:
            while self._running:
                try:
                    msg = sock.recv(flags=zmq.NOBLOCK)
                except zmq.Again:
                    time.sleep(0.001)
                    continue
                image = self._decode_message(msg)
                if image is not None:
                    self._set_latest(image)
        finally:
            sock.close()
            ctx.term()

    def latest(self) -> Optional[np.ndarray]:
        """最新一帧(拷贝, 线程安全; 无数据返回 None)。任意线程可调。"""
        with self._lock:
            return None if self._latest_image is None else self._latest_image.copy()

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)


class RealMotionClient:
    """Latest-only SUB to the pico retarget motion stream (:28701).

    Socket-level CONFLATE (no drain thread). Keeps the previous frame so
    ``reference_action_live_latest`` can finite-difference the anchor linear
    velocity, and exposes the pico record-control level.
    """

    def __init__(self, connect: str, joint_names, body_names):
        self.joint_names = list(joint_names)
        self.body_names = list(body_names)
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._sock.connect(connect)
        self._joint_pos: Optional[np.ndarray] = None
        self._body_pos_w: Optional[np.ndarray] = None
        self._body_quat_w: Optional[np.ndarray] = None
        self._ts: Optional[int] = None
        self._prev_body_pos_w: Optional[np.ndarray] = None
        self._prev_ts: Optional[int] = None
        self._toggle_data_collection: Optional[bool] = None

    def poll(self) -> None:
        try:
            raw = self._sock.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            return
        try:
            payload = json.loads(raw.decode("utf-8"))
        except Exception:
            return
        joint_pos = payload.get(JOINT_POS_KEY)
        body_pos_w = payload.get(BODY_POS_W_KEY)
        body_quat_w = payload.get(BODY_QUAT_W_KEY)
        if joint_pos is None or body_pos_w is None or body_quat_w is None:
            return
        ts = int(
            payload.get(PUBLISH_T_NS_KEY) or payload.get(SMPLX_T_NS_KEY) or time.time_ns()
        )
        toggle = payload.get(TOGGLE_DATA_COLLECTION_KEY)
        if isinstance(toggle, (list, tuple, np.ndarray)):
            toggle = toggle[0] if len(toggle) else None
        # 只保留最新帧; 上一帧留着算参考线速度(差分)
        self._prev_body_pos_w = self._body_pos_w
        self._prev_ts = self._ts
        self._joint_pos = np.asarray(joint_pos, dtype=np.float32)
        self._body_pos_w = np.asarray(body_pos_w, dtype=np.float32)
        self._body_quat_w = np.asarray(body_quat_w, dtype=np.float32)
        self._ts = ts
        if toggle is not None:
            self._toggle_data_collection = bool(toggle)

    @property
    def latest_toggle_data_collection(self) -> Optional[bool]:
        return self._toggle_data_collection

    def get_latest_frame(self):
        """最新一帧(无则 None), 字段与 ``RealtimeMotionBuffer.get_latest_frame`` 一致。"""
        if self._joint_pos is None or self._body_quat_w is None:
            return None
        return LatestMotionFrame(
            joint_pos=self._joint_pos,
            body_pos_w=self._body_pos_w,
            body_quat_w=self._body_quat_w,
            timestamp_ns=self._ts,
            prev_body_pos_w=self._prev_body_pos_w,
            prev_timestamp_ns=self._prev_ts,
        )

    def close(self) -> None:
        try:
            self._sock.close(0)
        except Exception:
            pass


class MotionToggleTrigger:
    """pico ``toggle_data_collection`` 电平触发器(与 SIMPLE 版同语义)。

    跟随电平而非单帧边沿: 丢一帧也不会失步。
    """

    def __init__(self, motion_client):
        self._client = motion_client
        self._level: Optional[bool] = None

    def poll(self) -> Optional[bool]:
        level = self._client.latest_toggle_data_collection
        if level is not None:
            self._level = level
        return self._level


def _load_policy_motion_config(policy_config: str) -> dict:
    import yaml

    with open(policy_config) as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
    motion = dict(cfg.get("motion", {}))
    return {
        "joint_names": list(cfg.get("joint_names_simulation", None)),
        "body_names": list(cfg.get("body_names_simulation", None)),
        "future_steps": np.asarray(motion.get("future_steps", [0]), dtype=int),
        "motion_zmq_connect": motion.get("motion_zmq_connect", "tcp://127.0.0.1:28701"),
        "anchor_body_name": motion.get("anchor_body_name", "waist_yaw_Link"),
    }


def _resolve_run_dir(save_dir: str, dr_level: int, run_name: str = "") -> str:
    run_id = run_name or time.strftime("%Y%m%d-%H%M%S")
    root = os.path.abspath(save_dir)
    run_save_dir = f"{root}/{run_id}/level-{dr_level}"
    suffix = 2
    while os.path.exists(run_save_dir):
        run_save_dir = f"{root}/{run_id}-{suffix}/level-{dr_level}"
        suffix += 1
    return run_save_dir


def main(
    # dataset
    save_dir: Annotated[str, typer.Option()] = "data/teleop_jaka_mf_real",
    run_name: Annotated[str, typer.Option(help="Leaf dir under --save-dir; default: timestamp")] = "",
    desc: Annotated[str, typer.Option()] = "close the trash can",
    num_episodes: Annotated[int, typer.Option()] = 100,
    frequency: Annotated[int, typer.Option()] = 30,
    dr_level: Annotated[int, typer.Option()] = 0,
    # ZMQ connects (state 来自实机 C++ StatePublisher, 需传机器人 IP)
    state_zmq_connect: Annotated[str, typer.Option()] = "tcp://10.5.5.100:28711",
    camera_zmq_connect: Annotated[str, typer.Option()] = "tcp://10.5.5.100:28712",
    motion_zmq_connect: Annotated[str, typer.Option()] = "tcp://127.0.0.1:28701",
    # policy (mirrors the control-side motion buffer so the action label matches)
    policy_config: Annotated[str, typer.Option()] = _DEFAULT_POLICY_CONFIG,
    # trigger: keyboard (Enter) | pico (A-button toggle_data_collection) | both
    trigger: Annotated[str, typer.Option()] = "both",
    # 实机相机(:28712)尚未接入: 用黑帧占位录制 state+action(格式完整, 仅链路验证;
    # 正式 VLA 数据等机载相机进程就绪后去掉此开关重录)
    no_camera: Annotated[bool, typer.Option()] = False,
):
    assert trigger in ("keyboard", "pico", "both"), f"Invalid trigger {trigger}"
    use_keyboard = trigger in ("keyboard", "both")
    use_pico = trigger in ("pico", "both")

    motion_cfg = _load_policy_motion_config(policy_config)

    print(f"[record-real] Connecting state→{state_zmq_connect} "
          f"motion→{motion_zmq_connect}"
          + ("" if no_camera else f" camera→{camera_zmq_connect}")
          + (f" [--no-camera: black placeholder {tuple(VIDEO_SHAPE)}]" if no_camera else ""))
    state_client = RealStateClient(state_zmq_connect)
    image_client = None if no_camera else RealImageClient(camera_zmq_connect)
    motion_client = RealMotionClient(
        connect=motion_zmq_connect,
        joint_names=motion_cfg["joint_names"],
        body_names=motion_cfg["body_names"],
    )
    pico_toggle = MotionToggleTrigger(motion_client)

    run_save_dir = _resolve_run_dir(save_dir, dr_level, run_name)

    exporter = None
    head_shape: tuple[int, ...] | None = None
    if no_camera:
        print(f"[record-real] Dataset will be created at {run_save_dir} with "
              f"black placeholder image {tuple(VIDEO_SHAPE)}")
    else:
        print(f"[record-real] New dataset will be created at {run_save_dir} on the "
              f"first camera frame (--run-name picks the subdir; existing datasets "
              f"are never touched)")

    state_root_calc = OpenHLMRootVel()   # (roll,pitch,yaw_vel) — state 30 / action 共用
    action_root_calc = OpenHLMRootVel()

    # Keyboard trigger: daemon thread reads stdin (Enter=record toggle, q=quit).
    rec_flags = {"toggle": False, "quit": False, "pico_level": None}

    def _stdin_listener():
        try:
            for line in sys.stdin:
                k = line.strip().lower()
                if k in ("q", "quit", "exit"):
                    rec_flags["quit"] = True
                    return
                if k == "" or k in ("s", "start", "r", "record"):
                    rec_flags["toggle"] = True
        except Exception:
            pass

    stdin_thread = None
    if use_keyboard:
        stdin_thread = threading.Thread(target=_stdin_listener, daemon=True)
        stdin_thread.start()
        print("[record-real] Keyboard: ENTER=record toggle, 'q'=quit")
    if use_pico:
        print("[record-real] Pico: A button (toggle_data_collection on the motion stream) starts/stops")

    recording = False
    frames_in_episode = 0
    episodes_saved = 0
    control_dt = 1.0 / frequency
    step = 0

    print("[record-real] Waiting for data. Press ENTER "
          + ("/A on pico " if use_pico else "")
          + "to start recording.\n")
    try:
        while not rec_flags["quit"]:
            start = time.time()

            # state/motion: 每轮各取一次最新帧(无新帧保留缓存);
            # image: 专用接收线程持续更新缓存(见 RealImageClient), 这里只读 latest()
            state_client.poll()
            motion_client.poll()

            desired: Optional[bool] = None
            if use_keyboard and rec_flags["toggle"]:
                rec_flags["toggle"] = False
                desired = not recording
            if use_pico:
                level = pico_toggle.poll()
                if level is not None:
                    if rec_flags["pico_level"] is None:
                        rec_flags["pico_level"] = level
                        desired = level
                        print(f"[record-real] pico toggle_data_collection = {level}")
                    elif level != rec_flags["pico_level"]:
                        rec_flags["pico_level"] = level
                        desired = level
                        print(f"[record-real] pico toggle_data_collection -> {level}")

            if desired is not None and desired != recording:
                recording = desired
                if recording:
                    state_root_calc.reset()
                    action_root_calc.reset()
                    frames_in_episode = 0
                    print(f"[record-real] EPISODE {episodes_saved}: RECORDING started")
                else:
                    if exporter is not None and frames_in_episode > 0:
                        exporter.save_episode()
                        try:
                            video_path = finalize_openhlm_episode(exporter)
                        except Exception as exc:
                            video_path = None
                            print(f"[record-real] WARN: mp4 encode failed ({exc}); "
                                  f"episode parquet is still valid")
                        episodes_saved += 1
                        print(f"[record-real] Episode {episodes_saved} saved"
                              + (f" (video {video_path})" if video_path else ""))
                        if episodes_saved >= num_episodes:
                            break
                    else:
                        print("[record-real] Stop toggled with no frames — episode skipped")

            if recording:
                joint_pos, base_quat_wxyz, _ts = state_client.latest()
                image = None if no_camera else image_client.latest()
                if joint_pos is None:
                    time.sleep(control_dt)
                    continue
                if not no_camera and image is None:
                    time.sleep(control_dt)
                    continue

                # 相机缺失时用黑帧占位, shape 取 VIDEO_SHAPE
                shape = (
                    tuple(int(v) for v in np.asarray(image).shape[:3])
                    if image is not None
                    else tuple(int(v) for v in VIDEO_SHAPE)
                )
                if exporter is None:
                    head_shape = shape
                    exporter = init_openhlm_exporter(
                        run_save_dir, fps=frequency, head_image_shape=head_shape
                    )
                    print(f"[record-real] Dataset created at {run_save_dir} "
                          f"(head_image_left {head_shape}, state {OPENHLM_STATE_DIM}, "
                          f"actions {OPENHLM_ACTION_DIM}, fps {frequency})")
                elif shape != head_shape:
                    print(f"[record-real] head image shape changed {head_shape} -> {shape}; "
                          f"skipping frame (restart the recorder if the camera cfg changed)")
                    time.sleep(control_dt)
                    continue

                t_ns = int(time.time() * 1e9)
                state = np.concatenate([
                    np.asarray(joint_pos, dtype=np.float32),
                    state_root_calc(base_quat_wxyz, t_ns),
                ])

                ref_joints, ref_anchor_quat, ref_anchor_lin_vel, ref_anchor_pos_w = (
                    reference_action_live_latest(
                        motion_client, anchor_name=motion_cfg["anchor_body_name"]
                    )
                )
                # action(40) = [27 dof | roll,pitch,yaw_vel | lin_vel xyz |
                #               anchor_pos_w xyz | anchor_quat_w wxyz]
                actions = np.concatenate([
                    ref_joints,
                    action_root_calc(ref_anchor_quat, t_ns),
                    ref_anchor_lin_vel,
                    ref_anchor_pos_w,
                    ref_anchor_quat,
                ])

                frame_image = (
                    image
                    if image is not None
                    else np.zeros(shape, dtype=np.uint8)  # 黑帧占位
                )
                exporter.add_frame(
                    build_openhlm_frame(frame_image, state, actions), task=desc
                )
                frames_in_episode += 1
                step += 1
                if step % (frequency * 5) == 0:
                    print(f"[record-real] step {step} saved "
                          f"(state{state.dtype} action{actions.shape})")

            elapsed = time.time() - start
            if elapsed < control_dt:
                time.sleep(control_dt - elapsed)
    except KeyboardInterrupt:
        print("\n[record-real] Interrupted by user.")
    finally:
        if exporter is not None and recording and frames_in_episode > 0:
            try:
                exporter.save_episode()
                episodes_saved += 1
            except Exception:
                pass
            try:
                finalize_openhlm_episode(exporter)
            except Exception:
                pass
        print(f"[record-real] Done. {episodes_saved} episodes saved")
        state_client.close()
        if image_client is not None:
            image_client.close()
        motion_client.close()


if __name__ == "__main__":
    typer.run(main)
