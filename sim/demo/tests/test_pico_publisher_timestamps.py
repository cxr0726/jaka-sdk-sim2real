#!/usr/bin/env python3
"""Execute publisher methods with fake XR/GMR/clocks; no hardware imports or sockets.

The AST loader skips only dependency imports and the main guard. The production
class/functions themselves are compiled unchanged, so timestamp placement,
paused-mode branches and send-loop sequence handling are exercised directly.
"""
from __future__ import annotations

import ast
from contextlib import redirect_stdout
from dataclasses import dataclass
import io
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
import uuid

import numpy as np


DEMO = Path(__file__).resolve().parents[1]
SOURCE = DEMO / "scripts/pico_retarget_pub.py"
ACTUAL_SOURCE = DEMO / "scripts/pico_pub_copy.py"
# Canonical wire keys, independent of an installed sim2real/GMR environment.
WIRE_KEYS = {
    "BODY_POS_W_KEY": "body_pos_w", "BODY_QUAT_W_KEY": "body_quat_w",
    "JOINT_POS_KEY": "joint_pos", "PUBLISH_T_NS_KEY": "publish_t_ns",
    "SEQ_KEY": "seq", "SMPLX_T_NS_KEY": "smplx_t_ns",
    "XROBOT_BODY_NAMES_KEY": "xrobot_body_names",
    "XROBOT_BODY_POS_W_KEY": "xrobot_body_pos_w",
    "XROBOT_BODY_QUAT_W_KEY": "xrobot_body_quat_w",
}
MS = 1_000_000


class FakeClock:
    def __init__(self):
        self.wall_ns = 1_790_000_000_000_000_000
        self.mono_ns = 120_000_000_000
        self.events = []

    def advance(self, ns):
        self.wall_ns += int(ns)
        self.mono_ns += int(ns)

    def time_ns(self):
        self.events.append(("wall_stamp", self.mono_ns))
        return self.wall_ns

    def monotonic_ns(self):
        self.events.append(("mono_stamp", self.mono_ns))
        return self.mono_ns

    def monotonic(self):
        return self.mono_ns / 1.e9

    perf_counter = monotonic

    def sleep(self, seconds):
        self.advance(round(seconds * 1.e9))


class DummyTimer:
    _instances = {}

    def __init__(self, name):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


class FakeAgain(Exception):
    pass


class FakeSocket:
    def __init__(self, outcomes=()):
        self.outcomes = list(outcomes)
        self.attempts = []
        self.closed = False

    def setsockopt(self, *args):
        pass

    def bind(self, *args):
        pass

    def send_string(self, text, flags=0):
        self.attempts.append(json.loads(text))
        outcome = self.outcomes.pop(0) if self.outcomes else None
        if outcome is not None:
            raise outcome

    def close(self, *args):
        self.closed = True


class FakeContext:
    def __init__(self):
        self.sockets = []
        self.next_outcomes = []

    def socket(self, *args):
        result = FakeSocket(self.next_outcomes)
        self.next_outcomes = []
        self.sockets.append(result)
        return result


class FakeStreamer:
    body_joint_names = ("Pelvis",)

    def __init__(self, clock):
        self.clock = clock
        self.read_calls = 0
        self.fail = False

    def get_raw_body_data(self):
        self.read_calls += 1
        self.clock.advance(2 * MS)
        self.clock.events.append(("raw_return", self.clock.mono_ns))
        if self.fail:
            return None, None, None, None, 42
        return np.asarray([[.1, .2, .8, 0, 0, 0, 1]], np.float32), None, None, None, 42

    def coordinate_transform_unity_data(self, body):
        self.clock.events.append(("coordinate_transform", self.clock.mono_ns))
        self.clock.advance(MS)
        return body


class FakeRetarget:
    robot_root_name = "pelvis"

    def __init__(self, clock):
        self.clock = clock
        self.cost_ns = 2 * MS
        self.scaled_human_data = {}
        positions = np.zeros((14, 3), np.float32)
        positions[:, 2] = .7
        quats = np.tile(np.asarray([1, 0, 0, 0], np.float32), (14, 1))
        qpos = np.zeros(34, np.float32)
        qpos[3] = 1
        self.configuration = types.SimpleNamespace(
            model=types.SimpleNamespace(nq=34),
            data=types.SimpleNamespace(xpos=positions, xquat=quats, qpos=qpos),
        )

    def retarget(self, body, **kwargs):
        self.clock.events.append(("gmr_begin", self.clock.mono_ns))
        self.clock.advance(self.cost_ns)
        self.scaled_human_data = {k: [np.asarray(v[0]).copy(), np.asarray(v[1]).copy()] for k, v in body.items()}

    update_targets = retarget


def load_production(clock, context, source=SOURCE):
    source = Path(source)
    tree = ast.parse(source.read_text(), filename=str(source))
    selected = [ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)]
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and getattr(node, "name", None) != "PublisherArgs":
            selected.append(node)
        elif isinstance(node, ast.Assign):
            try:
                ast.literal_eval(node.value)
            except (ValueError, TypeError):
                continue
            selected.append(node)
    module = types.ModuleType("pico_timestamp_test_" + uuid.uuid4().hex)
    sys.modules[module.__name__] = module
    env = module.__dict__
    env.update(np=np, json=json, time=clock, uuid=uuid, dataclass=dataclass,
               ScopedTimer=DummyTimer, PORTS={"pico_controller": 28712})
    env.update(WIRE_KEYS)
    env["zmq"] = types.SimpleNamespace(
        Context=types.SimpleNamespace(instance=lambda: context),
        PUB=1, LINGER=2, SNDHWM=3, CONFLATE=4, NOBLOCK=5, Again=FakeAgain,
    )
    env["get_robot_cfg"] = lambda robot: types.SimpleNamespace(
        qpos_size=34, default_qpos=np.r_[np.zeros(3), [1, 0, 0, 0], np.zeros(27)],
        body_names=("pelvis",) + tuple("body" + str(i) for i in range(13)),
        joint_names=tuple("joint" + str(i) for i in range(27)),
        mjcf_path="test-only.xml", resolve_mjcf_path=lambda: "test-only.xml",
    )
    env["RateLimiter"] = lambda **kwargs: types.SimpleNamespace(sleep=lambda: None)
    env["XRobotStreamer"] = lambda: FakeStreamer(clock)
    env["GMR"] = lambda **kwargs: FakeRetarget(clock)
    exec(compile(ast.fix_missing_locations(ast.Module(body=selected, type_ignores=[])), str(source), "exec"), env)
    return module


class PublisherTimestampTests(unittest.TestCase):
    source = SOURCE
    robot = "g1"

    def setUp(self):
        self.clock = FakeClock()
        self.context = FakeContext()
        self.module = load_production(self.clock, self.context, self.source)
        self.env = self.module.__dict__
        self.cls = self.env["LiveRetargetPublisher"]
        self.args = types.SimpleNamespace(
            robot=self.robot, publish_hz=50., actual_human_height=1.6, verbose=False,
            controller_hwm=1, controller_bind="fake-controller", bind="fake-motion", hwm=1,
            startup_sleep_s=0., skip_retarget=False,
        )
        self.addCleanup(sys.modules.pop, self.module.__name__, None)

    def make_worker(self):
        arrays = (np.zeros(27, np.float32), np.zeros((14, 3), np.float32),
                  np.tile(np.asarray([1, 0, 0, 0], np.float32), (14, 1)))
        with patch.object(self.cls, "_resolve_joint_qpos_indices", return_value=list(range(7, 34))), \
             patch.object(self.cls, "_resolve_body_ids", return_value=list(range(14))), \
             patch.object(self.cls, "_resolve_human_to_robot_body_name", return_value={}), \
             patch.object(self.cls, "_pose_arrays_from_qpos", return_value=arrays):
            worker = self.cls(self.args)
        worker._poll_pause_toggle = lambda: False
        worker._maybe_print_mode_hint = lambda: None
        worker._transform_live_body_pose_dict = lambda body: body
        worker._apply_min_link_height_offset = lambda body, **kwargs: body
        worker._maybe_print_late_frame_breakdown = lambda elapsed: None
        worker.paused = False
        return worker

    def test_pre_gmr_sampling_and_post_gmr_publication(self):
        worker = self.make_worker()
        initial_wall, initial_mono = self.clock.wall_ns, self.clock.mono_ns
        payload = worker.sample_and_retarget()
        self.assertNotIn("publish_t_ns", payload)
        self.assertEqual(payload["motion_t_ns"], initial_mono + 2 * MS)
        self.assertEqual(payload["pico_recv_monotonic_ns"], initial_mono + 2 * MS)
        self.assertEqual(payload["pico_recv_time_ns"], initial_wall + 2 * MS)
        self.assertEqual(payload["motion_source"], "xr_read")
        self.assertEqual(payload["source_time_field"], "motion_t_ns")
        self.assertEqual(payload["source_clock_id"], "monotonic:" + payload["stream_id"])
        self.assertLessEqual(payload["motion_t_ns"], next(t for name, t in self.clock.events if name == "coordinate_transform"))
        self.assertLess(payload["motion_t_ns"], next(t for name, t in self.clock.events if name == "gmr_begin"))
        worker._stamp_publication(payload)
        self.assertEqual(payload["publish_monotonic_ns"], initial_mono + 5 * MS)
        self.assertEqual(payload["publish_t_ns"], initial_wall + 5 * MS)
        self.assertAlmostEqual(payload["sample_to_publish_ms"], 3.)
        self.assertEqual(len(payload["joint_pos"]), 27)
        self.assertEqual(len(payload["body_pos_w"]), 14)

    def test_variable_gmr_cost_does_not_change_sampling_intervals(self):
        worker = self.make_worker()
        initial = self.clock.mono_ns
        payloads = []
        for i, cost in enumerate([2, 8, 4]):
            self.clock.advance(initial + i * 20 * MS - self.clock.mono_ns)
            worker.retarget.cost_ns = cost * MS
            payload = worker.sample_and_retarget()
            worker._stamp_publication(payload)
            payloads.append(payload)
        samples = [p["motion_t_ns"] for p in payloads]
        publishes = [p["publish_monotonic_ns"] for p in payloads]
        self.assertEqual(np.diff(samples).tolist(), [20 * MS, 20 * MS])
        self.assertEqual(np.diff(publishes).tolist(), [26 * MS, 16 * MS])
        self.assertEqual([p["sample_to_publish_ms"] for p in payloads], [3., 9., 5.])

    def test_pause_heartbeat_has_no_fabricated_xr_capture(self):
        worker = self.make_worker()
        worker.paused = True
        worker._latest_controller_t_ns = 77
        first = worker.sample_and_retarget()
        self.clock.advance(20 * MS)
        second = worker.sample_and_retarget()
        self.assertEqual(worker.streamer.read_calls, 0)
        self.assertEqual(second["motion_t_ns"] - first["motion_t_ns"], 20 * MS)
        for payload in [first, second]:
            self.assertEqual(payload["motion_source"], "paused_hold")
            self.assertIsNone(payload["pico_recv_time_ns"])
            self.assertIsNone(payload["pico_recv_monotonic_ns"])
            self.assertEqual(payload["smplx_t_ns"], 0)
            self.assertEqual(payload["controller_t_ns"], 77)
            self.assertEqual(payload["stream_id"], first["stream_id"])
        self.assertEqual(first["joint_pos"], second["joint_pos"])

    def test_missing_xr_returns_no_motion_frame(self):
        worker = self.make_worker()
        worker.streamer.fail = True
        with redirect_stdout(io.StringIO()):
            self.assertIsNone(worker.sample_and_retarget())

    def test_monotonic_motion_survives_wall_clock_step(self):
        worker = self.make_worker()
        first = worker.sample_and_retarget()
        worker._stamp_publication(first)
        self.clock.advance(20 * MS)
        self.clock.wall_ns -= 10_000 * MS
        second = worker.sample_and_retarget()
        worker._stamp_publication(second)
        self.assertGreater(second["motion_t_ns"], first["motion_t_ns"])
        self.assertLess(second["publish_t_ns"], first["publish_t_ns"])
        self.assertEqual(first["source_clock_id"], second["source_clock_id"])

    def test_stream_id_is_unique_per_worker(self):
        first, second = self.make_worker(), self.make_worker()
        self.assertTrue(first._stream_id)
        self.assertNotEqual(first._stream_id, second._stream_id)

    def test_send_failure_does_not_increment_seq(self):
        worker = self.make_worker()
        payloads = []
        for _ in range(3):
            self.clock.advance(20 * MS)
            payloads.append(worker.sample_and_retarget())
        iterator = iter(payloads)
        def sample():
            try:
                return next(iterator)
            except StopIteration:
                raise KeyboardInterrupt
        worker.sample_and_retarget = sample
        self.env["LiveRetargetPublisher"] = lambda args: worker
        self.context.next_outcomes = [None, FakeAgain(), None]
        with redirect_stdout(io.StringIO()):
            self.env["run_publish"](self.args)
        socket = self.context.sockets[-1]
        self.assertEqual([p["seq"] for p in socket.attempts], [0, 1, 1])
        self.assertEqual({p["stream_id"] for p in socket.attempts}, {worker._stream_id})
        self.assertTrue(all("publish_t_ns" in p and "publish_monotonic_ns" in p for p in socket.attempts))
        self.assertTrue(socket.closed)
        self.assertTrue(worker._controller_sock.closed)


class ActualJakaPublisherTimestampTests(PublisherTimestampTests):
    """The actual sender has JAKA mapping, startup silence and A/X edge logic."""
    source = ACTUAL_SOURCE
    robot = "jaka"

    def make_worker(self):
        worker = super().make_worker()
        worker._poll_pause_toggle = lambda: types.SimpleNamespace(X=False, A=False)
        worker._live_started = True  # inherited live/hold tests start after first X
        return worker

    def test_initial_pause_stays_silent_until_first_x(self):
        worker = self.make_worker()
        worker.paused, worker._live_started = True, False
        for _ in range(3):
            self.clock.advance(2_000 * MS)
            self.assertIsNone(worker.sample_and_retarget())
        self.assertEqual(worker.streamer.read_calls, 0)
        worker._poll_pause_toggle = lambda: types.SimpleNamespace(X=True, A=False)
        with redirect_stdout(io.StringIO()):
            payload = worker.sample_and_retarget()
        self.assertTrue(worker._live_started)
        self.assertFalse(worker.paused)
        self.assertEqual(payload["motion_source"], "xr_read")

    def test_actual_jaka_gmr_dispatch_is_preserved(self):
        targets = []
        def build(**kwargs):
            targets.append(kwargs["tgt_robot"])
            return FakeRetarget(self.clock)
        self.env["GMR"] = build
        self.make_worker()
        self.assertEqual(targets, ["jaka_mini"])
        with self.assertRaises(ValueError):
            self.env["_gmr_robot_name"]("unknown")

    def test_a_collection_toggle_remains_edge_triggered_bool_list(self):
        worker = self.make_worker()
        values = []
        for pressed in [True, True, False, True]:
            worker._poll_pause_toggle = lambda pressed=pressed: types.SimpleNamespace(X=False, A=pressed)
            with redirect_stdout(io.StringIO()):
                payload = worker.sample_and_retarget()
            values.append(payload["toggle_data_collection"])
        self.assertEqual(values, [[True], [True], [True], [False]])

    def test_x_pause_preserves_joints_xyz_and_clock_contract(self):
        worker = self.make_worker()
        original = np.arange(34, dtype=np.float32) / 100
        original[3:7] = [1, 0, 0, 0]
        worker.latest_qpos = original.copy()
        # Stub only the imported yaw helper; test the production pause capture.
        yaw = np.asarray([.98, 0, 0, .2], np.float32)
        self.env["yaw_quat"] = lambda quat: yaw.copy()
        worker._pose_arrays_from_qpos = lambda q: (
            np.asarray(q[7:]).copy(), np.zeros((14, 3), np.float32),
            np.tile(np.asarray([1, 0, 0, 0], np.float32), (14, 1)))
        stream = worker._stream_id
        worker._poll_pause_toggle = lambda: types.SimpleNamespace(X=True, A=False)
        with redirect_stdout(io.StringIO()):
            paused = worker.sample_and_retarget()
        np.testing.assert_array_equal(worker.paused_qpos[:3], original[:3])
        np.testing.assert_array_equal(worker.paused_qpos[7:], original[7:])
        np.testing.assert_array_equal(worker.paused_qpos[3:7], yaw)
        self.assertEqual(paused["motion_source"], "paused_hold")
        worker._poll_pause_toggle = lambda: types.SimpleNamespace(X=False, A=False)
        self.clock.advance(20 * MS)
        held = worker.sample_and_retarget()
        self.assertGreater(held["motion_t_ns"], paused["motion_t_ns"])
        worker._poll_pause_toggle = lambda: types.SimpleNamespace(X=True, A=False)
        with redirect_stdout(io.StringIO()):
            live = worker.sample_and_retarget()
        self.assertEqual(live["motion_source"], "xr_read")
        self.assertEqual(live["stream_id"], stream)
        self.assertEqual(live["source_clock_id"], paused["source_clock_id"])

    def test_actual_live_gather_missing_joint_uses_default(self):
        worker = self.make_worker()
        worker.joint_qpos_indices[0:3] = [8, 7, -1]
        worker.robot_cfg.default_qpos[9] = .87
        worker.retarget.configuration.data.qpos[:] = np.arange(34, dtype=np.float32) / 100
        payload = worker.sample_and_retarget()
        self.assertAlmostEqual(payload["joint_pos"][0], .08, places=6)
        self.assertAlmostEqual(payload["joint_pos"][1], .07, places=6)
        self.assertAlmostEqual(payload["joint_pos"][2], .87, places=6)
        self.assertAlmostEqual(payload["qpos"][9], .87, places=6)

    def test_actual_fk_scatter_uses_gmr_addresses(self):
        worker = self.make_worker()
        worker.joint_qpos_indices[0:3] = [8, 7, -1]
        worker.robot_cfg.default_qpos[9] = .87
        data = types.SimpleNamespace(qpos=np.zeros(34, np.float32),
            xpos=np.zeros((14, 3), np.float32),
            xquat=np.tile(np.asarray([1, 0, 0, 0], np.float32), (14, 1)))
        self.env["mujoco"] = types.SimpleNamespace(MjData=lambda model: data, mj_forward=lambda model, d: None)
        command = np.arange(34, dtype=np.float32) / 10
        joints, _, _ = self.cls._pose_arrays_from_qpos(worker, command)
        np.testing.assert_array_equal(data.qpos[:7], command[:7])
        self.assertAlmostEqual(data.qpos[8], command[7], places=6)
        self.assertAlmostEqual(data.qpos[7], command[8], places=6)
        self.assertAlmostEqual(joints[2], .87, places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
