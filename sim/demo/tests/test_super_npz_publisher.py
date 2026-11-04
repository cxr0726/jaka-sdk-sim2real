#!/usr/bin/env python3
"""Numerical/wire-contract tests; require NumPy, never open a ZMQ socket."""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "pub_motion_super_npz.py"
SPEC = importlib.util.spec_from_file_location("super_npz_publisher", SCRIPT)
publisher = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = publisher
SPEC.loader.exec_module(publisher)


class SuperNpzPublisherTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "motion.npz"

    def arrays(self, frames=3, bodies=28, fps=50.0):
        q = np.arange(frames * 27, dtype=np.float64).reshape(frames, 27)
        pos = np.arange(frames * bodies * 3, dtype=np.float64).reshape(frames, bodies, 3)
        quat = np.zeros((frames, bodies, 4))
        quat[..., 0] = 1.0
        return dict(fps=np.array([fps]), joint_pos=q, body_pos_w=pos, body_quat_w=quat)

    def write_load(self, arrays, layout="auto"):
        np.savez(self.path, **arrays)
        return publisher.load_motion(self.path, layout=layout)

    def named_arrays(self, bodies=45, base=0):
        arrays = self.arrays(bodies=bodies)
        names = [f"unused_body_{index}" for index in range(bodies)]
        names[base] = "base_link"
        arrays.update(joint_names=np.array(publisher.JAKA_JOINT_NAMES), body_names=np.array(names),
                      source_quat_order=np.array("xyzw"))
        return arrays

    def test_legacy_isaac_exact_gather_and_base_slot(self):
        arrays = self.arrays()
        motion = self.write_load(arrays)
        expected = [0, 3, 8, 13, 17, 21, 1, 4, 9, 14, 18, 22, 2,
                    5, 10, 15, 19, 23, 25, 7, 12, 16, 20, 24, 26, 6, 11]
        np.testing.assert_array_equal(motion.joint_pos, arrays["joint_pos"][:, expected])
        np.testing.assert_array_equal(motion.body_pos_w[:, 0], arrays["body_pos_w"][:, 0])
        np.testing.assert_array_equal(motion.body_pos_w[:, 1:], 0.0)
        np.testing.assert_array_equal(motion.body_quat_w[:, 1:, 0], 1.0)
        self.assertEqual(motion.describe(), dict(layout="isaac", frames=3, fps=50.0, reference_height_offset=0.0))

    def test_named_jaka_does_not_double_reorder_or_override_upper_body(self):
        arrays = self.named_arrays()
        arrays["body_quat_w"][:, 0] = [0.0, 0.0, 0.0, 2.0]
        motion = self.write_load(arrays)
        np.testing.assert_array_equal(motion.joint_pos, arrays["joint_pos"])
        np.testing.assert_array_equal(motion.body_pos_w[:, 0], arrays["body_pos_w"][:, 0])
        # source_quat_order=xyzw does not describe NPZ body_quat_w; output remains wxyz.
        np.testing.assert_array_equal(motion.body_quat_w[:, 0], [[0.0, 0.0, 0.0, 1.0]] * 3)
        self.assertEqual(motion.layout, "jaka")
        self.assertEqual(motion.reference_height_offset, 0.0)

    def test_named_shuffled_joints_and_compact_bodies_without_waist(self):
        arrays = self.named_arrays(bodies=15, base=10)
        expected_q = arrays["joint_pos"].copy()
        order = np.roll(np.arange(27)[::-1], 3)
        arrays["joint_names"] = arrays["joint_names"][order]
        arrays["joint_pos"] = arrays["joint_pos"][:, order]
        motion = self.write_load(arrays)
        np.testing.assert_array_equal(motion.joint_pos, expected_q)
        np.testing.assert_array_equal(motion.body_pos_w[:, 0], arrays["body_pos_w"][:, 10])

    def test_explicit_unnamed_jaka_uses_base_zero_only(self):
        arrays = self.arrays()
        motion = self.write_load(arrays, "jaka")
        np.testing.assert_array_equal(motion.joint_pos, arrays["joint_pos"])
        np.testing.assert_array_equal(motion.body_pos_w[:, 0], arrays["body_pos_w"][:, 0])
        self.assertEqual(motion.reference_height_offset, 0.0)

    def test_ambiguous_partial_duplicate_and_missing_names_rejected(self):
        with self.assertRaisesRegex(ValueError, "Cannot infer"):
            self.write_load(self.arrays(bodies=45))
        partial = self.named_arrays()
        del partial["body_names"]
        with self.assertRaisesRegex(ValueError, "Partial"):
            self.write_load(partial)
        duplicate = self.named_arrays()
        duplicate["joint_names"][0] = duplicate["joint_names"][1]
        with self.assertRaisesRegex(ValueError, "duplicate"):
            self.write_load(duplicate)
        missing = self.named_arrays()
        missing["body_names"][0] = "not_the_base"
        with self.assertRaisesRegex(ValueError, "must contain base_link"):
            self.write_load(missing)

    def test_nonfinite_zero_quaternion_and_bad_fps_rejected(self):
        for key in ("joint_pos", "body_pos_w", "body_quat_w"):
            with self.subTest(key=key):
                arrays = self.arrays()
                arrays[key].flat[-1] = np.nan
                with self.assertRaisesRegex(ValueError, "NaN or infinity"):
                    self.write_load(arrays)
        arrays = self.arrays()
        arrays["body_quat_w"][0, 0] = 0
        with self.assertRaisesRegex(ValueError, "zero or invalid quaternion"):
            self.write_load(arrays)
        for fps in (0, -1, np.inf, np.nan):
            with self.subTest(fps=fps), self.assertRaisesRegex(ValueError, "fps"):
                self.write_load(self.arrays(fps=fps))

    def test_source_30fps_is_sampled_in_seconds_at_50hz(self):
        arrays = self.arrays(frames=31, fps=30)
        arrays["joint_pos"][:] = np.arange(31)[:, None]
        motion = self.write_load(arrays, "jaka")
        for seconds, expected in ((0.02, 0.6), (0.5, 15.0), (1.0, 30.0)):
            q, _, _, _ = publisher.sample_motion(motion, seconds)
            np.testing.assert_allclose(q, expected)
        self.assertEqual(motion.duration_s, 1.0)

    def test_linear_position_and_shortest_path_quaternion_slerp(self):
        arrays = self.arrays(frames=2, fps=1)
        arrays["joint_pos"][:] = np.array([0.0, 10.0])[:, None]
        arrays["body_pos_w"][:] = np.array([0.0, 2.0])[:, None, None]
        arrays["body_quat_w"][1, 0] = [0.0, 0.0, 0.0, 1.0]
        motion = self.write_load(arrays, "jaka")
        q, pos, quat, finished = publisher.sample_motion(motion, 0.5)
        np.testing.assert_allclose(q, 5.0)
        np.testing.assert_allclose(pos[0], 1.0)
        np.testing.assert_allclose(quat[0], [2**-0.5, 0, 0, 2**-0.5], atol=1e-12)
        np.testing.assert_allclose(np.linalg.norm(quat, axis=-1), 1.0)
        self.assertFalse(finished)
        arrays["body_quat_w"][1, 0] = [-1.0, 0.0, 0.0, 0.0]
        motion = self.write_load(arrays, "jaka")
        _, _, quat, _ = publisher.sample_motion(motion, 0.5)
        np.testing.assert_allclose(quat[0], [1, 0, 0, 0], atol=1e-12)

    def test_final_pose_heartbeat_and_loop_timestamps(self):
        motion = self.write_load(self.arrays(frames=2, fps=50), "jaka")
        packets = []
        for seq, elapsed in enumerate((0.02, 0.04, 10.0)):
            packet, finished = publisher.make_payload(
                motion, elapsed, motion_t_ns=1_000_000_000 + int(elapsed * 1e9), seq=seq,
                stream_id="fixed-stream", source_clock_id="npz-monotonic:fixed-stream",
            )
            self.assertTrue(finished)
            np.testing.assert_array_equal(packet["joint_pos"], motion.joint_pos[-1])
            self.assertEqual(packet["source_time_field"], "motion_t_ns")
            self.assertGreater(packet["publish_t_ns"], 0)
            packets.append(packet)
        self.assertLess(packets[0]["motion_t_ns"], packets[-1]["motion_t_ns"])
        self.assertEqual(packets[0]["source_clock_id"], packets[-1]["source_clock_id"])
        q, _, _, finished = publisher.sample_motion(motion, 0.04, loop=True)
        np.testing.assert_array_equal(q, motion.joint_pos[0])
        self.assertFalse(finished)
        single = self.write_load(self.arrays(frames=1), "jaka")
        self.assertTrue(publisher.sample_motion(single, 0)[-1])

    def test_explicit_height_offset_is_applied_once_to_sampled_payload(self):
        arrays = self.named_arrays(bodies=15, base=10)
        original = {name: value.copy() for name, value in arrays.items()}
        baseline = self.write_load(arrays)
        shifted = publisher.load_motion(self.path, height_offset=0.05)
        stored_positions = shifted.body_pos_w.copy()
        kwargs = dict(motion_t_ns=1234567890, seq=7, stream_id="offset-test",
                      source_clock_id="npz-monotonic:offset-test")
        for elapsed in (0.0, 0.01, shifted.duration_s, 10.0, 0.01):
            with self.subTest(elapsed=elapsed), patch.object(publisher.time, "time_ns", return_value=9876543210):
                plain, plain_finished = publisher.make_payload(baseline, elapsed, **kwargs)
                packet, finished = publisher.make_payload(shifted, elapsed, **kwargs)
            expected = np.asarray(plain["body_pos_w"])
            expected[:, 2] += 0.05
            np.testing.assert_allclose(packet["body_pos_w"], expected, rtol=0, atol=1e-12)
            # Only world positions change, including placeholder body slots.
            for field in set(plain) - {"body_pos_w"}:
                self.assertEqual(packet[field], plain[field])
            self.assertEqual(finished, plain_finished)
            np.testing.assert_array_equal(shifted.body_pos_w, stored_positions)
        self.assertEqual(shifted.reference_height_offset, 0.05)
        for name, value in arrays.items():
            np.testing.assert_array_equal(value, original[name])
        with np.load(self.path, allow_pickle=False) as saved:
            for name, value in original.items():
                np.testing.assert_array_equal(saved[name], value)
        # Reloading without the explicit option must recover the original height.
        np.testing.assert_array_equal(publisher.load_motion(self.path).body_pos_w, baseline.body_pos_w)

    def test_invalid_height_offsets_rejected_before_socket_import(self):
        self.write_load(self.arrays())
        for offset in (np.nan, np.inf, -np.inf, 1.01, -1.01):
            with self.subTest(offset=offset):
                with self.assertRaisesRegex(ValueError, "height_offset must be finite"):
                    publisher.load_motion(self.path, height_offset=offset)
                error = io.StringIO()
                with patch.dict(sys.modules, {"zmq": None}), redirect_stderr(error):
                    with self.assertRaises(SystemExit) as raised:
                        publisher.main([str(self.path), "--describe", f"--height-offset={offset}"])
                self.assertEqual(raised.exception.code, 2)
                self.assertIn("height_offset must be finite", error.getvalue())

    def test_describe_does_not_import_zmq_or_open_socket(self):
        self.write_load(self.arrays())
        stream = io.StringIO()
        # A missing module would make `import zmq` fail if --describe reached it.
        with patch.dict(sys.modules, {"zmq": None}), redirect_stdout(stream):
            result = publisher.main([str(self.path), "--describe"])
        self.assertEqual(result, 0)
        self.assertEqual(json.loads(stream.getvalue()),
                         dict(layout="isaac", frames=3, fps=50.0, reference_height_offset=0.0))
        stream = io.StringIO()
        with patch.dict(sys.modules, {"zmq": None}), redirect_stdout(stream):
            result = publisher.main([str(self.path), "--describe", "--height-offset=-0.05"])
        self.assertEqual(result, 0)
        self.assertEqual(json.loads(stream.getvalue())["reference_height_offset"], -0.05)


if __name__ == "__main__":
    unittest.main()
