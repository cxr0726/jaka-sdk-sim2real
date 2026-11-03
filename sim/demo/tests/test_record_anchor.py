"""Regression checks against the recorded link poses, including the ZMQ wire format."""

import importlib.util
import json
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import pyarrow.parquet as pq
import zmq


DEMO = Path(__file__).resolve().parents[1]
EPISODES = DEMO / "scripts/20261022-221234/level-0"
SCRIPT = DEMO / "scripts/pub_motion_record.py"
spec = importlib.util.spec_from_file_location("record_publisher", SCRIPT)
publisher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(publisher)


class RecordAnchorTests(unittest.TestCase):
    def test_all_recorded_episodes_select_real_link_poses(self):
        for episode in sorted(EPISODES.glob("episode_*")):
            table = pq.read_table(episode / "data.parquet")
            actions = np.asarray(table["actions"].to_pylist(), dtype=np.float32)
            positions = np.asarray(table["motion_body_pos_w"].to_pylist(), dtype=np.float32).reshape(-1, 28, 3)
            rotations = np.asarray(table["motion_body_quat_w"].to_pylist(), dtype=np.float32).reshape(-1, 28, 4)
            parquet_modes = {}
            for anchor, index in (("base", 0), ("waist", 13)):
                with self.subTest(episode=episode.name, anchor=anchor, source="parquet"):
                    motion = publisher.load_parquet_or_npz(episode / "data.parquet", anchor=anchor)
                    np.testing.assert_array_equal(motion["joint_pos"], actions[:, :27])
                    np.testing.assert_array_equal(motion["anchor_pos_w"], positions[:, index])
                    np.testing.assert_array_equal(motion["anchor_quat_w"], rotations[:, index])
                    packet = json.loads(publisher.build_json_payload(0, motion))
                    np.testing.assert_allclose(packet["body_pos_w"], np.tile(positions[0, index], (28, 1)), atol=5.1e-6, rtol=0)
                    np.testing.assert_allclose(packet["body_quat_w"], np.tile(rotations[0, index], (28, 1)), atol=5.1e-6, rtol=0)
                    parquet_modes[anchor] = motion
                with np.load(episode / "motion_raw.npz", allow_pickle=False) as raw:
                    with self.subTest(episode=episode.name, anchor=anchor, source="raw_npz"):
                        motion = publisher.load_parquet_or_npz(episode / "motion_raw.npz", anchor=anchor)
                        np.testing.assert_array_equal(motion["joint_pos"], raw["joint_pos"])
                        np.testing.assert_array_equal(motion["anchor_pos_w"], raw["body_pos_w"][:, index])
                        np.testing.assert_array_equal(motion["anchor_quat_w"], raw["body_quat_w"][:, index])
            np.testing.assert_array_equal(parquet_modes["base"]["timestamps"], parquet_modes["waist"]["timestamps"])

    def test_isaac_training_npz_uses_base_zero_and_waist_three(self):
        path = DEMO / "scripts/motion_pico2.npz"
        with np.load(path, allow_pickle=False) as raw:
            for anchor, index in (("base", 0), ("waist", 3)):
                motion = publisher.load_parquet_or_npz(path, anchor=anchor)
                np.testing.assert_array_equal(motion["anchor_pos_w"], raw["body_pos_w"][:, index])
                np.testing.assert_array_equal(motion["anchor_quat_w"], raw["body_quat_w"][:, index])

    def test_interpolation_retains_selected_link_and_endpoints(self):
        path = EPISODES / "episode_0000/data.parquet"
        for anchor, name in (("base", "base_link"), ("waist", "waist_yaw_Link")):
            motion = publisher.load_parquet_or_npz(path, anchor=anchor)
            sampled = publisher.interpolate_motion(motion, 50)
            self.assertEqual(sampled["anchor_body_name"], name)
            np.testing.assert_allclose(sampled["anchor_pos_w"][[0, -1]], motion["anchor_pos_w"][[0, -1]], atol=1e-7)
            np.testing.assert_allclose(sampled["anchor_quat_w"][[0, -1]], motion["anchor_quat_w"][[0, -1]], atol=1e-6)

    def test_base_requires_full_body_columns_in_parquet(self):
        table = pq.read_table(EPISODES / "episode_0000/data.parquet", columns=["actions", "timestamp_ns"])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "actions_only.parquet"
            pq.write_table(table, path)
            publisher.load_parquet_or_npz(path)  # Existing waist-only files remain usable.
            with self.assertRaisesRegex(ValueError, "requires Parquet columns"):
                publisher.load_parquet_or_npz(path, anchor="base")

    def test_export_keeps_anchor_label_and_rejects_false_reselection(self):
        with tempfile.TemporaryDirectory() as directory:
            for anchor in ("base", "waist"):
                motion = publisher.load_parquet_or_npz(EPISODES / "episode_0000/data.parquet", anchor=anchor)
                path = Path(directory) / f"{anchor}.npz"
                publisher.export_to_npz(motion, path)
                restored = publisher.load_parquet_or_npz(path, anchor=anchor)
                np.testing.assert_array_equal(restored["anchor_pos_w"], motion["anchor_pos_w"])
                np.testing.assert_array_equal(restored["anchor_quat_w"], motion["anchor_quat_w"])
                with self.assertRaisesRegex(ValueError, "contains only"):
                    publisher.load_parquet_or_npz(path, anchor="waist" if anchor == "base" else "base")
            # Legacy exports contain only repeated waist pose, without an anchor label.
            with np.load(Path(directory) / "waist.npz", allow_pickle=False) as exported:
                legacy = {key: exported[key] for key in exported.files if key not in ("anchor_body_name", "body_pose_format")}
            legacy_path = Path(directory) / "legacy.npz"
            np.savez(legacy_path, **legacy)
            publisher.load_parquet_or_npz(legacy_path)
            with self.assertRaisesRegex(ValueError, "contains only"):
                publisher.load_parquet_or_npz(legacy_path, anchor="base")

    def test_cli_sends_requested_pose_in_all_slots(self):
        table = pq.read_table(EPISODES / "episode_0000/data.parquet")
        positions = np.asarray(table["motion_body_pos_w"][0].as_py()).reshape(28, 3)
        rotations = np.asarray(table["motion_body_quat_w"][0].as_py()).reshape(28, 4)
        for anchor, index in (("base", 0), ("waist", 13)):
            with self.subTest(anchor=anchor):
                with socket.socket() as probe:
                    probe.bind(("127.0.0.1", 0))
                    endpoint = f"tcp://127.0.0.1:{probe.getsockname()[1]}"
                context = zmq.Context()
                subscriber = context.socket(zmq.SUB)
                subscriber.setsockopt(zmq.LINGER, 0)
                subscriber.setsockopt(zmq.SUBSCRIBE, b"")
                subscriber.connect(endpoint)
                process = subprocess.Popen(
                    [sys.executable, "-u", str(SCRIPT), str(EPISODES / "episode_0000/data.parquet"),
                     "--anchor", anchor, "--bind", endpoint, "--startup-sleep", "0.5"],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                )
                try:
                    self.assertTrue(subscriber.poll(5000), "Publisher did not send a frame")
                    packet = subscriber.recv_json()
                    np.testing.assert_allclose(packet["body_pos_w"], np.tile(positions[index], (28, 1)), atol=5.1e-6, rtol=0)
                    np.testing.assert_allclose(packet["body_quat_w"], np.tile(rotations[index], (28, 1)), atol=5.1e-6, rtol=0)
                finally:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                    try:
                        output, _ = process.communicate(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        output, _ = process.communicate()
                    subscriber.close(0)
                    context.term()
                self.assertEqual(process.returncode, 0, output)


if __name__ == "__main__":
    unittest.main()
