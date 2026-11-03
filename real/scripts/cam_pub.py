#!/usr/bin/env python3
"""Publish Orbbec camera ROS 2 image frames over ZeroMQ.

Subscribes to the ROS 2 image topic (default: /camera/color/image_raw) published by
OrbbecSDK_ROS2 (e.g. launched via gemini2L.launch.py) and forwards the frames over a
ZeroMQ PUB socket (default: tcp://*:28712).

Wire format strictly matches ``RealImageClient._decode_message`` in
``record_jaka_zmq_real.py``:
    - 12-byte header: struct.pack("iii", width, height, channels)
    - Payload: raw uncompressed RGB uint8 bytes (size: width * height * channels)

Usage::

    # Run with defaults (keep camera native resolution, e.g. 1280x800@30Hz):
    python scripts/cam_pub.py

    # Resize to 640x360 or 640x480, rotate 180 degrees, and preview locally:
    python scripts/cam_pub.py --width 640 --height 360 --rotate-180 --show

    # Custom ROS topic or ZMQ bind address:
    python scripts/cam_pub.py --topic /camera/color/image_raw --bind tcp://*:28712
"""

from __future__ import annotations

import argparse
import signal
import struct
import sys
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
import zmq


def decode_ros_image_to_rgb(msg: Image) -> Optional[np.ndarray]:
    """Decode a ROS sensor_msgs/Image message to an RGB uint8 numpy array (H, W, 3)."""
    h = msg.height
    w = msg.width
    encoding = msg.encoding.lower()

    data = msg.data
    if not isinstance(data, (bytes, memoryview)):
        data = bytes(data)

    step = msg.step

    # Common color encodings
    if encoding in ("rgb8", "bgr8"):
        expected_step = w * 3
        if step >= expected_step:
            raw = np.frombuffer(data, dtype=np.uint8)
            if raw.size >= h * step:
                raw = raw[: h * step].reshape((h, step))
                img = raw[:, :expected_step].reshape((h, w, 3))
            elif raw.size >= h * expected_step:
                img = raw[: h * expected_step].reshape((h, w, 3))
            else:
                return None
        else:
            return None

        if encoding == "bgr8":
            return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    elif encoding in ("rgba8", "bgra8"):
        expected_step = w * 4
        if step >= expected_step:
            raw = np.frombuffer(data, dtype=np.uint8)
            if raw.size >= h * step:
                raw = raw[: h * step].reshape((h, step))
                img = raw[:, :expected_step].reshape((h, w, 4))
            elif raw.size >= h * expected_step:
                img = raw[: h * expected_step].reshape((h, w, 4))
            else:
                return None
        else:
            return None

        if encoding == "bgra8":
            return cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)

    elif encoding in ("mono8", "8uc1"):
        expected_step = w
        raw = np.frombuffer(data, dtype=np.uint8)
        if raw.size >= h * step:
            raw = raw[: h * step].reshape((h, step))
            img = raw[:, :expected_step].reshape((h, w))
        elif raw.size >= h * expected_step:
            img = raw[: h * expected_step].reshape((h, w))
        else:
            return None
        return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    elif encoding in ("yuv422", "yuyv", "yuv422_yuy2"):
        expected_step = w * 2
        raw = np.frombuffer(data, dtype=np.uint8)
        if raw.size >= h * step:
            raw = raw[: h * step].reshape((h, step))
            img = raw[:, :expected_step].reshape((h, w, 2))
        elif raw.size >= h * expected_step:
            img = raw[: h * expected_step].reshape((h, w, 2))
        else:
            return None
        return cv2.cvtColor(img, cv2.COLOR_YUV2RGB_YUYV)

    else:
        # Fallback for generic 3-channel data
        try:
            raw = np.frombuffer(data, dtype=np.uint8)
            if raw.size == h * w * 3:
                return raw.reshape((h, w, 3))
        except Exception:
            pass
        return None


def get_ros_qos_profile(qos_type: str) -> QoSProfile:
    """Create a QoSProfile based on user selection."""
    if qos_type in ("sensor_data", "best_effort"):
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
    # Default: Reliable (matches default Orbbec ROS 2 camera publisher)
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=2,
    )


class OrbbecImageZmqBridge(Node):
    """ROS 2 Node that subscribes to camera color topic and forwards over ZMQ PUB."""

    def __init__(
        self,
        topic: str,
        socket: zmq.Socket,
        target_size: Optional[Tuple[int, int]] = None,
        fps_limit: float = 0.0,
        rotate_180: bool = False,
        flip_lr: bool = False,
        flip_ud: bool = False,
        color_format: str = "rgb",
        qos_type: str = "default",
    ):
        super().__init__("orbbec_image_zmq_bridge")
        self.socket = socket
        self.target_size = target_size
        self.fps_limit = fps_limit
        self.min_interval = 1.0 / fps_limit if fps_limit > 0.0 else 0.0
        self.rotate_180 = rotate_180
        self.flip_lr = flip_lr
        self.flip_ud = flip_ud

        self.color_format = color_format.lower()

        self.last_pub_time = 0.0
        self.last_stat_time = time.time()
        self.frames_sent = 0
        self.bytes_sent = 0
        self.frames_dropped = 0
        self.total_received = 0

        self.latest_bgr_preview: Optional[np.ndarray] = None

        qos = get_ros_qos_profile(qos_type)
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            qos,
        )
        self.get_logger().info(
            f"Subscribed to topic '{topic}' (QoS: {qos_type}). "
            f"Forwarding over ZeroMQ (Format: {self.color_format.upper()})."
        )

    def image_callback(self, msg: Image) -> None:
        self.total_received += 1
        now = time.time()

        # Rate limiting check
        if self.min_interval > 0.0 and (now - self.last_pub_time) < self.min_interval:
            self.frames_dropped += 1
            return

        img_rgb = decode_ros_image_to_rgb(msg)
        if img_rgb is None:
            self.get_logger().warn(
                f"Unsupported or malformed image format (encoding: {msg.encoding}, "
                f"size: {msg.width}x{msg.height})"
            )
            return

        # Optional transformations
        if self.target_size is not None and (img_rgb.shape[1], img_rgb.shape[0]) != self.target_size:
            img_rgb = cv2.resize(img_rgb, self.target_size, interpolation=cv2.INTER_LINEAR)

        if self.rotate_180:
            img_rgb = cv2.rotate(img_rgb, cv2.ROTATE_180)
        if self.flip_lr:
            img_rgb = cv2.flip(img_rgb, 1)
        if self.flip_ud:
            img_rgb = cv2.flip(img_rgb, 0)

        # Convert to target color format
        if self.color_format == "bgr":
            out_img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            self.latest_bgr_preview = out_img
        else:
            out_img = img_rgb
            self.latest_bgr_preview = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

        # Ensure contiguous uint8 array
        out_img = np.ascontiguousarray(out_img, dtype=np.uint8)
        h, w, c = out_img.shape

        # Wire format: [w, h, c] int32 header + raw uint8 payload
        header = struct.pack("iii", w, h, c)
        payload = out_img.tobytes()
        msg_bytes = header + payload

        try:
            self.socket.send(msg_bytes, flags=zmq.NOBLOCK)
            self.frames_sent += 1
            self.bytes_sent += len(msg_bytes)
            self.last_pub_time = now
        except zmq.Again:
            self.frames_dropped += 1

        # Periodic statistics
        stat_elapsed = now - self.last_stat_time
        if stat_elapsed >= 5.0:
            fps = self.frames_sent / stat_elapsed
            mb_per_sec = (self.bytes_sent / (1024.0 * 1024.0)) / stat_elapsed
            print(
                f"[CamPublisher] Sent {self.frames_sent} frames ({fps:.1f} fps), "
                f"resolution={w}x{h}x{c}, throughput={mb_per_sec:.2f} MB/s, "
                f"dropped={self.frames_dropped}"
            )
            self.frames_sent = 0
            self.bytes_sent = 0
            self.frames_dropped = 0
            self.last_stat_time = now


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Publish Orbbec camera ROS 2 images over ZMQ for record_jaka_zmq_real.py"
    )
    parser.add_argument(
        "--bind",
        default="tcp://*:28712",
        help="ZMQ PUB bind address (default: tcp://*:28712, matching record_jaka_zmq_real)",
    )
    parser.add_argument(
        "--topic",
        default="/camera/color/image_raw",
        help="ROS 2 color image topic (default: /camera/color/image_raw)",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=0,
        help="Target image width (default: 0 = keep native camera resolution)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=0,
        help="Target image height (default: 0 = keep native camera resolution)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=0.0,
        help="Target publish FPS limit (default: 0 = forward all frames as received)",
    )
    parser.add_argument(
        "--qos",
        choices=["default", "sensor_data", "best_effort", "reliable"],
        default="default",
        help="ROS 2 subscription QoS profile (default: default)",
    )
    parser.add_argument(
        "--color-format",
        choices=["rgb", "bgr"],
        default="rgb",
        help="Color format to publish over ZMQ: 'rgb' (default, for record_jaka_zmq_real) or 'bgr'",
    )
    parser.add_argument(
        "--rotate-180",
        action="store_true",
        help="Rotate camera image 180 degrees (for inverted mounting)",
    )
    parser.add_argument(
        "--flip-lr",
        action="store_true",
        help="Flip image horizontally (left-right)",
    )
    parser.add_argument(
        "--flip-ud",
        action="store_true",
        help="Flip image vertically (upside-down)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show live image preview window locally via OpenCV (press 'q' to quit)",
    )
    args = parser.parse_args()

    target_size = (args.width, args.height) if (args.width > 0 and args.height > 0) else None

    # ZeroMQ PUB Socket
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SNDHWM, 2)
    socket.bind(args.bind)
    print(f"[CamPublisher] Bound ZMQ PUB to {args.bind}")
    print(f"[CamPublisher] Subscribing to ROS 2 topic '{args.topic}'")
    print(f"[CamPublisher] Output color format: {args.color_format.upper()}")
    if target_size:
        print(f"[CamPublisher] Resizing output to {target_size[0]}x{target_size[1]}")
    else:
        print("[CamPublisher] Keeping native camera resolution")

    # Signal handlers
    stop_flag = False

    def handle_signal(signum, frame):
        nonlocal stop_flag
        stop_flag = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    # Initialize ROS 2
    rclpy.init(args=None)
    node = OrbbecImageZmqBridge(
        topic=args.topic,
        socket=socket,
        target_size=target_size,
        fps_limit=args.fps,
        rotate_180=args.rotate_180,
        flip_lr=args.flip_lr,
        flip_ud=args.flip_ud,
        color_format=args.color_format,
        qos_type=args.qos,
    )

    try:
        if args.show:
            print("[CamPublisher] Preview enabled: press 'q' in the preview window to exit.")
            while rclpy.ok() and not stop_flag:
                rclpy.spin_once(node, timeout_sec=0.01)
                preview = node.latest_bgr_preview
                if preview is not None:
                    cv2.imshow("Orbbec Camera Preview (CamPublisher)", preview)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
        else:
            print("[CamPublisher] Running bridge loop (Ctrl+C to stop)...")
            while rclpy.ok() and not stop_flag:
                rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        print("\n[CamPublisher] Shutting down...")
        if args.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        socket.close()
        context.term()
        print("[CamPublisher] Cleanly exited.")


if __name__ == "__main__":
    main()
