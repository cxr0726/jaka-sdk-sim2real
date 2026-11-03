#!/usr/bin/env python3
"""Publish Intel RealSense D405 wrist camera frames over ZeroMQ.

Payload format matches `VisionClient` and `vision_publisher`:
[4-byte width][4-byte height][4-byte JPEG length][JPEG data]

Supports optional left/right cameras (by serial number). If两个都启用，水平拼接后发送；
若只启用一个，则发送单目画面。
"""

from __future__ import annotations

import argparse
import signal
import struct
import sys
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs
import zmq


def encode_and_send(socket: zmq.Socket, image: np.ndarray, quality: int) -> None:
    h, w = image.shape[:2]
    ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        return
    payload = jpeg.tobytes()
    header = struct.pack("iii", w, h, len(payload))
    socket.send(header + payload)


def start_pipeline(serial: str, width: int, height: int, fps: int) -> rs.pipeline:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    pipeline.start(config)
    print(f"[WristPublisher] Started D405 serial={serial} at {width}x{height}@{fps}")
    return pipeline


def grab_frame(pipeline: rs.pipeline, target_size: Tuple[int, int]) -> Optional[np.ndarray]:
    frames = pipeline.wait_for_frames(timeout_ms=1000)
    color = frames.get_color_frame()
    if not color:
        return None
    img = np.asanyarray(color.get_data())  # BGR
    if (img.shape[1], img.shape[0]) != target_size:
        img = cv2.resize(img, target_size)
    return img


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish RealSense D405 wrist cameras over ZMQ")
    parser.add_argument("--bind", default="tcp://*:5556", help="ZMQ bind address for wrists")
    parser.add_argument("--left-serial", default='230322271513', help="Left D405 serial number (optional)")
    parser.add_argument("--right-serial", default='230322270491', help="Right D405 serial number (optional)")
    parser.add_argument("--width", type=int, default=640, help="Per-camera width") # raw image is 1280x720
    parser.add_argument("--height", type=int, default=480, help="Per-camera height")  # D405 Color stream supports 640x480@30fps
    parser.add_argument("--fps", type=int, default=30, help="Capture FPS")
    parser.add_argument("--jpeg-quality", type=int, default=80, help="JPEG quality 0-100")
    parser.add_argument("--left-rotate-180", action="store_true", help="Rotate left camera 180 degrees")
    parser.add_argument("--right-rotate-180", action="store_true", help="Rotate right camera 180 degrees")
    parser.add_argument("--show", action="store_true", help="Preview locally")
    args = parser.parse_args()

    if not args.left_serial and not args.right_serial:
        print("[WristPublisher] At least one of --left-serial/--right-serial must be set")
        sys.exit(1)

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(args.bind)
    print(f"[WristPublisher] Bound to {args.bind}")

    pipelines: List[Tuple[str, rs.pipeline]] = []
    try:
        if args.left_serial:
            pipelines.append(("left", start_pipeline(args.left_serial, args.width, args.height, args.fps)))
        if args.right_serial:
            pipelines.append(("right", start_pipeline(args.right_serial, args.width, args.height, args.fps)))
    except Exception as exc:
        print(f"[WristPublisher] Failed to start pipelines: {exc}")
        socket.close()
        context.term()
        sys.exit(1)

    target_size = (args.width, args.height)

    def handle_sigint(signum, frame):  # noqa: D401 unused frame
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    try:
        last_print = time.time()
        while True:
            frames = []
            for name, pipe in pipelines:
                img = grab_frame(pipe, target_size)
                if img is None:
                    break
                # 根据参数决定是否旋转180度（适配物理安装方向）
                if name == "left" and args.left_rotate_180:
                    img = cv2.rotate(img, cv2.ROTATE_180)
                elif name == "right" and args.right_rotate_180:
                    img = cv2.rotate(img, cv2.ROTATE_180)
                frames.append(img)
            if len(frames) != len(pipelines):
                continue

            combined = frames[0] if len(frames) == 1 else np.hstack(frames)
            encode_and_send(socket, combined, args.jpeg_quality)

            if args.show:
                cv2.imshow("WristPublisher", combined)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            if time.time() - last_print > 5.0:
                h, w = combined.shape[:2]
                print(f"[WristPublisher] Sent wrist frame {w}x{h} q={args.jpeg_quality}")
                last_print = time.time()

    except KeyboardInterrupt:
        print("[WristPublisher] Interrupted, shutting down")
    finally:
        for _, pipe in pipelines:
            pipe.stop()
        socket.close()
        context.term()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()