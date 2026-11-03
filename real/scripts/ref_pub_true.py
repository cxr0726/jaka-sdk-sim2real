#!/usr/bin/env python3
"""Publish Unitree SV1 stereo frames over ZeroMQ for the host-side VisionClient.

Message format (raw, no JPEG) matches deploy_real/data_utils/vision_client.py:
[4-byte width][4-byte height][4-byte channels][raw uint8 image data]
The payload is BGR uint8, stitched as [left | right].

Unitree SV1 camera note:
- Raw frame format: [right_image | left_image] (right on left half, left on right half)
- This publisher reorders to standard [left | right] format for consistency with ZED
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
import zmq


def encode_and_send(socket: zmq.Socket, image: np.ndarray) -> None:
    """Send raw BGR image with header: [width][height][channels][payload]"""
    height, width = image.shape[:2]
    channels = image.shape[2] if image.ndim == 3 else 1
    if channels not in (1, 3, 4):
        return
    img_contig = np.ascontiguousarray(image, dtype=np.uint8)
    payload = img_contig.tobytes()
    header = struct.pack("iii", width, height, channels)
    socket.send(header + payload)


def init_sv1_camera(
    device_node: int, width: int, height: int, fps: int
) -> cv2.VideoCapture:
    """Initialize Unitree SV1 camera with V4L2 backend"""
    print(f"[SV1Publisher] Initializing camera at /dev/video{device_node}")
    print(f"[SV1Publisher] Frame size: {width}x{height}, FPS: {fps}")

    cap = cv2.VideoCapture(device_node, cv2.CAP_V4L2)

    if not cap.isOpened():
        print("[SV1Publisher] Error: Failed to open camera")
        sys.exit(1)

    # Set camera parameters
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    print(f"[SV1Publisher] Camera opened successfully at {width}x{height} @{fps}fps")
    return cap


def grab_stereo(cap: cv2.VideoCapture) -> Optional[np.ndarray]:
    """Grab stereo frame from SV1 camera and reorder to [left | right] format"""
    ret, frame = cap.read()

    if not ret or frame is None:
        return None

    # Split stereo image: Unitree format is [right | left]
    h, w = frame.shape[:2]
    half_w = w // 2

    right_img = frame[:, :half_w]  # Left half of raw image is right eye
    left_img = frame[:, half_w:]  # Right half of raw image is left eye

    # Concatenate as [left | right] for standard stereo format
    combined = np.hstack((left_img, right_img))
    return combined


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Publish Unitree SV1 frames over ZMQ for OpenHLM vision client"
    )
    parser.add_argument("--bind", default="tcp://*:5555", help="ZMQ bind address")
    parser.add_argument("--device", type=int, default=0, help="Camera device node (/dev/videoN)")
    parser.add_argument("--width", type=int, default=928, help="Camera capture width (full stereo)")
    parser.add_argument("--height", type=int, default=400, help="Camera capture height")
    parser.add_argument("--fps", type=int, default=30, help="Camera capture FPS")
    parser.add_argument("--show", action="store_true", help="Preview the combined frame locally")
    args = parser.parse_args()

    # Initialize ZMQ publisher
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(args.bind)
    print(f"[SV1Publisher] Bound to {args.bind}")

    # Initialize camera
    cap = init_sv1_camera(args.device, args.width, args.height, args.fps)

    # Setup signal handlers for graceful shutdown
    def handle_sigint(signum, frame):  # noqa: unused frame
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    try:
        last_print = time.time()
        frame_count = 0
        last_print_frame = 0

        print("[SV1Publisher] Starting frame capture and publishing...")

        while True:
            combined = grab_stereo(cap)
            if combined is None:
                time.sleep(0.001)
                continue

            # Send frame over ZMQ
            encode_and_send(socket, combined)
            frame_count += 1

            # Show preview if requested
            if args.show:
                cv2.imshow("SV1Publisher [Left | Right]", combined)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            # Print status every 5 seconds
            if time.time() - last_print > 5.0:
                h, w = combined.shape[:2]
                print(f"[SV1Publisher] Sent raw frame {w}x{h} (channels=3), total frames: {frame_count}, fps: {(frame_count - last_print_frame) / (time.time() - last_print):.1f}")
                last_print_frame = frame_count
                last_print = time.time()

    except KeyboardInterrupt:
        print("\n[SV1Publisher] Interrupted, shutting down")
    finally:
        cap.release()
        socket.close()
        context.term()
        if args.show:
            cv2.destroyAllWindows()
        print("[SV1Publisher] Camera stopped successfully")


if __name__ == "__main__":
    main()