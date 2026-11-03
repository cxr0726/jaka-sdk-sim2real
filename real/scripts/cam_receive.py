#!/usr/bin/env python3
"""Receive and display camera image frames streamed over ZeroMQ from cam_pub.py.

Subscribes to the ZeroMQ PUB socket hosted by ``cam_pub.py`` (default: tcp://127.0.0.1:28712
or tcp://<robot_ip>:28712) and renders the incoming frames using OpenCV ``cv2.imshow``.

Wire format:
    - 12-byte header: struct.unpack("iii", msg[:12]) -> (width, height, channels)
    - Payload: raw uncompressed uint8 image bytes (w * h * c)

Usage::

    # Local preview (cam_pub running on localhost:28712):
    python real/scripts/cam_receive.py

    # Receive from remote robot IP (e.g., 10.5.5.100):
    python real/scripts/cam_receive.py --connect tcp://10.5.5.100:28712

    # Or specify host and port separately:
    python real/scripts/cam_receive.py --host 10.5.5.100 --port 28712

    # If cam_pub was started with --color-format bgr:
    python real/scripts/cam_receive.py --color-format bgr

    # Resize display window:
    python real/scripts/cam_receive.py --width 640 --height 480
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


def decode_image_message(msg: bytes, color_format: str = "rgb") -> Optional[np.ndarray]:
    """Decode raw bytes into a BGR numpy array ready for cv2.imshow.

    Args:
        msg: Raw bytes containing 12-byte header (w, h, c) followed by pixel bytes.
        color_format: Format of pixels in the payload ('rgb' or 'bgr').

    Returns:
        BGR uint8 image array, or None if decoding fails.
    """
    if len(msg) < 12:
        return None

    try:
        w, h, c = struct.unpack("iii", msg[:12])
    except struct.error:
        return None

    expected_len = w * h * c
    payload = msg[12:]
    if len(payload) != expected_len or c not in (1, 3, 4):
        return None

    try:
        raw_img = np.frombuffer(payload, dtype=np.uint8).reshape((h, w, c))
    except (ValueError, TypeError):
        return None

    # Convert to standard BGR for cv2.imshow
    if c == 1:
        # Grayscale
        return cv2.cvtColor(raw_img, cv2.COLOR_GRAY2BGR)
    elif c == 4:
        # 4-channel BGRA / RGBA
        if color_format.lower() == "bgr":
            return cv2.cvtColor(raw_img, cv2.COLOR_BGRA2BGR)
        return cv2.cvtColor(raw_img, cv2.COLOR_RGBA2BGR)
    else:  # c == 3
        if color_format.lower() == "rgb":
            return cv2.cvtColor(raw_img, cv2.COLOR_RGB2BGR)
        return raw_img


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Receive and display ZeroMQ camera stream published by cam_pub.py",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--connect",
        type=str,
        default=None,
        help="Full ZMQ connect address, e.g. tcp://10.5.5.100:28712. Overrides --host/--port.",
    )
    parser.add_argument(
        "--host",
        type=str,
        default="127.0.0.1",
        help="Publisher IP or hostname (used if --connect is not set)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=28712,
        help="Publisher ZMQ port (used if --connect is not set)",
    )
    parser.add_argument(
        "--color-format",
        choices=["rgb", "bgr"],
        default="rgb",
        help="Color format published by cam_pub: 'rgb' (cam_pub default) or 'bgr'",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=0,
        help="Optional display width (0 = keep original resolution)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=0,
        help="Optional display height (0 = keep original resolution)",
    )
    parser.add_argument(
        "--no-overlay",
        action="store_true",
        help="Do not draw FPS and resolution info text on the image",
    )
    parser.add_argument(
        "--window-name",
        type=str,
        default="Camera Stream (cam_receive)",
        help="OpenCV display window title",
    )
    args = parser.parse_args()

    # Determine connection endpoint
    connect_addr = args.connect or f"tcp://{args.host}:{args.port}"
    target_display_size: Optional[Tuple[int, int]] = (
        (args.width, args.height) if (args.width > 0 and args.height > 0) else None
    )

    # Initialize ZeroMQ SUB socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.CONFLATE, 1)  # Keep only the latest frame, drop backlog
    socket.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all messages

    print(f"[CamReceiver] Connecting to {connect_addr}...")
    socket.connect(connect_addr)
    print(f"[CamReceiver] Connected. Expecting '{args.color_format.upper()}' input format.")
    print("[CamReceiver] Press 'q' or 'ESC' in the window (or Ctrl+C in terminal) to quit.\n")

    # Signal handling for clean exit
    stop_flag = False

    def handle_signal(signum, frame):
        nonlocal stop_flag
        stop_flag = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    # Poller for non-blocking wait with timeout (allows Ctrl+C even if no stream)
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    # Performance stats
    frame_count = 0
    fps = 0.0
    last_fps_time = time.time()
    frames_in_interval = 0
    total_bytes = 0

    cv2.namedWindow(args.window_name, cv2.WINDOW_AUTOSIZE)

    try:
        while not stop_flag:
            # Poll with 100ms timeout so loop regularly checks stop_flag
            socks = dict(poller.poll(timeout=100))
            if socket not in socks:
                # No frame received in this slice; handle window events
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):  # 'q' or ESC
                    break
                continue

            try:
                msg = socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                continue

            frame_bgr = decode_image_message(msg, color_format=args.color_format)
            if frame_bgr is None:
                continue

            frame_count += 1
            frames_in_interval += 1
            total_bytes += len(msg)

            # Calculate FPS every 0.5s
            now = time.time()
            elapsed = now - last_fps_time
            if elapsed >= 0.5:
                fps = frames_in_interval / elapsed
                frames_in_interval = 0
                last_fps_time = now

            # Optional display resize
            h, w = frame_bgr.shape[:2]
            if target_display_size is not None and (w, h) != target_display_size:
                frame_bgr = cv2.resize(frame_bgr, target_display_size, interpolation=cv2.INTER_LINEAR)

            # Optional info overlay
            if not args.no_overlay:
                disp_h, disp_w = frame_bgr.shape[:2]
                text = f"FPS: {fps:.1f} | Native: {w}x{h}"
                # Background banner for text readability
                cv2.rectangle(frame_bgr, (5, 5), (260, 32), (0, 0, 0), -1)
                cv2.putText(
                    frame_bgr,
                    text,
                    (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

            cv2.imshow(args.window_name, frame_bgr)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

    except KeyboardInterrupt:
        pass
    finally:
        print("\n[CamReceiver] Shutting down...")
        cv2.destroyAllWindows()
        socket.close()
        context.term()
        print(f"[CamReceiver] Exited cleanly (total frames received: {frame_count}).")


if __name__ == "__main__":
    main()
