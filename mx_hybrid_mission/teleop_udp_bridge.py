#!/usr/bin/env python3
"""
Joystick-to-UDP bridge for competition_mission.py manual override.

This uses the Linux joystick API directly, so it has no pygame dependency.
If a controller appears as /dev/input/js0, this script maps axes into the
mission teleop UDP protocol.

Run:
    TELEOP_ENABLED=1 python3 competition_mission.py

In another terminal:
    python3 teleop_udp_bridge.py --device /dev/input/js0
"""

import argparse
import json
import os
import select
import socket
import struct
import time


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def normalize_axis(value, deadband):
    normalized = clamp(value / 32767.0, -1.0, 1.0)
    if abs(normalized) < deadband:
        return 0.0
    return normalized


def parse_args():
    parser = argparse.ArgumentParser(description="Bridge Linux joystick input to RoboVerse teleop UDP.")
    parser.add_argument("--device", default=os.getenv("TELEOP_JOYSTICK", "/dev/input/js0"))
    parser.add_argument("--host", default=os.getenv("TELEOP_UDP_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.getenv("TELEOP_UDP_PORT", "14591")))
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--deadband", type=float, default=0.08)
    parser.add_argument("--axis-roll", type=int, default=int(os.getenv("TELEOP_AXIS_ROLL", "0")))
    parser.add_argument("--axis-pitch", type=int, default=int(os.getenv("TELEOP_AXIS_PITCH", "1")))
    parser.add_argument("--axis-yaw", type=int, default=int(os.getenv("TELEOP_AXIS_YAW", "2")))
    parser.add_argument("--axis-throttle", type=int, default=int(os.getenv("TELEOP_AXIS_THROTTLE", "3")))
    parser.add_argument(
        "--deadman-button",
        type=int,
        default=int(os.getenv("TELEOP_DEADMAN_BUTTON", "-1")),
        help="Button that must be held for manual commands. -1 disables the requirement.",
    )
    parser.add_argument("--max-forward", type=float, default=0.8)
    parser.add_argument("--max-right", type=float, default=0.8)
    parser.add_argument("--max-down", type=float, default=0.45)
    parser.add_argument("--max-yaw-rate", type=float, default=55.0)
    return parser.parse_args()


def main():
    args = parse_args()
    axes = {}
    buttons = {}
    packet_interval = 1.0 / max(args.rate, 1.0)
    next_send = 0.0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Opening joystick: {args.device}")
    print(f"Sending teleop UDP to {args.host}:{args.port}")
    print("Use --axis-* arguments if sticks are mapped differently.")

    with open(args.device, "rb", buffering=0) as joystick:
        while True:
            now = time.time()
            timeout = max(0.0, min(packet_interval, next_send - now))
            readable, _, _ = select.select([joystick], [], [], timeout)

            if readable:
                event = joystick.read(8)
                if len(event) == 8:
                    _, value, event_type, number = struct.unpack("IhBB", event)
                    clean_type = event_type & ~JS_EVENT_INIT

                    if clean_type == JS_EVENT_AXIS:
                        axes[number] = normalize_axis(value, args.deadband)
                    elif clean_type == JS_EVENT_BUTTON:
                        buttons[number] = bool(value)

            now = time.time()
            if now < next_send:
                continue

            next_send = now + packet_interval

            deadman_ok = (
                args.deadman_button < 0
                or buttons.get(args.deadman_button, False)
            )

            roll = axes.get(args.axis_roll, 0.0)
            pitch = axes.get(args.axis_pitch, 0.0)
            yaw = axes.get(args.axis_yaw, 0.0)
            throttle = axes.get(args.axis_throttle, 0.0)

            axis_active = any(
                abs(value) > 0.0
                for value in (roll, pitch, yaw, throttle)
            )
            enabled = deadman_ok if args.deadman_button >= 0 else axis_active

            payload = {
                "enabled": enabled,
                "forward": clamp(-pitch * args.max_forward, -args.max_forward, args.max_forward),
                "right": clamp(roll * args.max_right, -args.max_right, args.max_right),
                "down": clamp(throttle * args.max_down, -args.max_down, args.max_down),
                "yaw_rate": clamp(yaw * args.max_yaw_rate, -args.max_yaw_rate, args.max_yaw_rate),
            }

            sock.sendto(json.dumps(payload).encode("utf-8"), (args.host, args.port))


if __name__ == "__main__":
    try:
        main()
    except FileNotFoundError as error:
        print(f"Joystick device not found: {error.filename}")
        print("Check with: ls /dev/input/js*")
    except KeyboardInterrupt:
        print("\nTeleop bridge stopped.")
