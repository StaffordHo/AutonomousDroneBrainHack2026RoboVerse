#!/usr/bin/env python3
"""
Keyboard-to-UDP bridge for competition_mission.py manual override.

Run the mission with:
    TELEOP_ENABLED=1 python3 competition_mission.py

In another terminal:
    python3 teleop_keyboard_bridge.py

Controls:
    w/s: forward/back
    a/d: left/right
    q/e: yaw left/right
    r/f: climb/descend
    space: stop
    x: exit
"""

import argparse
import json
import os
import select
import socket
import sys
import termios
import time
import tty


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def parse_args():
    parser = argparse.ArgumentParser(description="Bridge keyboard input to RoboVerse teleop UDP.")
    parser.add_argument("--host", default=os.getenv("TELEOP_UDP_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.getenv("TELEOP_UDP_PORT", "14591")))
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--speed", type=float, default=0.45)
    parser.add_argument("--vertical", type=float, default=0.25)
    parser.add_argument("--yaw-rate", type=float, default=35.0)
    parser.add_argument("--hold-s", type=float, default=0.25)
    return parser.parse_args()


def read_key(timeout_s):
    readable, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if not readable:
        return None
    return sys.stdin.read(1)


def main():
    args = parse_args()
    packet_interval = 1.0 / max(args.rate, 1.0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    old_term = termios.tcgetattr(sys.stdin)

    command = {
        "enabled": False,
        "forward": 0.0,
        "right": 0.0,
        "down": 0.0,
        "yaw_rate": 0.0,
    }
    last_key_time = 0.0

    print(f"Sending keyboard teleop UDP to {args.host}:{args.port}")
    print("Controls: w/s forward, a/d strafe, q/e yaw, r/f climb/descend, space stop, x exit")

    try:
        tty.setcbreak(sys.stdin.fileno())

        while True:
            key = read_key(packet_interval)
            now = time.time()

            if key is not None:
                last_key_time = now

                command = {
                    "enabled": True,
                    "forward": 0.0,
                    "right": 0.0,
                    "down": 0.0,
                    "yaw_rate": 0.0,
                }

                if key == "w":
                    command["forward"] = args.speed
                elif key == "s":
                    command["forward"] = -args.speed
                elif key == "a":
                    command["right"] = -args.speed
                elif key == "d":
                    command["right"] = args.speed
                elif key == "q":
                    command["yaw_rate"] = -args.yaw_rate
                elif key == "e":
                    command["yaw_rate"] = args.yaw_rate
                elif key == "r":
                    command["down"] = -args.vertical
                elif key == "f":
                    command["down"] = args.vertical
                elif key == " ":
                    command["enabled"] = False
                elif key == "x":
                    break

            if now - last_key_time > args.hold_s:
                command["enabled"] = False
                command["forward"] = 0.0
                command["right"] = 0.0
                command["down"] = 0.0
                command["yaw_rate"] = 0.0

            command["forward"] = clamp(command["forward"], -args.speed, args.speed)
            command["right"] = clamp(command["right"], -args.speed, args.speed)
            command["down"] = clamp(command["down"], -args.vertical, args.vertical)
            command["yaw_rate"] = clamp(command["yaw_rate"], -args.yaw_rate, args.yaw_rate)

            sock.sendto(json.dumps(command).encode("utf-8"), (args.host, args.port))

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        stop = {"enabled": False, "forward": 0.0, "right": 0.0, "down": 0.0, "yaw_rate": 0.0}
        sock.sendto(json.dumps(stop).encode("utf-8"), (args.host, args.port))
        print("\nKeyboard teleop bridge stopped.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
