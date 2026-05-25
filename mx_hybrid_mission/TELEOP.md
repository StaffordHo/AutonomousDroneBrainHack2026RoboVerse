# Manual Teleop Override

The autonomous mission can now pause at motion gates while a UDP manual-control
stream is active. This is intended as an emergency/manual nudge channel during
RoboVerse testing, not as a replacement for PX4 safety checks.

## Start the Mission With Teleop Enabled

```bash
TELEOP_ENABLED=1 python3 competition_mission.py
```

The mission listens on `127.0.0.1:14591` by default for JSON packets:

```json
{"enabled": true, "forward": 0.2, "right": 0.0, "down": 0.0, "yaw_rate": 0.0}
```

All values are clamped by the mission:

- `forward`: body-frame forward velocity in m/s
- `right`: body-frame right velocity in m/s
- `down`: body-frame down velocity in m/s, negative is climb
- `yaw_rate`: degrees per second

## Linux Joystick Bridge

If the controller appears as a Linux joystick device, run:

```bash
python3 teleop_udp_bridge.py --device /dev/input/js0
```

Check devices with:

```bash
ls /dev/input/js*
```

If the axes are mapped differently, remap them:

```bash
python3 teleop_udp_bridge.py \
  --device /dev/input/js0 \
  --axis-roll 0 \
  --axis-pitch 1 \
  --axis-yaw 2 \
  --axis-throttle 3
```

By default, manual override is active only while a stick is outside the deadband.
For safer deliberate activation, require a held button:

```bash
python3 teleop_udp_bridge.py --device /dev/input/js0 --deadman-button 4
```

## DJI RC-N3 Notes

DJI RC-N3 controllers are normally designed to connect to a mobile device and
DJI app stack. If Linux does not expose the RC-N3 as `/dev/input/js*`, this
bridge cannot read it directly. In that case use either a standard USB/Bluetooth
gamepad, QGroundControl joystick manual control, or a small phone/app bridge
that reads DJI controller state and sends the UDP JSON packet above.

## Keyboard Fallback

If no joystick device exists, use the keyboard bridge for manual nudge testing:

```bash
python3 teleop_keyboard_bridge.py
```

Controls are `w/s` forward/back, `a/d` left/right, `q/e` yaw,
`r/f` climb/descend, `space` stop, and `x` exit.
