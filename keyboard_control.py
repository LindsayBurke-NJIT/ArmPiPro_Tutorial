#!/usr/bin/python3
# coding=utf8
"""
Keyboard teleop for the Arm Pi Pro robot.
Works over SSH - reads from terminal stdin (no pynput needed).

- WASD: Drive and strafe (W+D = forward-right diagonal, etc.)
- Arrow keys: Zero-point turn (rotate in place)
- Press Ctrl+C or Escape to stop and exit
"""
import sys
import time
import select
import tty
import termios

from direct_drive import MecanumChassis

DRIVE_SPEED = 60
TURN_SPEED = 50
KEY_TIMEOUT = 0.15  # Consider key "released" if not seen for this long (terminal repeats when held)
# Invert if directions are wrong (robot orientation/motor wiring varies)
INVERT_FORWARD = False
INVERT_STRAFE = False
INVERT_ROTATION = False


def main():
    if not sys.stdin.isatty():
        print("Error: Must run from an interactive terminal (e.g. SSH session).")
        print("Pipe/redirect won't work - run: python keyboard_control.py")
        sys.exit(1)

    chassis = MecanumChassis()

    # Keys we care about -> last time seen (terminal sends repeats when held)
    last_seen = {}
    KEY_UP = '\x1b[A'
    KEY_DOWN = '\x1b[B'
    KEY_RIGHT = '\x1b[C'
    KEY_LEFT = '\x1b[D'

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def is_pressed(key_id):
        return (time.time() - last_seen.get(key_id, 0)) < KEY_TIMEOUT

    def update_motors():
        forward = 0
        strafe = 0
        rotation = 0

        if is_pressed('w'):
            forward += DRIVE_SPEED
        if is_pressed('s'):
            forward -= DRIVE_SPEED
        if is_pressed('a'):
            strafe -= DRIVE_SPEED
        if is_pressed('d'):
            strafe += DRIVE_SPEED
        if is_pressed(KEY_LEFT):
            rotation -= TURN_SPEED
        if is_pressed(KEY_RIGHT):
            rotation += TURN_SPEED

        if INVERT_FORWARD:
            forward = -forward
        if INVERT_STRAFE:
            strafe = -strafe
        if INVERT_ROTATION:
            rotation = -rotation

        chassis.drive_xy(forward=forward, strafe=strafe, rotation=rotation)

    def read_key():
        """Non-blocking read of one key (handles escape sequences for arrows)."""
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            # Arrow keys send \x1b[A etc - use longer timeout for SSH latency
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch2 = sys.stdin.read(1)
                if ch2 == '[' and select.select([sys.stdin], [], [], 0.1)[0]:
                    ch3 = sys.stdin.read(1)
                    return ch + ch2 + ch3  # e.g. \x1b[A
            return ch  # Plain Escape (no follow-up bytes)
        return ch.lower() if ch.isalpha() else ch

    print("Keyboard control active (terminal/SSH mode).")
    print("  WASD = drive/strafe (W+D = diagonal, etc.)")
    print("  Arrow keys = zero-point turn")
    print("  Esc or Ctrl+C = quit")
    if any((INVERT_FORWARD, INVERT_STRAFE, INVERT_ROTATION)):
        print("  (Axis inversion active - edit INVERT_* constants if directions wrong)")
    print()

    try:
        tty.setraw(fd)
        running = True
        while running:
            key = read_key()
            if key is not None:
                if key == '\x03' or key == '\x1b':  # Ctrl+C or plain Escape
                    running = False
                    continue
                last_seen[key] = time.time()

            update_motors()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        chassis.stop_motors()
        print("\nMotors stopped.")


if __name__ == "__main__":
    main()
