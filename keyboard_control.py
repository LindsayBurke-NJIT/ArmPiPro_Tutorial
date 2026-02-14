#!/usr/bin/python3
# coding=utf8
"""
Keyboard teleop for the Arm Pi Pro robot.
Works over SSH - reads from terminal stdin (no pynput needed).

- WASD: Drive and strafe (W=forward, A=left, S=back, D=right)
- Q/E: Zero-point turn (Q=left, E=right)
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


def main():
    if not sys.stdin.isatty():
        print("Error: Must run from an interactive terminal (e.g. SSH session).")
        print("Pipe/redirect won't work - run: python keyboard_control.py")
        sys.exit(1)

    chassis = MecanumChassis()

    # Keys we care about -> last time seen (terminal sends repeats when held)
    last_seen = {}

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def is_pressed(key_id):
        return (time.time() - last_seen.get(key_id, 0)) < KEY_TIMEOUT

    def update_motors():
        # Robot chassis mapping: forward+ -> left, forward- -> right, strafe+ -> forward, strafe- -> back
        forward = 0
        strafe = 0
        rotation = 0

        if is_pressed('w'):
            strafe += DRIVE_SPEED   # forward
        if is_pressed('s'):
            strafe -= DRIVE_SPEED   # backward
        if is_pressed('a'):
            forward += DRIVE_SPEED  # left
        if is_pressed('d'):
            forward -= DRIVE_SPEED  # right
        if is_pressed('q'):
            rotation -= TURN_SPEED  # turn left
        if is_pressed('e'):
            rotation += TURN_SPEED  # turn right

        chassis.drive_xy(forward=forward, strafe=strafe, rotation=rotation)

    def read_key():
        """Non-blocking read of one key (ignores arrow escape sequences)."""
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            # Arrow keys send \x1b[A etc - consume and ignore (we use Q/E for turn)
            if select.select([sys.stdin], [], [], 0.05)[0]:
                sys.stdin.read(2)  # consume [A, [B, etc.
                return None  # Ignore arrow keys
            return ch  # Plain Escape = quit
        return ch.lower() if ch.isalpha() else ch

    print("Keyboard control active (terminal/SSH mode).")
    print("  WASD = drive/strafe (W=forward, A=left, S=back, D=right)")
    print("  Q/E = zero-point turn")
    print("  Esc or Ctrl+C = quit")
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
