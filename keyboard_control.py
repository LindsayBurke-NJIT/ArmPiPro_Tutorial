#!/usr/bin/python3
# coding=utf8
"""
Keyboard teleop for the Arm Pi Pro robot.
- WASD: Drive and strafe (W+D = forward-right diagonal, etc.)
- Arrow keys: Zero-point turn (rotate in place)
- Press Ctrl+C or Escape to stop and exit
"""
import sys
import time

try:
    from pynput import keyboard
except ImportError:
    print("This script requires pynput. Install with: pip install pynput")
    sys.exit(1)

from direct_drive import MecanumChassis

DRIVE_SPEED = 60
TURN_SPEED = 50

def main():
    chassis = MecanumChassis()
    pressed = set()

    def on_press(key):
        try:
            pressed.add(key)
        except AttributeError:
            pass

    def on_release(key):
        try:
            pressed.discard(key)
        except AttributeError:
            pass
        # Escape to exit
        if key == keyboard.Key.esc:
            return False

    def has_key(char):
        """Check if a letter key is pressed (pynput uses KeyCode, not identity)."""
        for k in pressed:
            if hasattr(k, 'char') and k.char and k.char.lower() == char.lower():
                return True
        return False

    def update_motors():
        forward = 0
        strafe = 0
        rotation = 0

        if has_key('w'):
            forward += DRIVE_SPEED
        if has_key('s'):
            forward -= DRIVE_SPEED
        if has_key('a'):
            strafe -= DRIVE_SPEED
        if has_key('d'):
            strafe += DRIVE_SPEED
        if keyboard.Key.left in pressed:
            rotation -= TURN_SPEED
        if keyboard.Key.right in pressed:
            rotation += TURN_SPEED

        chassis.drive_xy(forward=forward, strafe=strafe, rotation=rotation)

    print("Keyboard control active.")
    print("  WASD = drive/strafe (W+D = diagonal, etc.)")
    print("  Arrow keys = zero-point turn")
    print("  Esc or Ctrl+C = quit")
    print("Press Ctrl+C to stop.\n")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while listener.running:
            update_motors()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        chassis.stop_motors()
        print("Motors stopped.")


if __name__ == "__main__":
    main()
