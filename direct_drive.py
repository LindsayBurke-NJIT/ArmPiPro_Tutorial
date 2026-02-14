#!/usr/bin/python3
# coding=utf8
import time
import smbus2
import math

# Motor controller I2C address
ENCODER_MOTOR_MODULE_ADDRESS = 0x34

class MecanumChassis:
    def __init__(self, i2c_port=1):
        """Initialize the mecanum wheel chassis controller
        Args:
            i2c_port (int): The I2C port number (default is 1)
        """
        self.i2c_port = i2c_port
        # Initialize motor controller type
        with smbus2.SMBus(self.i2c_port) as bus:
            # Set motor type to 3
            bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 51, [3,])

    def set_motor_speeds(self, speeds):
        """Set the speeds for all motors
        Args:
            speeds (list): List of 4 speed values (-100 to 100) for motors 1-4
        """
        with smbus2.SMBus(self.i2c_port) as bus:
            try:
                # Send all motor speeds at once
                bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 51, speeds) #set speeds to [3,] for all motors one by one (change speeds to [1,2,3,4])
            except Exception as e:
                print(f"Error setting motor speeds: {e}")

    def stop_motors(self):
        """Stop all motors"""
        self.set_motor_speeds([0, 0, 0, 0])

    def drive_forward(self, speed=60, duration=2):
        """Drive the robot forward
        Args:
            speed (int): Speed in range -100 to 100 mm/s (default 60)
            duration (int): How long to drive in seconds (default 2)
        """
        if not -100 <= speed <= 100:
            raise ValueError("Speed must be between -100 and 100")

        # For mecanum wheels moving forward:
        # All motors move in same direction
        speeds = [speed] * 4
        
        try:
            print(f"Driving forward at speed {speed} for {duration} seconds...")
            self.set_motor_speeds(speeds)
            time.sleep(duration)
        finally:
            print("Stopping...")
            self.stop_motors()

    def drive_xy(self, forward=0, strafe=0, rotation=0, base_speed=60):
        """Continuous drive with forward, strafe, and rotation components.
        Call repeatedly (e.g. from a keyboard loop) for responsive control.
        Args:
            forward (int): Forward/backward (-100 to 100, positive=forward)
            strafe (int): Left/right strafe (-100 to 100, positive=right)
            rotation (int): In-place turn (-100 to 100, positive=clockwise)
            base_speed (int): Speed multiplier (default 60)
        """
        # Mecanum wheel kinematics: [frontLeft, frontRight, backLeft, backRight]
        fl = forward + strafe - rotation
        fr = forward - strafe + rotation
        bl = forward - strafe - rotation
        br = forward + strafe + rotation
        speeds = [fl, fr, bl, br]
        # Scale down if any motor exceeds max
        max_val = max(abs(s) for s in speeds)
        if max_val > 100:
            scale = 100 / max_val
            speeds = [int(s * scale) for s in speeds]
        speeds = [max(-100, min(100, s)) for s in speeds]
        self.set_motor_speeds(speeds)

    def drive(self, speed=100, angle=0, duration=2):
        """Drive the robot forward
        Args:
            speed (int): Speed in range -100 to 100 mm/s (default 60)
            angle (int): Angle at which to drive the robot (0 to 359 -> 0 is right, 180 is back, 270 is left)
            duration (int): How long to drive in seconds (default 2)
        """
        if not -100 <= speed <= 100:
            raise ValueError("Speed must be between -100 and 100")
        radians = angle*(math.pi/180.)

        frontLeftSpeed = int(math.cos(radians)*speed+math.sin(radians)*speed)
        frontRightSpeed = int(-1*math.cos(radians)*speed+math.sin(radians)*speed)
        backLeftSpeed = int(-1*math.cos(radians)*speed+math.sin(radians)*speed)
        backRightSpeed = int(math.cos(radians)*speed + math.sin(radians)*speed)
        speeds = [frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed]
        
        try:
            print(f"Driving on {angle} degree angle at speed {speed} for {duration} seconds...")
            self.set_motor_speeds(speeds)
            time.sleep(duration)
        finally:
            print("Stopping...")
            self.stop_motors()
    
    def move_servo(self, servo_id, angle):
        import serial
        ser = serial.Serial('/dev/ttyAMA0', 10000, timeout=1)
        if 0<=angle<=180:
            data_packet = bytes([255, servo_id, angle])
            ser.write(data_packet)

def main():
    i2cPort = 1
    chassis = MecanumChassis()
    try:
        # Drive forward at 60mm/s for 2 seconds
        chassis.drive(60, 90, 2)
    except KeyboardInterrupt:
        print("\nStopping due to keyboard interrupt...")
    finally:
        chassis.stop_motors()
        with smbus2.SMBus(1) as bus:
            bus.close()
        print("Motors stopped")

if __name__ == '__main__':
    main()
