#!/usr/bin/python3
# coding=utf8
import time
import smbus2

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

def main():
    i2cPort = 1
    chassis = MecanumChassis()
    try:
        # Drive forward at 60mm/s for 2 seconds
        chassis.drive_forward(60, 2)
    except KeyboardInterrupt:
        print("\nStopping due to keyboard interrupt...")
    finally:
        chassis.stop_motors()
        with smbus2.SMBus(1) as bus:
            bus.close()
        print("Motors stopped")

if __name__ == '__main__':
    main()
