# Introduction

Contributors: Lindsay Burke, Johnnuel Magno

For use at NJIT's Robotics Class

This git repository features code libraries and tutorials for the Hiwonder Arm Pi Pro. Initially it utilized ROS, but the robot itself isn't robust enough to properly utilize it, so the code was abstracted.

The current libraries created are MecanumChasis and Lidar. 

## MecanumChasis Library

The MecanumChasis Library focuses on the motor drive control of the Arm Pi Pro robot. Below lists the functions in the library and how to utilize them. 
The special thing about the Arm Pi Pro is that it utilizes mecanum wheels, meaning that the robot can strafe. 
Strafing means the robot keeps the same oreintation while being able to move in different directions. 
This makes it perfect for applications where tight turns cannot easily be made, such as in warehouses.

A key note about mecanum wheels is that you want to make sure they operate in a smooth enivronment as the design of the wheels can easily get jammed due to debris.
Below are the functions provided in the library. Proper manipulation of the arguments/parameters in the functions are necessary to acchieve specific robot movements.

```
set_motor_speeds(self, speeds)
# Self refers to the object itself, the editable parameter/argument in the speeds.
# Speed ranges from -100 to 100, determines how fast the wheels move and the robot's direction

stop_motors(self)
# Stops the motors, setting them to zero

drive(self, speed=100, angle=0, duration=2)
# Makes the robot drive, and has three parameters: speed, angle and duration.
# Speed determines the speed of the motors and the direction they spin (in mm/s).
# Angle plays into the strafeing aspect, can make the robot move diagonally (in degrees).
# Duration is how long the robot moves (seconds).

drive_xy(self, forward=0, strafe=0, rotation=0, base_speed=60)
# Continuous drive for keyboard/joystick control.
# Use with keyboard_control.py for WASD + arrow key teleop.
```

### Keyboard Control

Run `python keyboard_control.py` to drive the robot with:
- **WASD**: Move and strafe (W=forward, A=left, S=back, D=right)
- **Q/E**: Zero-point turn (Q=left, E=right)
- **Esc** or **Ctrl+C**: Stop and exit

Works over SSH (reads from terminal stdin).

### Example Codes for MecanumChasis Library

### Recommended Practice Problems for MecanumChasis Library

## Lidar Library

To implement more robust autonomous navigation onto the robot, a LIDAR will be used.
A LiDAR(Light Detection and Ranging) is a specialized sensor that uses light in the form of pulsed lasers to measure distances. Most LiDARs have a built-in IMU to calculate distance traveled and depth.
Using this data, it can accurately create a 3D map of its surroundings. 
These sensors feature a rotating motor inside, enabling them to capture a 360-degree view of their surroundings. 
Functionally, you can think of it as a Hypersonic sensor; conceptually, though, they work on completely different physical mediums.

The LiDAR that is used for this robot is the Hokuyo UST-10LX. It utilizes two cables, one for power and one for communication. 
Since the LiDAR requires more voltage than the Raspberry Pi can provide, it is connected to an external Lithium Ion battery pack to supply the power.
It also has a python library, which has been adapted for the code library used for this robot.

The main motivation for implementing a LiDAR is for SLAM (Simultaneous Localization and Mapping) methodology. 
This allows the robot to create a 3D map using the LiDAR data and localize itself on the map at the same time. 
Essentially, the LiDAR serves as the robot's eyes, enabling it to navigate through various terrains by using data about its surroundings.
