# Additional Resources for ArmPiPro Robot
For more resources on the specific functionalities of the ArmPiPro robot, including how the mecanum drive works, the wiring schematic, and the logic for the programming (including the original ROS-based program) use the following link: [here](https://drive.google.com/drive/folders/1STIYCfLzk3rt_0GHM_-8SaYc4Za9FjiZ?usp=sharing). 

# Introduction

Contributors: Lindsay Burke, Johnnuel Magno

For use at NJIT's Robotics Class

This git repository features code libraries and tutorials for the Hiwonder Arm Pi Pro. Initially it utilized ROS, but ROS code typically includes a convoluted web of topics and nodes that can have a steep learning curve. So, we decided to create a Python library for ease of learning how to interact with the robot and implement Simultaneous Localization and Mapping (SLAM) for mapping environmments using a LiDAR sensor. 

The current libraries created are MecanumChasis and Lidar. 

## MecanumChasis Library

The MecanumChasis Library focuses on the motor drive control of the Arm Pi Pro robot. Below lists the functions in the library and how to utilize them. 
A key feature of the Arm Pi Pro is that it utilizes mecanum wheels, meaning that the robot can strafe. 
Strafing means the robot keeps the same oreintation while being able to move in different directions. 
This makes it perfect for applications where tight turns cannot easily be made, such as in warehouses.

A key note about mecanum wheels is to make sure they operate in a smooth enivronment as the design of the wheels can easily get jammed due to debris.
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
# Continuous drive for keyboard/joystick control. Combine forward, strafe, and rotation.
# Use with keyboard_control.py for WASD + arrow key teleop.
```

### Keyboard Control

Run `python keyboard_control.py` to drive the robot with:
- **WASD**: Move and strafe (e.g. W+D = diagonal forward-right)
- **Arrow keys**: Zero-point turn (rotate in place)
- **Esc** or **Ctrl+C**: Stop and exit

Requires `pynput`: `pip install pynput`

### Example Problem Codes for MecanumChasis Library

1) Make the robot travel in the shape of a diamond.

To solve this problem, you first need to determine the turning (strafe) angles required to form the desired shape. Since the problem does not specify constraints such as side lengths or exact dimensions, the key requirement is that the total exterior turning angle must equal 360° in order for the robot (or path) to return to its starting orientation and complete a closed shape.

Note that 360° is not the sum of the interior angles for most polygons. Instead, 360° is the sum of the exterior angles of any closed polygon, taken one at each vertex.

For this case, we can choose two turning angles: 30° and 150°. These add up to 180°. By repeating this pair twice (30° + 150° + 30° + 150°), the total turning becomes 360°, ensuring the path closes properly.

Once the turning angles are determined, the rest is straightforward implementation: define the appropriate movement and turning functions in the code, then sequence them in the required order to generate the desired shape.
```
# Always remember to import the proper libraries 
from direct_drive import MecanumChassis
import time

# Create chassis object, note you can name the object as anything you want, but make it meaningful.
tsubasa_k = MecanumChassis()

# For most shapes, they are mostly symmetrical.
# This means you can mirror your code so you can have one set moving forward and the other moving backward.
tsubasa_k.drive(60, 30, 2)
tsubasa_k.drive(60, 150, 2)
tsubasa_k.drive(-60, 30, 2)
tsubasa_k.drive(-60, 150, 2)

time.sleep(2)

tsubasa_k.stop_motors()
```
2) Code the robot to travel in the path of a hexagon.

Like the previous problem, the interior angles must be determined so the robot travels in the path of the shape.

The interior angles can be determined by the following formula: [(n-2)*180]/n = interior angle. Then to determine the turn, utilize: 180-interior angle = turn, to get the neccessary turn angles.

For this problem, since there are no constraints on the specified dimensions, a typical hexagon can be assumed, so it would have an interior angle of 120 utilizing the formula (or from memory).

Then subtract it from 180, which results in the angles needed. From there, start from 30 degrees and increment by 60 degrees to acchieve the desired path. Then implement it in the code.

```
# Always remember to import the proper libraries 
from direct_drive import MecanumChassis
import time

# Create chassis object
tsubasa_k = MecanumChassis()

# Lists can be utilize here to condense the code so you don't have to repeatedly copy & paste the function.
# Hexagon angles (60° apart)
angles = [30, 90, 150, 210, 270, 330]

for angle in angles:
    tsubasa_k.drive(60, angle, 2)

time.sleep(1)

tsubasa_k.stop_motors()
```

### General Method for Solving Path Problems

From the practice examples, it is clear that most of these path planning problems relate to application of geometric properties, primarily angles. A step-by-step guide is provided below.

Method:

1) Analyze the Geometry. Identify the shape of the path and note any geometric properties such as symmetry or equal sides.

2) Determine Travel Angles. Figure out the angles or directions the robot needs to move or turn at each segment.

3) Use the Shape as a Guide. Refer to the geometry of the shape to plan rotations, strafing, or directional changes.

4) Test and Calibrate. Experiment with different speeds and time durations to achieve the desired movement path.

5) Apply Kinematic Relationships (if given dimensions). When specific distances or dimensions are provided, use basic displacement and motion formulas to calculate timing or distance traveled.


### Recommended Practice Problems for MecanumChasis Library

1) Code the robot to move in the path of a circle. (hint, you'll need to utilize integration)
2) Code the robot to move in the path of a square.
3) Code the robot to move in the path of an “X”.
4) Code the robot to follow this path:
5) Code the robot to follow this path:
6) Challenge:  Code the robot to follow this path (follow the arrows). Hint: Utilize both strafing and rotation:


## Lidar Library

To implement more robust autonomous navigation onto the robot, a LiDAR will be used.
A LiDAR(Light Detection and Ranging) is a specialized sensor that uses light in the form of pulsed lasers to measure distances. This data collected become points in space. Most LiDARs have a built-in IMU to calculate distance traveled and depth.
Using this data, it can accurately create a 3D map of its surroundings. 
These sensors feature a rotating motor inside, enabling them to capture a 360-degree view of their surroundings. 
Functionally, you can think of it as a Hypersonic sensor; conceptually, though, they work on completely different physical mediums.

The LiDAR that is used for this robot is the Hokuyo UST-10LX. It utilizes two cables, one for power and one for communication. 
Since the LiDAR requires more voltage than the Raspberry Pi can provide, it is connected to an external Lithium Ion battery pack to supply the power.
It also has a python library, which has been adapted for the code library used for this robot.

The main motivation for implementing a LiDAR is for SLAM (Simultaneous Localization and Mapping) methodology. 
This allows the robot to create a 3D map using the LiDAR data and localize itself on the map at the same time. 
Essentially, the LiDAR serves as the robot's eyes, enabling it to navigate through various terrains by using data about its surroundings.

### Sampling Method

Using the LiDAR, the robot moves in a square path to collect data points in its environment space. Alternatively, you can set way point at different points that the robot much reach around the environment if you know the dimensions of the environment you wish to achieve with the sampling. There is built-in object avoidance, so if an object is too close, the robot will move in the direction with the larger distance. 
The quality of the robot's mapping depends on how long it is in sampling operation.

## Configuration
The settings are configurable in ```config.py```.

<h5>Settings for LiDAR obstacle avoidance</h5>

    lidar_mask_angle_intervals_deg: tuple[tuple[float, float], ...] = () #sets the angle intervals to mask for obstacle avoidance (e.g. range where the arm/chassis of robot
                                                                         is to avoid sensing itself)
                                                                         
    lidar_forward_cone_half_width_deg: float     #sets the width of the forward cone for obstacle avoidance
    
    lidar_forward_clearance_percentile: float    #sets the percentile of the forward cone for obstacle avoidance
    
    lidar_emergency_close_mm: float              #sets the distance at which the robot will stop if it is too close to an obstacle
    
    lidar_emergency_debounce_scans: int          #sets the number of scans at which the robot will stop if it is too close to an obstacle (to prevent jittering)


<h5>Settings for SLAM navigation</h5>

    exploration_mode: str = "waypoints"
    
    max_iterations: int = 5000
    
    free_heading_reseed_interval: int = 25
    
    waypoints: tuple[tuple[float, float], ...] = (
        (1.0, 0.0),
        (1.0, 1.0),
        ...
    ) #exploration_mode "free" ignores waypoints at runtime
    
    slam_resolution: float = 0.05
    
    map_out_dir: str = "maps" #output directory for the map created by SLAM
    
    map_filename_prefix: str = "lidar_map" #file name of output map
    
    save_map_npy: bool = True
