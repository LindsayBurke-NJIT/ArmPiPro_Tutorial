import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from direct_drive import MecanumChassis
from lidar import Lidar
from slam import SLAM

def main():
    chassis = MecanumChassis()
    lidar = Lidar()

    #Run SLAM: get scans and build map
    slam = SLAM(resolution=0.05)
    laser = lidar.get_laser()
    try:
        for _ in range(200):
            slam.step(laser)
            x, y, t = slam.get_pose()
            print(f"pose x={x:.2f} y={y:.2f} theta={t:.3f}")
    except KeyboardInterrupt:
        pass
    finally:
        laser.close()
    map_prob = slam.get_map_prob()

if __name__ == "__main__":
    main()
