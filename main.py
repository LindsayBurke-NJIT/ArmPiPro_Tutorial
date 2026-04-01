import os
import sys
import time
import numpy as np
import math
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import Config
from direct_drive import MecanumChassis
from lidar import Lidar
from slam import SLAM
#######################################################

def main():
    cfg = Config()

    chassis = None
    lidar = None
    slam = None
    laser = None
    
    try:
        chassis = MecanumChassis()
        lidar = Lidar()

        try:
            slam = SLAM(resolution=cfg.slam_resolution)
            laser = lidar.get_laser()
            
            if laser is None:
                print("Error: Failed to get laser instance")
                return
            
            print("Starting SLAM with autonomous exploration...")
            print(f"Mode: {cfg.exploration_mode}")
            print("The robot will explore with obstacle avoidance using lidar data.")
            print("Press Ctrl+C to stop.\n")

            success_count = slam.explore_waypoints(chassis, laser, cfg)
            
            print(f"\nSLAM completed: {success_count} scans processed successfully")
            
            if slam is not None:
                try:
                    map_prob = slam.get_map_prob()
                    print(f"Map generated: shape={map_prob.shape}")
                    x, y, theta = slam.get_pose()
                    print(f"Final pose: x={x:.2f}m y={y:.2f}m theta={math.degrees(theta):.1f}°")

                    try:
                        out_path = slam.save_map_visualization(
                            out_dir=cfg.map_out_dir,
                            filename_prefix=cfg.map_filename_prefix,
                            map_prob=map_prob,
                            pose=(x, y, theta),
                            save_npy=cfg.save_map_npy,
                        )
                        print(f"Saved map visualization to: {out_path}")
                    except Exception as e:
                        print(f"Error saving map visualization: {e}")
                except Exception as e:
                    print(f"Error getting map: {e}")
        except RuntimeError as e:
            print(f"SLAM initialization error: {e}")
        except Exception as e:
            print(f"Error during SLAM: {e}")
            import traceback
            traceback.print_exc()

    except KeyboardInterrupt:
        print("\nStopping due to keyboard interrupt...")
    except Exception as e:
        print(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if laser is not None:
                laser.close()
        except Exception as e:
            print(f"Error closing laser: {e}")
        
        try:
            if chassis is not None:
                chassis.stop_motors()
        except Exception as e:
            print(f"Error stopping motors: {e}")

if __name__ == "__main__":
    main()
