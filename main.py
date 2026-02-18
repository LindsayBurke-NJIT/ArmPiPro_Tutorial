import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from direct_drive import MecanumChassis
from lidar import Lidar
from slam import SLAM
#######################################################

def main():
    chassis = None
    lidar = None
    slam = None
    laser = None
    
    try:
        chassis = MecanumChassis()
        lidar = Lidar()

        try:
            slam = SLAM(resolution=0.05)
            laser = lidar.get_laser()
            
            if laser is None:
                print("Error: Failed to get laser instance")
                return
            
            print("Starting SLAM...")
            success_count = 0
            for i in range(200):
                try:
                    if slam.step(laser):
                        success_count += 1
                        x, y, t = slam.get_pose()
                        if i % 10 == 0:
                            print(f"Scan {i}: pose x={x:.2f} y={y:.2f} theta={t:.3f}")
                    else:
                        print(f"Warning: Scan {i} processing failed")
                except RuntimeError as e:
                    print(f"SLAM error at iteration {i}: {e}")
                    continue
                except KeyboardInterrupt:
                    print("\nInterrupted by user")
                    break
                except Exception as e:
                    print(f"Unexpected error at iteration {i}: {e}")
                    continue
            
            print(f"\nSLAM completed: {success_count}/{i+1} scans processed successfully")
            
            if slam is not None:
                try:
                    map_prob = slam.get_map_prob()
                    print(f"Map generated: shape={map_prob.shape}")
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
