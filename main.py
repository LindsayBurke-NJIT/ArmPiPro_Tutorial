import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))    

from direct_drive import MecanumChassis
from lidar import Lidar

def main():
   chassis = MecanumChassis()
   lidar = Lidar()

   lidar.run()

   #use libraries
   #try:
   #    chassis.drive_forward(60, 2)
   #except KeyboardInterrupt:
   #    print("\nStopping due to keyboard interrupt...")
   #finally:
   #    chassis.stop_motors()

if __name__ == "__main__":
    main()
