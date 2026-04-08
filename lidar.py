import sys
import subprocess
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from hokuyolx import HokuyoLX

#LiDAR Ethernet on ArmPi (set once here or via persistent netplan/dhcpcd on the robot)
_LIDAR_IFACE = "end0"
_LIDAR_CIDR = "192.168.0.15/24" #this is the default for the Hokuyo LX-10 lidar model

def ensure_lidar_network():
    """On Linux, add the static IP for the Hokuyo if it is not already configured."""
    if sys.platform != "linux":
        return
    try:
        show = subprocess.run(
            ["ip", "-4", "addr", "show", "dev", _LIDAR_IFACE],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if show.returncode == 0 and "192.168.0.15" in show.stdout:
            return
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return

    add = ["ip", "addr", "add", _LIDAR_CIDR, "dev", _LIDAR_IFACE]
    for cmd in (add, ["sudo", "-n"] + add):
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if r.returncode == 0:
                return
        except FileNotFoundError:
            continue
    print(
        "Warning: could not add LiDAR IP. Run once: "
        f"sudo ip addr add {_LIDAR_CIDR} dev {_LIDAR_IFACE}\n"
        "Or allow passwordless sudo for that command, or run this script as root."
    )

#######################################################
class Lidar:
    def __init__(self):
        try:
            self.laser = HokuyoLX()
        except Exception as e:
            print(f"Error initializing HokuyoLX: {e}")
            raise

    def update(self, laser, plot, text):
        try:
            if laser is None:
                raise ValueError("Laser is None")
            timestamp, scan = laser.get_filtered_dist(dmax=10000)
            if scan is None or scan.size == 0:
                return
            if scan.ndim != 2 or scan.shape[1] < 2:
                print(f"Warning: Invalid scan shape: {scan.shape}")
                return
            plot.set_data(*scan.T)
            text.set_text('t: %d' % timestamp)
            plt.draw()
            plt.pause(.001)
        except Exception as e:
            print(f"Error in lidar update: {e}")

    def get_laser(self):
        if self.laser is None:
            raise ValueError("Laser not initialized")
        return self.laser

    def run(self):
        laser = None
        try:
            plt.ion()
            laser = self.get_laser()
            if laser is None:
                raise ValueError("Failed to get laser instance")
            
            ax = plt.subplot(111, projection='polar')
            plot = ax.plot([], [], '.')[0]
            text = plt.text(0, 1, '', transform=ax.transAxes)
            ax.set_rmax(10000)
            ax.grid(True)
            plt.show()
            
            while plt.get_fignums():
                try:
                    self.update(laser, plot, text)
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"Error in update loop: {e}")
                    continue
        except KeyboardInterrupt:
            print("\nLidar visualization interrupted")
        except Exception as e:
            print(f"Error in lidar run: {e}")
            import traceback
            traceback.print_exc()
        finally:
            try:
                if laser is not None:
                    laser.close()
            except Exception as e:
                print(f"Error closing laser: {e}")

def main():
    ensure_lidar_network()
    myLidar = Lidar()
    myLidar.run()

if __name__ == "__main__":
    main()
