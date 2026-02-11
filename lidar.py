import sys
import os
import matplotlib.pyplot as plt
from hokuyolx import HokuyoLX

class Lidar:
    def __init__(self):
        pass

    def update(self, laser, plot, text):
        timestamp, scan = laser.get_filtered_dist(dmax=10000)
        plot.set_data(*scan.T)
        text.set_text('t: %d' % timestamp)
        plt.draw()
        plt.pause(.001)

    def run(self):
        try:
            plt.ion()
            laser = HokuyoLX()
            ax = plt.subplot(111, projection='polar')
            plot = ax.plot([], [], '.')[0]
            text = plt.text(0, 1, '', transform=ax.transAxes)
            ax.set_rmax(10000)
            ax.grid(True)
            plt.show
            while plt.get_fignums():
                self.update(laser, plot, text)
            laser.close()
        except Exception as e:
            print(e)

def main():
    myLidar = Lidar()
    myLidar.run()

if __name__ == "__main__":
    main()
