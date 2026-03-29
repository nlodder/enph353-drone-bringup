#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque

class LaserTerrainPlotter:
    def __init__(self, buffer_size=50):
        self.buffer_size = buffer_size
        self.drone_elevation = None

        # store last N scans in a buffer
        self.scan_buffer = deque(maxlen=buffer_size)
        rospy.init_node('drone_down_las_viewer', anonymous=True)

        # setup plotting
        plt.ion() # interactive mode
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Subscribe to the laser topic defined Gazebo plugin
        self.window_name = f"Down Laser Feed: {rospy.get_namespace()}"
        self.laser_sub = rospy.Subscriber("down_laser/scan", LaserScan, self.data_callback)
        rospy.loginfo("Drone Laser Node Initialized. Waiting for scans...")

    def data_callback(self, data):
        # Convert LaserScan to numpy array
        ranges = np.array(data.ranges)
        self.drone_elevation = data.ranges[len(ranges) // 2] # assume middle beam is directly below drone for elevation reference
        angles = None
        if hasattr(data, 'angles'):
            angles = np.array(data.angles)
        else:
            angles = np.linspace(data.angle_min, data.angle_max, len(ranges))
        ranges[np.isinf(ranges)] = 0 # replace inf with 0 for plotting

        # correct ranges for their angles to get actual elevations
        # assuming laser is pointing down, we can calculate the height of the terrain at each beam
        heights = ranges * np.cos(angles) # height = range * cos(angle)
        heights = self.drone_elevation - heights # convert to height above ground
        self.scan_buffer.append(heights) # store heights in buffer
    
    def update_plot(self):
        if len(self.scan_buffer) == 0:
            return # not enough data to plot
        
        self.ax.clear()
        # create meshgrid for buffer
        # X index of laser beam, Y is time step, Z is range
        heights = np.array(self.scan_buffer)
        time_steps, beam_count = heights.shape
        X, Y = np.meshgrid(np.arange(beam_count), np.arange(time_steps))

        # plot surface
        self.ax.plot_surface(X, Y, heights, cmap='viridis', edgecolor='none')

        self.ax.set_zlim(0, max(self.drone_elevation, 0.01)) # set z limit for plot based on drone elevation
        self.ax.set_title(self.window_name)
        self.ax.set_xlabel('Laser Beam Index')
        self.ax.set_ylabel('Time Step')
        self.ax.set_zlabel('Range (m)')
        plt.pause(0.01) # pause to update plot"


if __name__ == '__main__':
    plotter = LaserTerrainPlotter()
    while not rospy.is_shutdown():
        plotter.update_plot()