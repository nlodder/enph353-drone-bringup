#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
from std_msgs.msg import Float64

class LaserTerrainPlotter:
    def __init__(self, buffer_size=50):
        self.buffer_size = buffer_size
        self.drone_elevation = 0.0

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
        self.elev_pub = rospy.Publisher("altitude", Float64, queue_size=1) # publish altitude for cmd bridge
        rospy.loginfo("Drone Laser Node Initialized. Waiting for scans...")
        
        rospy.sleep(0.5)

    def data_callback(self, data):
        # Convert LaserScan to numpy array
        ranges = np.array(data.ranges)
        
        # FIX: Filter out Inf/NaN before averaging
        # If the laser hits nothing, it returns 'inf'. We should ignore these for altitude.
        valid_indices = np.where(np.isfinite(ranges))[0]
        
        if len(valid_indices) > 0:
            # Use middle section of the scan for altitude (downward looking)
            mid = len(ranges) // 2
            slice_data = ranges[max(0, mid-20) : min(len(ranges), mid+20)]
            
            # Filter infs from the slice specifically
            valid_slice = slice_data[np.isfinite(slice_data)]
            
            if len(valid_slice) > 0:
                self.drone_elevation = np.mean(valid_slice)
            else:
                # If center is inf but edges are valid, use general mean
                self.drone_elevation = np.mean(ranges[valid_indices])
        
        # Publish for the PID controller
        self.elev_pub.publish(Float64(self.drone_elevation))

        # Prep for plotting: convert Inf to a high value (max range) so the plot doesn't break
        plot_ranges = np.where(np.isinf(ranges), data.range_max, ranges)
        
        # heights = altitude - range (distance from drone to ground)
        heights = self.drone_elevation - plot_ranges
        self.scan_buffer.append(heights)
    
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

        self.ax.set_zlim( 0, min( max(self.drone_elevation, 0.01), 10 ) ) # set z limit for plot based on drone elevation
        self.ax.set_title(self.window_name)
        self.ax.set_xlabel('Laser Beam Index')
        self.ax.set_ylabel('Time Step')
        self.ax.set_zlabel('Range (m)')
        plt.pause(0.01) # pause to update plot"


if __name__ == '__main__':
    plotter = LaserTerrainPlotter()
    # while not rospy.is_shutdown():
    #     plotter.update_plot()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")