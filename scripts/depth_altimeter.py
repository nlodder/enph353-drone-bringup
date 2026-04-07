#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

class DepthTerrainPlotter:
    def __init__(self):
        self.drone_elevation = 0.0

        rospy.init_node('depth_altimeter', anonymous=True)
        
        self.laser_sub = rospy.Subscriber("depth_sensor/depth/image_raw", Image, self.depth_callback)
        self.elev_pub = rospy.Publisher("altitude", Float64, queue_size=1) # publish altitude for cmd bridge
        rospy.loginfo("Depth Altimeter Node Initialized. Waiting for scans...")
        
        self.bridge = CvBridge()
        self.depth_img = None
        rospy.sleep(0.5)

    def depth_callback(self, data):
        self.depth_img = data
        return
    
    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and self.depth_img is None:
            rate.sleep()

        while not rospy.is_shutdown():
            try:
                loc_img = self.depth_img
                # convert ROS Image to OpenCV format (32-bit float, values in meters)
                depth_image = self.bridge.imgmsg_to_cv2(loc_img, desired_encoding="32FC1")
                h, w = depth_image.shape
                
                # define a Region of Interest in the center of the frame
                roi_size = 40
                r = roi_size // 2
                y1, y2 = max(0, h//2 - r), min(h, h//2 + r)
                x1, x2 = max(0, w//2 - r), min(w, w//2 + r)
                
                roi = depth_image[y1:y2, x1:x2]

                # filter out NaNs and Infs and Zeros/near zeros
                valid_depths = roi[(np.isfinite(roi)) & (roi > 0.0001)]

                if len(valid_depths) > 0:
                    # use MEDIAN to ignore outliers like tree branches or the worker drone
                    self.drone_elevation = np.median(valid_depths)
                    
                    # publish for PID controller
                    self.elev_pub.publish(Float64(self.drone_elevation))
                else:
                    # fallback: check the whole image if the center is empty
                    global_valid = depth_image[(np.isfinite(depth_image)) & (depth_image > 0.0001)]
                    if len(global_valid) > 0:
                        self.drone_elevation = np.median(global_valid)
                        self.elev_pub.publish(Float64(self.drone_elevation))
                    else:
                        rospy.logwarn_throttle(1, "Depth perception lost!!!")

            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")

def main():
    bridge = DepthTerrainPlotter()
    try:
        bridge.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()