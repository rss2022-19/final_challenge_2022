#!/usr/bin/env python

import numpy as np
import rospy

import os
import re
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
# from tqdm import tqdm_notebook
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from final_race.msg import GoalPixel
from final_race.msg import GoalPoint

from image_processing import find_goal_point

class LaneDetector():
    """
    A class for lane detection in the final race.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the goal point in the image frame (units are pixels).
    """
    def __init__(self):
        # Pubish pixel information 
        self.goal_pub = rospy.Publisher("/relative_cone_px", GoalPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)

        # Subscribe to ZED camera RGB frames
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.lookahead_y = rospy.get_param("lookahead_y", 225)

    def image_callback(self, image_msg):
        print("MAKES IT HERE")
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        goal = find_goal_point(image, display=False)

        goal_pixel = GoalPixel()
        goal_pixel.u = goal[0]
        goal_pixel.v = goal[1]
        self.goal_pub.publish(goal_pixel)

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('LaneDetector', anonymous=True)
        LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
