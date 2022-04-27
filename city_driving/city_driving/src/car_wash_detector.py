#!/usr/bin/env python2

import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Polygon, Point32

from sensor_msgs.msg import Image

class CarWashDetector:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.rect_pub = rospy.Publisher("/car_wash_rect", Polygon, queue_size=1)
        # We bash Polygon into the shape we want
        # We will only pay attention to Point32.{x, y}
        # We assume that there are only 2 points in the polygon: topleft, bottomright
        self.bridge = CvBridge()
    
    def image_callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # TODO Do color segmentation to get the carwash image



if if __name__ == "__main__":
    rospy.init_node('car_wash_detector')
    cwd = CarWashDetector()
    rospy.spin()