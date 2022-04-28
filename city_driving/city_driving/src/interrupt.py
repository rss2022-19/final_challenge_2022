#!/usr/bin/env python2

import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Polygon, Point32

from threading import Lock

class InterruptNode:
    TRIGGER_HEIGHT = 0 # (px) height of detected carwash image to switch mode
    SCREEN_WIDTH = 0
    TRIGGER_EPSILON = 0 # (px) epsilon between center of width and midpoint of cw detected rect to activate
    # carwash mode
    def __init__(self):
        # self.wf_sub = rospy.Subscriber("/wf_drive", AckermannDriveStamped, self.wf_callback)
        self.lf_sub = rospy.Subscriber("/lf_drive", AckermannDriveStamped, self.lf_callback)
        self.cw_sub = rospy.Subscriber("/car_wash_rect", Polygon, self.car_wash_callback)
        # self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        # self.bridge = CvBridge()

        self.mode = "normal" # "normal", "stopsign", "carwash"
        self.mode_lock = Lock()

        DRIVE_TOPIC = rospy.get_param("~drive_topic")
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)

    def car_wash_callback(self, polygon):
        topleft = np.array([polygon[0].x, polygon[0].y])
        bottomright = np.array([polygon[1].x, polygon[1].y])
        midpoint = (bottomright + topleft) / 2.0 # in px
        height = (bottomright - topleft)[1] # in px

        with self.mode_lock:
            mode = self.mode

        if mode == "normal":
            if height > self.TRIGGER_HEIGHT and abs(midpoint[0] - self.SCREEN_WIDTH // 2) < self.TRIGGER_EPSILON:
                with self.mode_lock:
                    self.mode = "carwash"
        elif mode == "carwash":
            drive = AckermannDriveStamped()
            # TODO Do the drive do (in a straight line)
            self.drive.publish(drive)
            if height < self.TRIGGER_HEIGHT: # No detect == empty rect
                with self.mode_lock:
                    self.mode = "normal"


    def lf_callback(self, drive):
        with self.mode_lock:
            mode = self.mode

        if mode == "normal":
            self.drive_pub.publish(drive) # Forward linefollower in normal mode

    def image_callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")



if __name__ == "__main__":
    rospy.init_node("interrupt")
    inter = InterruptNode()
    rospy.spin()
