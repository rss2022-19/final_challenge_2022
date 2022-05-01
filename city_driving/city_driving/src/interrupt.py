#!/usr/bin/env python2

import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Float32MultiArray

from threading import Lock


class InterruptNode:
    TRIGGER_HEIGHT = 10  # (px) height of detected carwash image to switch mode
    SCREEN_WIDTH = 0
    # (px) epsilon between center of width and midpoint of cw
    # detected rect to activate carwash mode
    TRIGGER_EPSILON = 5

    # (px) height of detected stop sign to enter stopsign mode
    SS_TRIGGER_HEIGHT = 70
    # (px) DO WE NEED A STOPSIGN EPSILON?
    # (sec) Amount of time to wait before detecting stop signs again
    SS_TIME_EPSILON = 5

    def __init__(self):
        # self.wf_sub = rospy.Subscriber("/wf_drive", AckermannDriveStamped, self.wf_callback)
        self.lf_sub = rospy.Subscriber(
            "/lf_drive", AckermannDriveStamped, self.lf_callback)
        self.cw_sub = rospy.Subscriber(
            "/car_wash_rect", Polygon, self.car_wash_callback)
        # self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        # self.bridge = CvBridge()
        self.ss_sub = rospy.Subscriber(
            "/stop_sign_bbox", Float32MultiArray, self.stop_sign_callback)

        self.mode = "normal"  # "normal", "stopsign", "carwash"
        self.mode_lock = Lock()

        DRIVE_TOPIC = rospy.get_param("~drive_topic")
        self.drive_pub = rospy.Publisher(
            DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)

        self.stop_sign_time = 0
        self.last_stop_sign_check = 0
        self.stop_sign_disabled = False

    def car_wash_callback(self, polygon):
        topleft = np.array([polygon[0].x, polygon[0].y])
        bottomright = np.array([polygon[1].x, polygon[1].y])
        midpoint = (bottomright + topleft) / 2.0  # in px
        height = (bottomright - topleft)[1]  # in px

        with self.mode_lock:
            mode = self.mode

        if mode == "normal":
            if height > self.TRIGGER_HEIGHT and abs(midpoint[0] - self.SCREEN_WIDTH // 2) < self.TRIGGER_EPSILON:
                with self.mode_lock:
                    self.mode = "carwash"
        elif mode == "carwash":
            drive_command = AckermannDriveStamped()
            # TODO Do the drive do (in a straight line)
            drive_command.header.stamp = rospy.Time.now()
            drive_command.drive.steering_angle = 0
            drive_command.drive.speed = 1
            self.drive_pub.publish(drive_command)
            if height < self.TRIGGER_HEIGHT:  # No detect == empty rect
                with self.mode_lock:
                    self.mode = "normal"

    def lf_callback(self, drive):
        with self.mode_lock:
            mode = self.mode
        if mode == "normal":
            # Forward linefollower in normal mode
            self.drive_pub.publish(drive)

    def stop_sign_callback(self, bbox):
        topleft = (bbox.data[0], bbox.data[1])
        bottomright = (bbox.data[2], bbox.data[3])

        midpoint = (bottomright + topleft) / 2.0  # in px
        height = (bottomright - topleft)[1]  # in px

        if self.stop_sign_disabled:
            now = rospy.get_time()
            if now - self.last_stop_sign_check > self.SS_TIME_EPSILON:
                self.stop_sign_disabled = False
            else:
                return

        with self.mode_lock:
            mode = self.mode

        if mode == "normal":
            if height > self.SS_TRIGGER_HEIGHT:
                with self.mode_lock:
                    mode = "stopsign"
                # Stop the car!
                drive = AckermannDriveStamped()
                drive.header.stamp = rospy.Time.now()
                drive.drive.speed = 0
                self.drive_pub.publish(drive)
                # Set to now
                self.last_stop_sign_check = rospy.get_time()
        elif mode == "stopsign":
            elapsed = rospy.get_time() - self.last_stop_sign_check
            self.stop_sign_time += elapsed
            if self.stop_sign_time >= 1.0:  # 1 second has passed
                with self.mode_lock:
                    mode = "normal"
                self.stop_sign_disabled = True
        else:
            pass

    def image_callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")


if __name__ == "__main__":
    rospy.init_node("interrupt")
    inter = InterruptNode()
    rospy.spin()
