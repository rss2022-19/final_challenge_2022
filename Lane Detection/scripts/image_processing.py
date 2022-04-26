import numpy as np
import rospy

import os
import re
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tqdm import tqdm_notebook
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from final_challenge_2022.msg import GoalPixel
from final_challenge_2022.msg import GoalPoint

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
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # create a zero array
        stencil = np.zeros_like(image[:,:,0])
        # specify coordinates of the polygon
        #polygon = np.array([[-150,376], [280,160], [400,160], [1100,376]])
        polygon = np.array([[0,376], [0,150], [672,150], [672,376]])
        # fill polygon with ones
        cv2.fillConvexPoly(stencil, polygon, 1)

        # apply polygon as a mask on the frame
        img = cv2.bitwise_and(image[:,:,0], image[:,:,0], mask=stencil)

        # apply image thresholding
        ret, thresh = cv2.threshold(img, 165, 200, cv2.THRESH_BINARY)

        lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 60, maxLineGap=200)
        lines_filtered = []
        # create a copy of the original frame
        dmy = image[:,:,0].copy()

        # draw Hough lines
        for line in lines:
            x1, y1, x2, y2 = line[0]
        
            m = (y2 - y1) / (x2 - x1)
            if (abs(m) > 0.3):
                lines_filtered.append(line)
                cv2.line(dmy, (x1, y1), (x2, y2), (0, 0, 0), 3)

        goal = self.find_goal(lines_filtered)
        goal_pixel = GoalPixel()
        goal_pixel.u = goal[0]
        goal_pixel.v = goal[1]
        self.goal_pub.publish(goal_pixel)


    def find_goal(self, lines_filtered):
        """
        returns: (u, v) pixel point for goal to follow 
        """
        sumx = 0
        for line in lines_filtered:
            x1, y1, x2, y2 = line[0]
            
            m = (y2 - y1) / (x2 - x1)
            b = y2 - (m * x2)
                
            x_lookahead = (self.lookahead_y - b) / m
            sumx += x_lookahead

        avgx = sumx / (len(lines_filtered))
        return (avgx, self.lookahead_y)