import cv2
import numpy as np
import matplotlib.pyplot as plt


def find_goal_point(rgb_img, display=False, ret_image=True):
    """
    returns: (u, v) pixel point for goal to follow 
    """

    COLOR_POINT = (0, 255, 0)
    COLOR_LINES = (0, 0, 255)
    
    ## THRESHOLDING
    # create a zero array
    stencil = np.zeros_like(rgb_img[:,:,0])
    # specify coordinates of the polygon
    #polygon = np.array([[-150,376], [280,160], [400,160], [1100,376]])
    polygon = np.array([[0,376], [0,150], [672,150], [672,376]])
    # fill polygon with ones
    cv2.fillConvexPoly(stencil, polygon, 1)
    cropped_rgb_img = cv2.bitwise_and(rgb_img, rgb_img, mask=stencil)
    # apply image thresholding
    thresh, ret = cv2.threshold(cv2.cvtColor(cropped_rgb_img, cv2.COLOR_RGB2GRAY), 165, 200, cv2.THRESH_BINARY)

    ## HOUGH TRANSFORM TO GET LINES
    lines = cv2.HoughLinesP(ret, 1, np.pi/180, 60, maxLineGap=200)
    lines_filtered = []
    # create a copy of the original frame
    # dmy = image[:,:,0].copy()
    rbg_img_copy = rgb_img.copy()
    # draw Hough lines
    for line in lines:
      x1, y1, x2, y2 = line[0]
      m = (y2-y1)/(x2-x1)
      if (abs(m) > 0.25):
          lines_filtered.append(line)
          cv2.line(rbg_img_copy, (x1, y1), (x2, y2), COLOR_LINES, 3)

#     # PREVIOUS CODE
#     y_lookahead = 255
#     sumx = 0
#     for line in lines_filtered:
#         x1, y1, x2, y2 = line[0]
#         m = (y2-y1)/(x2-x1)
#         b = y2-(m*x2)
#         x_lookahead = (y_lookahead-b)/m
#         sumx += x_lookahead
    
#     avgx = sumx/(len(lines_filtered))
#     test_image = rbg_img_copy.copy()
#     cv2.circle(test_image, (int(avgx), y_lookahead), 5, (0, 0, 255), 3)
#     #########
    

    ## GET LANE POINT WITH LOOKAHEAD RANGE
    y_lookahead_min, y_lookahead_max = 255, 256
    y_lookahead_range = range(y_lookahead_min, y_lookahead_max)
    sumx = 0;
    for y_lookahead in y_lookahead_range:
        for line in lines_filtered:
            x1, y1, x2, y2 = line[0]
            m = (y2-y1)/(x2-x1)
            b = y2-(m*x2)
            x_lookahead = (y_lookahead-b)/m
            sumx += x_lookahead
    
    avgx = int(sumx/(len(lines_filtered) * (y_lookahead_max-y_lookahead_min)))

    avgy = int((y_lookahead_min+y_lookahead_max) / 2.0)
    
    cv2.circle(rbg_img_copy, (avgx, avgy), 5, COLOR_POINT, 3)
    
    if display==True:
        plt.imshow(rbg_img_copy)
        plt.show()
    if ret_image==True:
        return (avgx, avgy), rbg_img_copy
    
    return (avgx, avgy)