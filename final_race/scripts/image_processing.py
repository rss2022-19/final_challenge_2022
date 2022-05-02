import cv2
import numpy as np
import matplotlib.pyplot as plt

def find_goal_point(rgb_img, display=False, ret_image=True):
    """
    returns: (u, v) pixel point for goal to follow 
    """

    COLOR_POINT = (0, 255, 0)
    COLOR_LINES = (0, 0, 255)
    
    rbg_img_copy = rgb_img.copy()
    
    ## THRESHOLDING
    # create a zero array
    stencil = np.zeros_like(rgb_img[:,:,0])
    # specify coordinates of the polygon
    polygon = np.array([np.array([[0,240], [0,175], [672,175], [672,240]])])
    # fill polygon with ones
    cv2.fillConvexPoly(stencil, polygon, 1)
    cropped_rgb_img = cv2.bitwise_and(rgb_img, rgb_img, mask=stencil)
    thresh, ret = cv2.threshold(cv2.cvtColor(cropped_rgb_img, cv2.COLOR_RGB2GRAY), 165, 200, cv2.THRESH_BINARY)

    ## HOUGH TRANSFORM TO GET LINES
    lines = cv2.HoughLinesP(ret, 1, np.pi/180, 35, maxLineGap=200)
    #print("hough lines:", len(lines))
    lines_filtered = []
    
    pos_lines_info = []
    neg_lines_info = []
    
    # create a copy of the original frame
    # draw Hough lines
    for line_idx, line in enumerate(lines):
      x1, y1, x2, y2 = line[0]
      m = float(y2-y1)/float(x2-x1)
      if (abs(m) > 0.25):
          lines_filtered.append(line)
          cv2.line(rbg_img_copy, (x1, y1), (x2, y2), COLOR_LINES, 3)
          b = y1 - m*x1
        
          if m > 0:
              pos_lines_info.append((m, b, line_idx))
          else:
              neg_lines_info.append((m, b, line_idx))
            
    #print("pos_lines_info:", len(pos_lines_info))
    #print("neg_lines_info:", len(neg_lines_info))
    Y_TOP = 200

    pos_lines_intersections = []
    neg_lines_intersections = []

    for i, tup in enumerate(pos_lines_info):
        if (((Y_TOP - tup[1])/tup[0]) > (672*1.0/3.0)):
            pos_lines_intersections.append((tup[2], (Y_TOP - tup[1])/tup[0]))

    for i, tup in enumerate(neg_lines_info):
        if (((Y_TOP - tup[1])/tup[0]) < (672*2.0/3.0)):
            neg_lines_intersections.append((tup[2], (Y_TOP - tup[1])/tup[0]))

    #pos_lines_intersections = [(tup[2], (Y_TOP - tup[1])/tup[0]) if (((Y_TOP - tup[1])/tup[0]) < (672/2))  for i, tup in enumerate(pos_lines_info)]
    #neg_lines_intersections = [(tup[2], (Y_TOP - tup[1])/tup[0])  if (((Y_TOP - tup[1])/tup[0]) > (672/2)) for i, tup in enumerate(neg_lines_info)]

    pos_lines_intersections.sort(key=lambda x: x[1])
    neg_lines_intersections.sort(key=lambda x: x[1])

    print("pos_lines_intersections:", len(pos_lines_intersections))
    print("neg_lines_intersections:", len(neg_lines_intersections))
    
    pos_line_ind = pos_lines_intersections[0][0]
    neg_line_ind = neg_lines_intersections[-1][0]
    pos_line = lines[pos_line_ind][0]
    neg_line = lines[neg_line_ind][0]

    cv2.line(rbg_img_copy, (pos_line[0], pos_line[1]), (pos_line[2], pos_line[3]), (0, 0, 0), 3)
    cv2.line(rbg_img_copy, (neg_line[0], neg_line[1]), (neg_line[2], neg_line[3]), (0, 0, 0), 3)

    new_lines_filtered = [lines[pos_line_ind], lines[neg_line_ind]]
    #new_lines_filtered = []
    y_lookahead = 200
    sumx = 0
    for line in new_lines_filtered:
        x1, y1, x2, y2 = line[0]
        m = float(y2-y1)/float(x2-x1)
        b = y2-(m*x2)
        x_lookahead = (y_lookahead-b)/m
        sumx += x_lookahead
    
    avgx = int(sumx/(len(new_lines_filtered)))
    avgy = int(y_lookahead)
    
    cv2.circle(rbg_img_copy, (avgx, avgy), 5, COLOR_POINT, 3)
    
    if display==True:
        plt.imshow(rbg_img_copy)
        plt.show()
    if ret_image==True:
        return (avgx, avgy), rbg_img_copy

    return (avgx, avgy)

# def find_goal_point(rgb_img, display=False, ret_image=False):
#     """
#     returns: (u, v) pixel point for goal to follow 
#     """

#     COLOR_POINT = (0, 255, 0)
#     COLOR_LINES = (0, 0, 255)
    
#     ## THRESHOLDING
#     # create a zero array
#     stencil = np.zeros_like(rgb_img[:,:,0])
#     # specify coordinates of the polygon
#     top_polygon = np.array([[0, 376], [0, 175], [672, 175], [672, 376]])
#     bottom_polygon = np.array([[0, 376], [0, 240], [672, 240], [672, 376]])

#     middle_polygon = np.array([[100, 376], [250, 200], [400,200], [550, 376]])
#     # fill polygon with ones
#     cv2.fillConvexPoly(stencil, top_polygon, 1)

#     cv2.fillConvexPoly(stencil, bottom_polygon, 0)
#     cropped_rgb_img = cv2.bitwise_and(rgb_img, rgb_img, mask=stencil)
#     # apply image thresholding
#     thresh, ret = cv2.threshold(cv2.cvtColor(cropped_rgb_img, cv2.COLOR_RGB2GRAY), 165, 200, cv2.THRESH_BINARY)

#     ## HOUGH TRANSFORM TO GET LINES
#     lines = cv2.HoughLinesP(ret, 1, np.pi/180, 30, maxLineGap=200)
#     lines_filtered = []
#     # create a copy of the original frame
#     # dmy = image[:,:,0].copy()
#     rbg_img_copy = rgb_img.copy()
#     # draw Hough lines
#     for line in lines:
#       x1, y1, x2, y2 = line[0]
#       m = (float(y2-y1)/float(x2-x1))
#       #print("line[0]:", line[0])
#       #print("y2-y1:", y2-y1)
#       #print("x2-x1:", x2-x1)
#       #print("m:", m)
#       if (abs(m) > 0.5):
#           lines_filtered.append(line)
#           cv2.line(rbg_img_copy, (x1, y1), (x2, y2), COLOR_LINES, 3)
    
    

# #     # PREVIOUS CODE
#     y_lookahead = 200
# #     sumx = 0
# #     for line in lines_filtered:
# #         x1, y1, x2, y2 = line[0]
# #         m = (y2-y1)/(x2-x1)
# #         b = y2-(m*x2)
# #         x_lookahead = (y_lookahead-b)/m
# #         sumx += x_lookahead
    
# #     avgx = sumx/(len(lines_filtered))
# #     test_image = rbg_img_copy.copy()
# #     cv2.circle(test_image, (int(avgx), y_lookahead), 5, (0, 0, 255), 3)
# #     #########
    

#     ## GET LANE POINT WITH LOOKAHEAD RANGE
#     #y_lookahead_min, y_lookahead_max = 255, 256
#     #y_lookahead_range = range(y_lookahead_min, y_lookahead_max)
    
#     sumx = 0
#     neg_x_lookahead = None
#     pos_x_lookahead = None
#     #for y_lookahead in y_lookahead_range:
#     for line in lines_filtered:
#         x1, y1, x2, y2 = line[0]
#         m = float(y2-y1)/float(x2-x1)
#         if m < 0 and neg_x_lookahead is None:
#             b = y2-(m*x2)
#             neg_x_lookahead = (y_lookahead-b)/m
#         elif m > 0 and pos_x_lookahead is None:
#             b = y2-(m*x2)
#             pos_x_lookahead = (y_lookahead-b)/m
#         # sumx += x_lookahead
#         if neg_x_lookahead is not None and pos_x_lookahead is not None:
#             break

    
#     #avgx = int(sumx/(len(lines_filtered))) #* (y_lookahead_max-y_lookahead_min)))
#     avgx = int((neg_x_lookahead + pos_x_lookahead)/2)
#     avgy = int(y_lookahead) #int((y_lookahead_min+y_lookahead_max) / 2.0)
    
#     cv2.circle(rbg_img_copy, (avgx, avgy), 5, COLOR_POINT, 3)
    
#     if display==True:
#         plt.imshow(rbg_img_copy)
#         plt.show()
#     if ret_image==True:
#         return (avgx, avgy), rbg_img_copy
    
#     return (avgx, avgy)
