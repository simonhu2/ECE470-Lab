#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.0
beta = 752.21      #10m apart
tx = 0.0
ty = 0.0

# Function that converts image coord to world coord
def IMG2W(col, row):
    pass
    # O_r = 240       #from Figure B.2
    # O_c = 320

    # x_c = (col - O_c) / beta
    # y_c = (O_r - row) / beta        #since y increases when it goes down, we have to flip y

    # x_w = x_c * np.cos(theta) - y_c * np.sin(theta) + tx
    # y_w = x_c * np.sin(theta) + y_c * np.cos(theta) + ty

    # return (x_w, y_w)


# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 50
    params.maxArea = 1000

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.70
    params.maxCircularity = 0.97

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    lower = (5,100,100)     # orange lower
    upper = (15,255,255)   # orange upper

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)



    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================
    # circle1x, circle1y = keypoints[0].pt
    # circle2x, circle2y = keypoints[1].pt

    # pix_dist = np.sqrt((circle2x - circle1x)**2 + (circle2y - circle1y)**2)

    # beta = pix_dist/0.1         #10cm from Appendix B
    # print(f"{beta}")

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
