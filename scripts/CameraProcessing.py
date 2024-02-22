#!/usr/bin/env python3
import cv2
import numpy as np
import warnings
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

def hsv_image(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

def set_range_red(hsv_img):
    # red_lower = np.array([0, 100, 100], np.uint8)
    # red_upper = np.array([10, 255, 255], np.uint8)
    red_lower = np.array([0, 100, 100]).astype(np.uint8)
    red_upper = np.array([10, 255, 255]).astype(np.uint8)
    red_mask = cv2.inRange(hsv_img, red_lower, red_upper)
    return red_mask

def set_range_yellow(hsv_img):
    #Yellow Color
    yellow_lower = np.array([24,107,21]).astype(np.uint8)
    yellow_upper = np.array([180,255,255]).astype(np.uint8)
    yellow_mask = cv2.inRange(hsv_img, yellow_lower, yellow_upper)
    return yellow_mask

def set_range_blue(hsv_img):
    # co the dieu chinh
    # blue_lower = np.array([110,50,50], np.uint8) 
    # blue_upper = np.array([130,255,255], np.uint8) 
    blue_lower = np.array([94, 80, 2]).astype(np.uint8)
    blue_upper = np.array([120, 255, 255]).astype(np.uint8)
    blue_mask = cv2.inRange(hsv_img, blue_lower, blue_upper)
    return blue_mask

def get_contour_by_mask(mask):
    return cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# for color detection 
def color_detector(img):
    
    hsv_img = hsv_image(img)
    # Set range for red color and define mask
    red_mask = set_range_red(hsv_img)
    yellow_mask = set_range_yellow(hsv_img)
    blue_mask = set_range_blue(hsv_img)

    # Morphological Transform, Dilation  for each color and bitwise_and operator 
	# between imageFrame and mask determines to detect only that particular color 
    kernal = np.ones((5, 5), "uint8")

    # for red color
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(img, img,mask=red_mask)
    # # For green color
    yellow_mask = cv2.dilate(yellow_mask, kernal)
    res_yellow = cv2.bitwise_and(img, img,
                                mask=yellow_mask)
    # For blue color
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(img, img, mask=blue_mask)

    # Creating contour to track RED color
    contours_red, hierarchy = get_contour_by_mask(red_mask)

    if len(contours_red) != 0:
        cnt = sorted(contours_red, key=cv2.contourArea, reverse=True)[0]
        area_red = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        if area_red > 1500:
            # center_point_red = [0.5*(x + w), 0.5*(y + h)]
            center_red = np.array([(x + w/2), (y + h/2)])
            img = cv2.rectangle(img, (x, y),
                                        (x + w, y + h),
                                        [0, 255, 255], 2)
            cv2.putText(img, "Red", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 255))
        else:
            center_red = np.array([])
    else:
        center_red = np.array([])

    # Creating contour to track YELLOW color
    contours_yellow, hierarchy = get_contour_by_mask(yellow_mask)
    if len(contours_yellow) != 0:
        cnt = sorted(contours_yellow, key=cv2.contourArea, reverse=True)[0]
        area_yellow = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        # print(f"x: {x}, y: {y}, w: {w}, h: {h}")
        if area_yellow > 1500:
            if h > 135:
                center_yellow = np.array([(x + w/2), (y + h/2)])
                img = cv2.rectangle(img, (x, y),
                                            (x + w, y + h),
                                            [0, 255, 255], 2)
            else:
                center_yellow = np.array([])
        else:
            center_yellow = np.array([])
    else:
        center_yellow = np.array([])

    # Creating contour to track blue color
    contours_blue, hierarchy = get_contour_by_mask(blue_mask)
    if len(contours_blue) != 0:
        # if cv2.contourArea()
        cnt = sorted(contours_blue, key=cv2.contourArea, reverse=True)[0]
        x, y, w, h = cv2.boundingRect(cnt)
        center_blue = np.array([(x + w/2), (y + h/2)])
        img = cv2.rectangle(img, (x, y),
                                    (x + w, y + h),
                                    (0, 0, 255), 2)
        cv2.putText(img, "Blue", (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (255, 0, 0))
    else:
        center_blue = np.array([])
    
    return img, center_blue, center_red, center_yellow

warnings.simplefilter('ignore', np.RankWarning) # Ignore rank warning
np.seterr(divide='warn', invalid='warn')

if __name__ == "__main__":
    imageDir = '../rectange-pics/more/'
    imageFiles = os.listdir(imageDir)
    imageList = []
    for i in range(0, len(imageFiles)):
        image = cv2.imread(imageDir + imageFiles[i])
        # image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        imageList.append(image)

    result = list(map(color_detector, imageList))

    combo_img_avrage = []

    for img, x_error, yaw_error, _ in result:
        combo_img_avrage.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
