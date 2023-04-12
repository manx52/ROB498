import cv2
import numpy as np


def point_cloud_processing(self, image, img_header, img, image_publisher, point_cloud_publisher):
    # Find bounding box
    cnts, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cntsSorted = sorted(cnts, key=lambda x: cv2.contourArea(x), reverse=True)
    box = []
    temp_src = image
    black_canvas = image
    for i, region in enumerate(cntsSorted):
        area = cv2.contourArea(cntsSorted[i])

        if area > 1000:
            print("Area: ", area, " number ", i )
            #cv2.drawContours(temp_src, cntsSorted, i, (255, 0, 0), 2)
            x, y, w, h = cv2.boundingRect(region)

            boundingBoxes = [[x, y + h], [x + w, y]]


            cv2.rectangle(temp_src, (x, y), (x + w, y + h), (0, 0, 255), 2)
            black_canvas = np.zeros_like(temp_src)
            cv2.rectangle(black_canvas, (x, y), (x + w, y + h), (0, 255, 255), cv2.FILLED)
            newImage = cv2.bitwise_and(temp_src, black_canvas)
            hsv2 = cv2.cvtColor(src=newImage, code=cv2.COLOR_BGR2HSV)
            green_only = cv2.inRange(hsv2, (35, 85, 0), (115, 255, 255))

            #print(cv2.countNonZero(green_only),cv2.countNonZero(red_only))
            if cv2.countNonZero(green_only) > 0:
                color = (0, 255, 0)
            else:
                color = (0, 0, 255)
            cv2.rectangle(temp_src, (x, y), (x + w, y + h), color, 2)
            # cv2.imshow('hsv2', hsv2)
            # cv2.imshow('green_only', green_only)
            # cv2.imshow('red_only', red_only)
            # cv2.waitKey(0)

    cv2.imshow('image_rectangle', temp_src)
    cv2.imshow('black_canvas', black_canvas)
    cv2.waitKey(0)




debug = True
image = cv2.imread('../../images/img_color_1.jpeg')
image = cv2.resize(image, (640, 480))
image_crop_blurred = cv2.bilateralFilter(image, 9, 75, 75)

hsv = cv2.cvtColor(src=image_crop_blurred, code=cv2.COLOR_BGR2HSV)

if debug:
    # cv2.imshow("CVT Color", image)
    cv2.imshow("image_crop_blurred", image_crop_blurred)
    cv2.imshow('HSV', hsv)
# yellow (240, 200, 80) RGB
# Grass Mask
# Hue > 115 needed
yellow_only = cv2.inRange(hsv, (0, 180, 190),
                          (40, 235, 255))  # cv2.inRange(image_crop_blurred, (0, 130, 210), (100, 220, 255))
# yellow_only = cv2.inRange()
# print(hsv[540, 1820, :])  # bright yellow [ 25 200 247]
# print(hsv[3487, 1225, :])  # dark yellow [ 21 215 210]
if debug:
    cv2.imshow('Yellow mask', cv2.inRange(hsv, (0, 180, 190), (40, 235, 255)))  # (210, 180, 0), (254, 220, 255

point_cloud_processing(True, image_crop_blurred, True, yellow_only, True, True)
