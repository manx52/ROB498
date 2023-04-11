import cv2
import numpy as np


def circular_mask( radius: int):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius, radius))
def point_cloud_processing(self, image, img_header, img, image_publisher, point_cloud_publisher):
    # Find bounding box
    cnts, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cntsSorted = sorted(cnts, key=lambda x: cv2.contourArea(x), reverse=True)
    box = []
    temp_src = image
    for i, region in enumerate(cntsSorted):
        area = cv2.contourArea(cntsSorted[i])

        if area > 1000:
            print("Area: ", area, " number ", i )
            cv2.drawContours(temp_src, cntsSorted, i, (255, 0, 0), 2)
            x, y, w, h = cv2.boundingRect(region)

            boundingBoxes = [[x, y + h], [x + w, y]]
            cv2.rectangle(temp_src, (x, y), (x + w, y + h), (0, 0, 255), 2)

    cv2.imshow('image_rectangle', temp_src)
    cv2.waitKey(0)




debug = True
image = cv2.imread('../../images/image_mono_1.jpeg')
image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
image = cv2.resize(image, (640, 480))

# hsv = cv2.cvtColor(src=image_crop_blurred, code=cv2.COLOR_BGR2HSV)
_, thresh = cv2.threshold(image, 50, 255, cv2.THRESH_BINARY_INV)
kernel = np.ones((9,9),np.uint8)
# morph_open = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel)
# morph_close = cv2.morphologyEx(morph_open,cv2.MORPH_OPEN,kernel)
morph_open = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, circular_mask(5))
morph_close = cv2.morphologyEx(morph_open, cv2.MORPH_DILATE, circular_mask(5))
# morph_open = cv2.morphologyEx(morph_close, cv2.MORPH_OPEN, circular_mask(21))
# morph_close = cv2.morphologyEx(morph_open, cv2.MORPH_CLOSE, circular_mask(61))
if debug:

    cv2.imshow("image_crop_blurred", image)
    cv2.imshow('thresh', thresh)
    cv2.imshow("morph_open", morph_open)
    cv2.imshow("morph_close", morph_close)

# yellow (240, 200, 80) RGB
# Grass Mask
# Hue > 115 needed

# yellow_only = cv2.inRange(hsv, (0, 0, 0),
#                           (179,100,130))  # cv2.inRange(image_crop_blurred, (0, 130, 210), (100, 220, 255))
# # yellow_only = cv2.inRange()
# # print(hsv[540, 1820, :])  # bright yellow [ 25 200 247]
# # print(hsv[3487, 1225, :])  # dark yellow [ 21 215 210]
# if debug:
#     cv2.imshow('Yellow mask', cv2.inRange(hsv, (0, 180, 190), (40, 235, 255)))  # (210, 180, 0), (254, 220, 255

point_cloud_processing(True, image, True, morph_close, True, True)
