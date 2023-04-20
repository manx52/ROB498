#!/usr/bin/env python3
import argparse
import os
import time

import cv2
import imutils
import numpy as np
import rospy
from cv_bridge import CvBridge
from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from drone_perception.detector import Detector


class DetectorNumbers(Detector):
    """
    A class that detects obstacles from an image stream.
    """

    def __init__(self):
        """
        Initializes the DetectorObstacles class.
        Large buff size (https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/

        Args:
            None

        Returns:
            None
        """

        super().__init__()
        # Initialize subscribers and publishers
        if not self.sim:
            self.image_mono_subscriber = rospy.Subscriber(
                rospy.get_param("/detector_obstacles/camera_topic_mono") + "_rect", Image, self.image_mono_callback,
                queue_size=1,
                buff_size=DEFAULT_BUFF_SIZE * 64
            )
            self.mono3_publisher = rospy.Publisher(rospy.get_param("/detector_obstacles/camera_topic_mono") + "_rect",
                                                   Image, queue_size=1)
            self.image_mono_subscriber3 = rospy.Subscriber(
                rospy.get_param("/detector_obstacles/camera_topic_mono"), Image, self.image_mono3_callback,
                queue_size=1,
                buff_size=DEFAULT_BUFF_SIZE * 64
            )
        else:
            self.image_mono_subscriber = rospy.Subscriber(
                rospy.get_param("/detector_obstacles/camera_topic_mono"), Image, self.image_mono_callback, queue_size=1,
                buff_size=DEFAULT_BUFF_SIZE * 64
            )
        self.number_pub = rospy.Publisher("rob498_drone_07/numbers", Int16MultiArray, queue_size=1)
        # Initialize random number generator seed
        cv2.setRNGSeed(12345)

    def image_mono3_callback(self, img):
        # Transform to cv2/numpy image

        image = CvBridge().imgmsg_to_cv2(img, desired_encoding="8UC3")

        image = cv2.undistort(image, np.array(self.camera.camera_info.K).reshape((3, 3)),
                              np.array(self.camera.camera_info.D))

        if self.mono3_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
            img_out.header = img.header
            self.mono3_publisher.publish(img_out)

    def image_mono_callback(self, img: Image, debug=False):
        """
        Callback function for image topic subscription.
        Detects obstacles in the image and publishes their corresponding masks and point clouds.

        :param img: Image message received from the camera
        :param debug: Boolean flag for showing debug information
        :return: None
        """

        # Record start time
        t_start = time.time()

        rospy.loginfo_once("Started Publishing Numbers")

        # Convert ROS image message to OpenCV image
        image = CvBridge().imgmsg_to_cv2(img, desired_encoding="bgr8")

        # construct the argument parser and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-t", "--template", help="Path to template image", default='../../template_myhal')
        ap.add_argument("-v", "--visualize", default=False,
                        help="Flag indicating whether or not to visualize each iteration")
        args = vars(ap.parse_args())
        # load the image image, convert it to grayscale, and detect edges

        output_confidence = []
        output_x = []
        output_number = []
        template_dir = os.listdir(args['template'])
        for template_path in template_dir:
            template = cv2.imread(os.path.join(args['template'], template_path))
            template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            template = cv2.Canny(template, 50, 200)
            kernel = np.ones((8, 8), np.uint8)
            template = cv2.dilate(template, kernel, iterations=1)
            # contours, hierarchy = cv2.findContours(template,
            # cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            # cv2.drawContours(template, contours, -1, (0, 0, 0), 2)
            # cv2.imshow('Canny Edges After Contouring', template)
            # cv2.waitKey(0)
            (tH, tW) = template.shape[:2]
            cv2.imshow("Template", template)
            # cv2.waitKey(0)
            # image = cv2.fastNlMeansDenoisingColored(image,None,10,10,7,21)
            # cv2.imshow('Denoised', image)
            # cv2.waitKey(0)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            found = None
            # loop over the scales of the image
            for scale in np.linspace(0.2, 1.0, 20)[::-1]:
                # resize the image according to the scale, and keep track
                # of the ratio of the resizing
                resized = imutils.resize(gray, width=int(gray.shape[1] * scale))
                r = gray.shape[1] / float(resized.shape[1])
                # if the resized image is smaller than the template, then break
                # from the loop
                if resized.shape[0] < tH or resized.shape[1] < tW:
                    break
                # detect edges in the resized, grayscale image and apply template
                # matching to find the template in the image
                edged = cv2.Canny(resized, 50, 200)
                result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF_NORMED)
                (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
                # print(maxVal)
                # check to see if the iteration should be visualized
                if args.get("visualize", False):
                    # draw a bounding box around the detected region
                    clone = np.dstack([edged, edged, edged])
                    cv2.rectangle(clone, (maxLoc[0], maxLoc[1]),
                                  (maxLoc[0] + tW, maxLoc[1] + tH), (0, 0, 255), 2)
                # cv2.imshow("Visualize", clone)
                # cv2.waitKey(0)
                # if we have found a new maximum correlation value, then update
                # the bookkeeping variable
                if found is None or maxVal > found[0]:
                    found = (maxVal, maxLoc, r)
            # unpack the bookkeeping variable and compute the (x, y) coordinates
            # of the bounding box based on the resized ratio
            (maxVal, maxLoc, r) = found
            # print(maxVal)
            (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
            (endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))
            # draw a bounding box around the detected result and display the image
            cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
            # cv2.imshow("Image", image)
            # cv2.waitKey(0)
            # save the candidate
            output_confidence.append(maxVal)
            output_x.append(abs(endX - startX) / 2 + startX)
            output_number.append(int(template_path[0]))

        list1 = ["c", "b", "d", "a"]
        list2 = [2, 3, 1, 4]
        # sort by confidence and take top 4 matches
        zipped_lists = zip(output_confidence, output_x, output_number)
        sorted_pairs = sorted(zipped_lists, reverse=True)
        tuples = zip(*sorted_pairs)
        sorted_confidence, sorted_x, sorted_number = [list(tuple) for tuple in tuples]
        # print(sorted_confidence, sorted_x, sorted_number)

        # sort by x coordinate to get final reading
        top4_x = sorted_x[:4]
        top4_number = sorted_number[:4]
        zipped_lists = zip(top4_x, top4_number)
        sorted_pairs = sorted(zipped_lists, reverse=False)
        tuples = zip(*sorted_pairs)
        final_x, final_number = [list(tuple) for tuple in tuples]
        # print(final_x, final_number)
        temp_msg = Int16MultiArray()
        temp_msg.data = final_number
        self.number_pub.publish(temp_msg)
        # Record end time and log the detection rate
        t_end = time.time()
        rospy.loginfo_throttle(60, "Number detection rate: " + str(t_end - t_start))


if __name__ == "__main__":
    rospy.init_node("detector_number")
    number_detector = DetectorNumbers()
    rospy.spin()
