#!/usr/bin/env python3
import os
import time

from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE
from tf import TransformBroadcaster

from drone_perception.transformation import Transformation

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, Header

from drone_perception.detector import Detector


class DetectorObstacles(Detector):
    def __init__(self):
        super().__init__()

        # self.initial_pose_subscriber = rospy.Subscriber("initialpose", PoseWithCovarianceStamped,
        #                                                 self.initial_pose_callback, queue_size=1)
        self.image_subscriber = rospy.Subscriber(
            rospy.get_param("/detector_obstacles/camera_topic") + "image_raw", Image, self.image_callback, queue_size=1,
            buff_size=DEFAULT_BUFF_SIZE * 64
        )  # Large buff size (https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/)

        self.bounding_box_publisher = rospy.Publisher("camera/bounding_box", Image, queue_size=1)
        self.green_mask_publisher = rospy.Publisher("camera/green_mask", Image, queue_size=1)
        self.red_mask_publisher = rospy.Publisher("camera/red_mask", Image, queue_size=1)
        self.green_mask_point_cloud_publisher = rospy.Publisher("green_mask_point_cloud", PointCloud2, queue_size=1)
        self.red_mask_point_cloud_publisher = rospy.Publisher("red_mask_point_cloud", PointCloud2, queue_size=1)
        self.tf_broadcaster = TransformBroadcaster()

        # TODO tune
        self.point_cloud_max_distance = rospy.get_param("point_cloud_max_distance", 15)
        self.point_cloud_spacing = rospy.get_param("point_cloud_spacing", 30)
        self.publish_point_cloud = True

        cv2.setRNGSeed(12345)
        pass

    # def initial_pose_callback(self, initial_pose: PoseWithCovarianceStamped):
    #     self.publish_point_cloud = True

    def image_callback(self, img: Image, debug=False):

        t_start = time.time()

        if not self.camera.ready():
            return

        # # Uncomment for ground truth
        rospy.loginfo_once("Started Publishing Obstacles")

        image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")

        image_crop_blurred = cv2.bilateralFilter(image, 9, 75, 75)

        hsv = cv2.cvtColor(src=image_crop_blurred, code=cv2.COLOR_BGR2HSV)

        if debug:
            cv2.imshow("CVT Color", image)
            cv2.imshow("CVT Color Contrast", image_crop_blurred)
            cv2.waitKey(0)

        # Grass Mask
        # Hue > 115 needed
        green_only = cv2.inRange(hsv, (35, 85, 0), (115, 255, 255))
        red_only = cv2.inRange(hsv, (100, 70, 50), (180, 255, 255))

        self.point_cloud_processing(image, img.header, green_only, self.green_mask_publisher,
                                    self.green_mask_point_cloud_publisher)
        self.point_cloud_processing(image, img.header, red_only, self.red_mask_publisher,
                                    self.red_mask_point_cloud_publisher)

        if self.bounding_box_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(image)
            img_out.header = img.header
            self.bounding_box_publisher.publish(img_out)

        t_end = time.time()
        rospy.loginfo_throttle(60, "Obstacle detection rate: " + str(t_end - t_start))

    def point_cloud_processing(self, image, img_header, img, image_publisher, point_cloud_publisher):
        # Find bounding box
        cnts, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        box = []
        for i, region in enumerate(cnts):
            x, y, w, h = cv2.boundingRect(region)
            boundingBoxes = [[x, y + h], [x + w, y]]
            # msg = "boundingBoxes {index:} "
            # rospy.loginfo_throttle(1, msg.format(
            #     index=boundingBoxes))
            # # print(self.camera.calculateBallFromBoundingBoxes(0.3,boundingBoxes).position)
            # msg = "Obstacle Position {index:} "
            # rospy.loginfo_throttle(1, msg.format(
            #     index=self.camera.calculateBallFromBoundingBoxes(0.3, boundingBoxes).position))
            # area = cv2.contourArea(cnts[i])
            # cv2.drawContours(image, cnts, i, (255, 0, 0), 3)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            box.append(self.camera.calculateBallFromBoundingBoxes(0.3, boundingBoxes))

        if image_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(img)
            img_out.header = img_header
            image_publisher.publish(img_out)

        if self.publish_point_cloud and point_cloud_publisher.get_num_connections() > 0:
            points3d = []

            for pt in box:

                # Exclude points too far away
                if pt.norm_squared < self.point_cloud_max_distance ** 2:  # TODO remove lines behind
                    points3d.append(pt.position)

            # Publish fieldlines in laserscan format
            header = Header()
            header.stamp = img_header.stamp
            header.frame_id = "map"
            point_cloud_msg = pcl2.create_cloud_xyz32(header, points3d)
            point_cloud_publisher.publish(point_cloud_msg)


if __name__ == "__main__":
    rospy.init_node("detector_obstacles")
    obstacle_detector = DetectorObstacles()
    rospy.spin()
