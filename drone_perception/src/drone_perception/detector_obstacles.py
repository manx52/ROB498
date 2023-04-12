#!/usr/bin/env python3
import time

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, Header
from tf import TransformBroadcaster

from drone_common.transformation import Transformation
from drone_perception.detector import Detector


class DetectorObstacles(Detector):
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

        self.bounding_box_publisher = rospy.Publisher("camera/bounding_box", Image, queue_size=1)
        self.green_mask_publisher = rospy.Publisher("camera/green_mask", Image, queue_size=1)
        self.red_mask_publisher = rospy.Publisher("camera/red_mask", Image, queue_size=1)
        self.green_mask_point_cloud_publisher = rospy.Publisher("green_mask_point_cloud", PointCloud2, queue_size=1)
        self.red_mask_point_cloud_publisher = rospy.Publisher("red_mask_point_cloud", PointCloud2, queue_size=1)
        self.tf_broadcaster = TransformBroadcaster()

        # Initialize point cloud parameters and publishing flag
        self.point_cloud_max_distance = rospy.get_param("point_cloud_max_distance", 5)
        self.point_cloud_spacing = rospy.get_param("point_cloud_spacing", 30)
        self.publish_point_cloud = False

        self.add_obs = rospy.get_param("/detector_obstacles/add_obs", False)
        self.add_obs_sub = rospy.Subscriber("add_obs", Bool, self.add_obs_callback)

        # Initialize random number generator seed
        cv2.setRNGSeed(12345)

    def add_obs_callback(self, msg):
        """

        :param msg:
        :return:
        """
        self.add_obs = msg.data

    def image_mono3_callback(self, img):
        # Transform to cv2/numpy image

        image = CvBridge().imgmsg_to_cv2(img, desired_encoding="8UC3")

        image = cv2.undistort(image, np.array(self.camera.camera_info.K).reshape((3, 3)),
                              np.array(self.camera.camera_info.D))
        # image = cv2.rotate(image, cv2.ROTATE_180)

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

        # Check if the camera is ready
        if not self.camera.ready():
            return

        rospy.loginfo_once("Started Publishing Obstacles")
        if self.sim:
            # Convert ROS image message to OpenCV image
            image = CvBridge().imgmsg_to_cv2(img, desired_encoding="rgb8")
        else:
            # Convert ROS image message to OpenCV image
            image = CvBridge().imgmsg_to_cv2(img, desired_encoding="bgr8")

        # Apply bilateral filtering to reduce noise
        image_crop_blurred = cv2.bilateralFilter(image, 9, 75, 75)

        # Convert the image to HSV color space for better color segmentation
        hsv = cv2.cvtColor(src=image_crop_blurred, code=cv2.COLOR_BGR2HSV)

        # Show debug images if debug flag is True
        if debug:
            cv2.imshow("CVT Color", image)
            cv2.imshow("CVT Color Contrast", image_crop_blurred)
            cv2.waitKey(0)

        if self.sim:
            # Create masks for green and red obstacles in the image
            green_only = cv2.inRange(hsv, (35, 85, 0), (115, 255, 255))
            red_only = cv2.inRange(hsv, (100, 70, 50), (180, 255, 255))
            # Publish point clouds and masks for green and red obstacles
            self.point_cloud_processing(image, img.header, green_only, self.green_mask_publisher,
                                        self.green_mask_point_cloud_publisher)
            self.point_cloud_processing(image, img.header, red_only, self.red_mask_publisher,
                                        self.red_mask_point_cloud_publisher)
        else:
            yellow_only = cv2.inRange(hsv, (0, 180, 190), (40, 235, 255))  # (0, 180, 190), (40, 235, 255)

            self.point_cloud_processing(image, img.header, yellow_only, self.red_mask_publisher,
                                        self.red_mask_point_cloud_publisher)
        # Publish bounding box image message if there are subscribers
        if self.bounding_box_publisher.get_num_connections() > 0:
            img_out = CvBridge().cv2_to_imgmsg(image)
            img_out.header = img.header
            self.bounding_box_publisher.publish(img_out)

        # Record end time and log the detection rate
        t_end = time.time()
        rospy.loginfo_throttle(60, "Obstacle detection rate: " + str(t_end - t_start))

    def point_cloud_processing(self, image, img_header, img, image_publisher, point_cloud_publisher):
        """
        Process a point cloud image and publish it to ROS topics.

        :param image: Input image as a numpy array.
        :param img_header: Header information for the input image.
        :param img: Binary image with contours.
        :param image_publisher: ROS publisher for the input image.
        :param point_cloud_publisher: ROS publisher for the output point cloud.
        :return: None.
        """
        # Find bounding box for each contour
        cnts, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        box = []
        obs_color = "red"
        for i, region in enumerate(cnts):
            x, y, w, h = cv2.boundingRect(region)
            boundingBoxes = [[x, y + h], [x + w, y]]

            # remove edge cases
            x_left_cond = x == 0 or x == self.camera.resolution_x
            x_right_cond = (x + w) == 0 or (x + w) == self.camera.resolution_x
            if x_left_cond or x_right_cond:
                continue
            if not self.sim:
                area = cv2.contourArea(cnts[i])

                if area > 500:
                    # Draw bounding box on the input image
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    box.append(self.camera.calculateBallFromBoundingBoxes(0.3, boundingBoxes))

                    black_canvas = np.zeros_like(image)
                    cv2.rectangle(black_canvas, (x, y), (x + w, y + h), (0, 255, 255), cv2.FILLED)
                    newImage = cv2.bitwise_and(image, black_canvas)
                    hsv2 = cv2.cvtColor(src=newImage, code=cv2.COLOR_BGR2HSV)
                    green_only = cv2.inRange(hsv2, (35, 85, 0), (115, 255, 255))

                    # print(cv2.countNonZero(green_only),cv2.countNonZero(red_only))
                    if cv2.countNonZero(green_only) > 0:
                        obs_color = "green"

                    # cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            else:
                # Draw bounding box on the input image
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                box.append(self.camera.calculateBallFromBoundingBoxes(0.3, boundingBoxes))
        if self.sim:
            if len(cnts) > 0:
                self.publish_point_cloud = True
            else:

                self.publish_point_cloud = False

            # Publish the input image if there is any subscriber
            if image_publisher.get_num_connections() > 0:
                img_out = CvBridge().cv2_to_imgmsg(img)
                img_out.header = img_header
                image_publisher.publish(img_out)

            # Create point cloud message and publish it if there is any subscriber
            if self.publish_point_cloud and point_cloud_publisher.get_num_connections() > 0 and self.add_obs:
                points3d = []

                for pt in box:

                    # Exclude points too far away
                    pt_drone_frame = pt.position - self.camera.pose.position
                    pt_transform = Transformation(position=pt_drone_frame)
                    if pt_transform.norm_squared < self.point_cloud_max_distance ** 2:  # TODO remove lines behind
                        points3d.append(pt.position)
                # print("How many", len(points3d))
                if len(points3d) > 0:
                    # Publish point cloud message
                    header = Header()
                    header.stamp = img_header.stamp
                    header.frame_id = "map"
                    point_cloud_msg = pcl2.create_cloud_xyz32(header, points3d)
                    point_cloud_publisher.publish(point_cloud_msg)
        else:
            if len(cnts) > 0:
                self.publish_point_cloud = True
            else:

                self.publish_point_cloud = False

            # Publish the input image if there is any subscriber
            if obs_color == "green":
                image_publisher = self.green_mask_publisher
                point_cloud_publisher = self.green_mask_point_cloud_publisher

            if image_publisher.get_num_connections() > 0:
                img_out = CvBridge().cv2_to_imgmsg(img)
                img_out.header = img_header
                image_publisher.publish(img_out)

            # Create point cloud message and publish it if there is any subscriber
            if self.publish_point_cloud and point_cloud_publisher.get_num_connections() > 0 and self.add_obs:
                points3d = []

                for pt in box:

                    # Exclude points too far away
                    pt_drone_frame = pt.position - self.camera.pose.position
                    pt_transform = Transformation(position=pt_drone_frame)
                    if pt_transform.norm_squared < self.point_cloud_max_distance ** 2:  # TODO remove lines behind
                        points3d.append(pt.position)
                # print("How many", len(points3d))
                if len(points3d) > 0:
                    # Publish point cloud message
                    header = Header()
                    header.stamp = img_header.stamp
                    header.frame_id = "map"
                    point_cloud_msg = pcl2.create_cloud_xyz32(header, points3d)
                    point_cloud_publisher.publish(point_cloud_msg)
