#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped
import rospy
from drone.utils import *
from drone_perception import Transformation


class Vicon:
    """
    Handles Vicon data for drone pose estimation.
    """
    def __init__(self, node):
        """
        Constructor for the Vicon class.
        :param node: The ROS node that subscribes to Vicon data.
        """
        self.node = node

        self.vicon_pose_enabled = rospy.get_param("/vicon_enabled") # Check if Vicon data is enabled.
        self.VICON_RECEIVED = False # Flag to indicate if Vicon data has been received.
        self.vicon_transform = None # Placeholder for Vicon transformation object.

        # Create a ROS subscriber for Vicon data.
        self.vicon_sub = rospy.Subscriber("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, self.vicon_callback)

        # Create a ROS publisher for Vicon pose data.
        self.vicon_pose_pub = rospy.Publisher("mavros/vision_pose/pose", PoseStamped, queue_size=10)

    def vicon_callback(self, msg: TransformStamped):
        """
        Callback function for Vicon data subscriber.
        :param msg: The Vicon data message.
        """
        if self.VICON_RECEIVED:
            return # Ignore Vicon data if already received.

        self.VICON_RECEIVED = True # Set flag to indicate Vicon data has been received.
        rospy.loginfo("Vicon Received")

        # Extract roll, pitch, and yaw from Vicon quaternion.
        q = msg.transform.rotation
        [roll, pitch, yaw] = Transformation.get_euler_from_quaternion([q.w, q.x, q.y, q.z])

        # Create a Vicon quaternion from the Euler angles. Removing roll and pitch
        vicon_quaternion = Transformation.get_quaternion_from_euler([0, 0, yaw])

        # Extract Vicon position from message.
        vicon_position = [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z]

        # Create a Vicon transformation object from position and quaternion.
        self.vicon_transform = Transformation(vicon_position, vicon_quaternion)

        rospy.loginfo(self.vicon_transform.rotation_matrix)

        # Update drone position based on Vicon data.
        self.node.drone_pose.pose.position.x = msg.transform.translation.x
        self.node.drone_pose.pose.position.y = msg.transform.translation.y
        self.node.drone_pose.pose.position.z = msg.transform.translation.z

        # Publish Vicon pose data if enabled.
        if self.vicon_pose_enabled:
            self.node.drone_pose.header = msg.header
            self.node.drone_pose.pose.position.x = msg.transform.translation.x
            self.node.drone_pose.pose.position.y = msg.transform.translation.y
            self.node.drone_pose.pose.position.z = msg.transform.translation.z
            self.node.drone_pose.pose.orientation = msg.transform.rotation
            self.vicon_pose_pub.publish(self.node.drone_pose)
