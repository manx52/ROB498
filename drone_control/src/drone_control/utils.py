# !/usr/bin/env python3
import math
from typing import Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

"""
Utility Function
"""


def calc_yaw(q: Quaternion) -> float:
    """

    :param q:
    :return:
    """
    euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]


def waypoint_pose_error(goal_pose: Pose, curr_pose: PoseStamped, debug: bool = False) -> Tuple[
    float, float, float]:
    """
    Calculates the Euclidean distance between the goal pose and the current pose.

    :param debug:
    :param goal_pose: A pose of (x, y, z) representing the desired position.
    :param curr_pose: A pose of (x, y, z) representing the current position.
    :return:
    """

    # Position Error

    # Calculate the difference in position for each axis.
    x_diff = goal_pose.position.x - curr_pose.pose.position.x
    y_diff = goal_pose.position.y - curr_pose.pose.position.y
    z_diff = goal_pose.position.z - curr_pose.pose.position.z

    # Calculate the Euclidean distance using the Pythagorean theorem.
    error_pos = math.sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)

    A = np.array((curr_pose.pose.position.x, curr_pose.pose.position.y))
    B = np.array((goal_pose.position.x, goal_pose.position.y))

    # Calculate the angle of rotation
    theta_d = np.arctan2((B[1] - A[1]), (B[0] - A[0]))
    theta = calc_yaw(curr_pose.pose.orientation)
    head_error = theta_d - theta
    heading_error_norm = math.atan2(math.sin(head_error), math.cos(head_error))

    # # Convert the euler-angle representation to a quaternion
    # q = quaternion_from_euler(0, 0, theta_d)
    if debug:
        print(theta_d)
        print(theta)
        print(heading_error_norm)
        print(error_pos)

    # Return the calculated error.
    return error_pos, theta_d, heading_error_norm


def vis_marker(pose: Pose, r: float, g: float, b: float, action: int) -> Marker:
    """
    The function creates a new Marker object and sets its properties based on the input arguments. The marker is then returned.

    :param pose: a Pose message containing the position and orientation of the marker
    :param r: a float representing the red component of the marker's color (0.0 to 1.0)
    :param g: a float representing the green component of the marker's color (0.0 to 1.0)
    :param b: a float representing the blue component of the marker's color (0.0 to 1.0)
    :param action: an integer representing the action to take with the marker (e.g. ADD, DELETE)
    :return: Marker object
    """

    # Create a new Marker object
    marker = Marker()

    # Set the frame ID and timestamp of the marker
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    # Set the type of marker to a sphere
    marker.type = marker.SPHERE

    # Set the action to take with the marker (e.g. ADD, DELETE)
    marker.action = action

    # Set the scale of the marker
    marker.scale.x = 0.125
    marker.scale.y = 0.125
    marker.scale.z = 0.125

    # Set the alpha (transparency) of the marker to fully opaque
    marker.color.a = 1.0

    # Set the color of the marker based on the input arguments
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b

    # Set the orientation of the marker to the default (no rotation)
    marker.pose.orientation.w = 1.0

    # Set the position of the marker based on the input pose
    marker.pose.position.x = pose.position.x
    marker.pose.position.y = pose.position.y
    marker.pose.position.z = pose.position.z

    # Return the marker object
    return marker
