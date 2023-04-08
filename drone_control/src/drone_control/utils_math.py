# !/usr/bin/env python3
import math
from typing import Tuple

import numpy as np
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from tf.transformations import euler_from_quaternion

"""
Utility Math Functions
"""


def calc_yaw(q: Quaternion) -> float:
    """
    Calculate yaw angle (rotation around the z-axis) from quaternion.

    :param q: Quaternion object.
    :return: Yaw angle in radians.
    """

    # convert quaternion to euler angles and return the yaw component
    euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]


def waypoint_pose_error(goal_pose: Pose, curr_pose: PoseStamped, debug: bool = False) -> Tuple[
    float, float, float]:
    """
    Calculates the position and orientation error between a current pose and a desired pose.

    :param goal_pose: A pose of (x, y, z) representing the desired position.
    :param curr_pose: A pose of (x, y, z) representing the current position.
    :param debug: Whether to print debugging information.
    :return: A tuple of (position_error, desired_yaw, heading_error) representing the errors.
    """

    # Position Error

    # Calculate the difference in position for each axis.
    x_diff = goal_pose.position.x - curr_pose.pose.position.x
    y_diff = goal_pose.position.y - curr_pose.pose.position.y
    z_diff = goal_pose.position.z - curr_pose.pose.position.z

    # Calculate the Euclidean distance using the Pythagorean theorem.
    error_pos = math.sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)

    # Convert the current and desired positions to numpy arrays for easier math.
    A = np.array((curr_pose.pose.position.x, curr_pose.pose.position.y))
    B = np.array((goal_pose.position.x, goal_pose.position.y))

    # Calculate the desired yaw (angle of rotation) using the arctan2 function.
    theta_d = np.arctan2((B[1] - A[1]), (B[0] - A[0]))

    # Calculate the current yaw using a helper function 'calc_yaw'.
    theta = calc_yaw(curr_pose.pose.orientation)

    # Calculate the heading error between the current and desired yaws.
    head_error = theta_d - theta

    # Normalize the heading error to be between -pi and pi.
    heading_error_norm = math.atan2(math.sin(head_error), math.cos(head_error))

    # Print debugging information if debug flag is set.
    if debug:
        print(theta_d)
        print(theta)
        print(heading_error_norm)
        print(error_pos)

    # Return the calculated errors.
    return error_pos, theta_d, heading_error_norm
