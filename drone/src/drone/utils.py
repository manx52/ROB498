# !/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
import numpy as np
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from typing import Tuple

"""
Utility Function
"""


# def calc_quaternion(A, B):
#     """
#
#     :param A:
#     :param B:
#     :return:
#     """
#     # a = np.cross(A, B)
#     # x = a[0]
#     # y = a[1]
#     # z = a[2]
#     # A_length = np.linalg.norm(A)
#     # B_length = np.linalg.norm(B)
#     # w = math.sqrt((A_length ** 2) * (B_length ** 2)) + np.dot(A, B)
#     #
#     # norm = math.sqrt(x ** 2 + y ** 2 + z ** 2 + w ** 2)
#     # if norm == 0:
#     #     norm = 1
#     #
#     # x /= norm
#     # y /= norm
#     # z /= norm
#     # w /= norm
#     temp = np.dot(A, B) / (np.linalg.norm(A) * np.linalg.norm(B))
#     theta = math.acos(temp)
#     q = quaternion_from_euler(0, 0, theta)
#
#     return Quaternion(q[0], q[1], q[2], q[3])

def calc_quaternion(A: np.ndarray, B: np.ndarray) -> Quaternion:
    """
    Calculate the quaternion that represents the rotation required to align vector A with vector B.

    Args:
        A (numpy.ndarray): 3D vector to be rotated.
        B (numpy.ndarray): 3D vector representing the target direction for A after rotation.

    Returns:
        Quaternion: the quaternion that represents the rotation from A to B.

    Notes:
        The function uses the axis-angle representation of a rotation, which is a 3D vector that indicates the
        axis of rotation (perpendicular to the plane formed by A and B) and the angle of rotation (the angle
        between A and B). The axis-angle representation is then converted to a quaternion using the
        `quaternion_from_euler()` function from the `tf.transformations` module.

    """
    # Calculate the cosine of the angle between A and B
    cos_theta = np.dot(A, B) / (np.linalg.norm(A) * np.linalg.norm(B))

    # Calculate the angle of rotation
    theta = math.acos(cos_theta)

    # Convert the axis-angle representation to a quaternion
    q = quaternion_from_euler(0, 0, theta)

    # Return the quaternion as a Quaternion object
    return Quaternion(q[0], q[1], q[2], q[3])


def position_diff(goal_pose: PoseStamped, curr_pose: PoseStamped) -> Tuple[float, float, float]:
    """
    Calculates the difference in position between the goal pose and the current pose.

    :param goal_pose: The desired goal pose (geometry_msgs/Pose).
    :param curr_pose: The current pose (geometry_msgs/Pose).
    :return: A tuple containing the differences in x, y, and z position (float).
    """

    # Calculate the differences in position along each axis
    x_diff = goal_pose.pose.position.x - curr_pose.pose.position.x
    y_diff = goal_pose.pose.position.y - curr_pose.pose.position.y
    z_diff = goal_pose.pose.position.z - curr_pose.pose.position.z

    # Return the differences as a tuple
    return x_diff, y_diff, z_diff


def waypoint_error(goal_pose: PoseStamped, curr_pose: PoseStamped) -> float:
    """
    Calculates the Euclidean distance between the goal pose and the current pose.

    :param goal_pose: A tuple of (x, y, z) representing the desired position.
    :param curr_pose: A tuple of (x, y, z) representing the current position.
    :return: A float representing the Euclidean distance between the goal and current positions.
    """

    # Calculate the difference in position for each axis.
    x_diff, y_diff, z_diff = position_diff(goal_pose, curr_pose)

    # Calculate the Euclidean distance using the Pythagorean theorem.
    error_pose = math.sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)

    # Return the calculated error.
    return error_pose


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
