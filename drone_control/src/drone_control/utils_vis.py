# !/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray

"""
Utility Visualize Functions
"""


def vis_marker(pose: Pose, r: float, g: float, b: float, action: int, scale: float = 0.125,
               marker_type: int = 2, a: float = 1.0) -> Marker:
    """
    The function creates a new Marker object and sets its properties based on the input arguments. The marker is then
    returned.

    :param a: a float representing the transparency component of the marker's color (0.0 to 1.0)
    :param marker_type: shape of marker
    :param scale: how big the marker should be
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
    marker.type = marker_type

    # Set the action to take with the marker (e.g. ADD, DELETE)
    marker.action = action

    # Set the scale of the marker
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale

    # Set the alpha (transparency) of the marker to fully opaque
    marker.color.a = a

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


def vis_sub_points(vis: bool, action: int, traj_matrix: list, collision_traj_idx: list, sub_points_N: int,
                   vis_traj_waypoints_pub):
    """
    Visualizes the trajectory matrix by publishing a MarkerArray of markers.

    :param vis_traj_waypoints_pub: A ROS publisher for the trajectory markers.
    :param sub_points_N: The number of sub-points between each pair of waypoints in the trajectory.
    :param collision_traj_idx: A list of indices of trajectories that are in collision.
    :param traj_matrix: The trajectory matrix containing all the waypoints.
    :param vis: A boolean indicating whether the trajectory is visualizable or not.
    :param action: The action being performed on the trajectory.
    :return: None
    """

    # Check if node is visualizable
    if vis:

        # Create an empty MarkerArray to hold the trajectory markers
        markerArray_traj = MarkerArray()

        # Loop over all the trajectory poses in the trajectory matrix
        for i, traj_pose in enumerate(traj_matrix):
            # Loop over all the points in the trajectory pose
            for j, pts in enumerate(traj_pose.poses):

                # If the trajectory index is in the list of collision trajectory indices, mark it as red
                if i in collision_traj_idx:
                    marker_traj = vis_marker(pts, 1, 0, 0, action, 0.02, 4, 0.85)

                # Otherwise, mark it as green
                else:
                    marker_traj = vis_marker(pts, 1, 1, 0, action, 0.02, 4, 0.85)

                # init the trajectory points for the current marker
                marker_traj.points = []

                # Add a point at the current position of the marker
                p = Point()
                p.x = 0
                p.y = 0
                p.z = 0
                marker_traj.points.append(p)

                # If we're not at the last point, calculate the distance between the current point and the next one
                if j != sub_points_N - 1:
                    p = Point()
                    p.x = traj_pose.poses[j + 1].position.x - marker_traj.pose.position.x
                    p.y = traj_pose.poses[j + 1].position.y - marker_traj.pose.position.y
                    p.z = traj_pose.poses[j + 1].position.z - marker_traj.pose.position.z
                    marker_traj.points.append(p)

                # Add the current marker to the MarkerArray
                markerArray_traj.markers.append(marker_traj)

                # Set the ID of the current marker to the current index in the MarkerArray
                markerArray_traj.markers[j + (i * sub_points_N)].id = j + (i * sub_points_N)

        # Publish the MarkerArray of trajectory markers
        vis_traj_waypoints_pub.publish(markerArray_traj)


def vis_waypoints(vis: bool, waypoints_poses: list, waypoint_index: int, vis_waypoints_pub):
    """
    This function visualizes the waypoints on a ROS node. Create markers for all the waypoints, set the ones
    before the current waypoint as red and the current and remaining waypoints as green

    :param vis: a boolean flag to indicate if visualization is required
    :param waypoints_poses: a list of the poses of all the waypoints
    :param waypoint_index: the index of the current waypoint
    :param vis_waypoints_pub: the ROS publisher to publish the marker array
    :return: None
    """

    # Create a MarkerArray to hold the markers for each waypoint
    markerArray = MarkerArray()

    # If the visualization flag is set, create markers for all the waypoints
    if vis:
        for i, pt in enumerate(waypoints_poses):

            # If the waypoint is before the current waypoint, delete it
            if i < waypoint_index:
                marker = vis_marker(pt, 1, 0, 0, 2)
                markerArray.markers.append(marker)
                markerArray.markers[i].id = i

            # If the waypoint is the current waypoint, set it to green
            elif i == waypoint_index:
                marker = vis_marker(pt, 0, 1, 0, 0)
                markerArray.markers.append(marker)
                markerArray.markers[i].id = i

            # If the waypoint is after the current waypoint, set it to red
            else:
                marker = vis_marker(pt, 1, 0, 0, 0)
                markerArray.markers.append(marker)
                markerArray.markers[i].id = i

        # Publish the marker array for visualization
        vis_waypoints_pub.publish(markerArray)
