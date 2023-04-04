from typing import Tuple, Any

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray

from drone.utils import *


class LocalPlanner:
    def __init__(self, node):
        """
        Constructor for the LocalPlanner class.
        :param node: Main ROS node
        """
        self.set_yaw = 0
        self.node = node

    def waypoint_nav(self, curr_pose: PoseStamped) -> Tuple[float, float, PoseStamped]:
        """

        :return:
        """
        # Create a PoseStamped object with header frame id set to 'map'
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "map"

        # Create a MarkerArray object for visualization
        markerArray = MarkerArray()

        # If new waypoints are received, update the current waypoint and check if the drone has reached the
        # current waypoint
        if self.node.services.bool_test and self.node.WAYPOINTS_RECEIVED:

            # Calculate the error in current waypoint
            error_pos, theta_d, heading_error_norm = waypoint_pose_error(
                self.node.waypoints.poses[self.node.waypoint_index], self.node.drone_pose)
            q = quaternion_from_euler(0, 0, theta_d)
            # error_pos_curr, theta_d_curr, heading_error_norm_curr = waypoint_pose_error(
            #     self.node.waypoints.poses[self.node.waypoint_index], self.node.drone_pose)

            # While holding position rotate to direction of new point
            if abs(heading_error_norm) > 0.1 and error_pos > 0.3:
                pose_goal.pose.position = curr_pose.pose.position
                pose_goal.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

                rospy.loginfo_throttle(1, "heading_error_norm > 0.1")
            else:

                # Set the current waypoint as the goal waypoint
                pose_goal.pose = self.node.waypoints.poses[self.node.waypoint_index]
                pose_goal.pose.orientation = curr_pose.pose.orientation
                # Visualization: Create markers for all the waypoints, set the ones before the current waypoint as
                # red and the current and remaining waypoints as green
                if self.node.vis:
                    for i, pt in enumerate(self.node.waypoints.poses):
                        if i <= self.node.waypoint_index:
                            marker = vis_marker(pt, 1, 0, 0, 2)
                            markerArray.markers.append(marker)
                            markerArray.markers[i].id = i
                        else:
                            marker = vis_marker(pt, 1, 0, 0, 0)
                            markerArray.markers.append(marker)
                            markerArray.markers[i].id = i

                    # Publish the marker array for visualization
                    self.node.vis_waypoints_pub.publish(markerArray)

                # Check if the error between the drone's current pose and the current waypoint is within the error
                # tolerance and there are more waypoints in the sequence
                if error_pos < self.node.error_tol and self.node.waypoint_index < (
                        len(self.node.waypoints.poses) - 1):  # TODO tune
                    # If the drone has reached the current waypoint and there are more waypoints in the sequence,
                    # set the next waypoint as the current one and update the drone's pose
                    msg = "Waypoint " + str(self.node.waypoint_index)+" Cleared"
                    rospy.loginfo_throttle(1, msg)

                    self.node.waypoint_index += 1

                    error_pos, theta_d, heading_error_norm = waypoint_pose_error(
                        self.node.waypoints.poses[self.node.waypoint_index], self.node.drone_pose)
                    q = quaternion_from_euler(0, 0, theta_d)

                    pose_goal.pose = self.node.waypoints.poses[self.node.waypoint_index]
                    pose_goal.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        else:
            # If boolean test is False or waypoints have not been received
            # Set the goal waypoint as the current waypoint
            pose_goal = self.node.waypoint_goal

            # Calculate the error between the goal waypoint and drone's current pose
            error_pos, theta_d, heading_error_norm = waypoint_pose_error(pose_goal.pose, self.node.drone_pose)

        # Set the timestamp of the pose header to the current time
        pose_goal.header.stamp = rospy.Time.now()

        return error_pos, heading_error_norm, pose_goal
