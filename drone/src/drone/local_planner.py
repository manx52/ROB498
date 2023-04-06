from typing import Tuple, Any

import rosparam
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray

from drone.utils import *


class LocalPlanner:
    def __init__(self, node):
        """
        Constructor for the LocalPlanner class.
        :param node: Main ROS node
        """
        self.node = node
        self.yawing = 0
        self.rotate_angle = rosparam.get_param("/rotate_angle")

        self.add_obs_pub = rospy.Publisher("add_obs", Bool, queue_size=1)

    def rotating(self, theta_d: float, heading_error_norm: float, curr_pose: PoseStamped) -> Tuple[float, float]:
        """

        :param heading_error_norm:
        :param theta_d:
        :param curr_pose:
        :return:
        """

        # Default Straight
        head_err_norm = heading_error_norm
        yaw = theta_d

        if self.yawing == 1:  # Right
            theta = calc_yaw(curr_pose.pose.orientation)
            yaw = (theta_d + 0.523599)  # % (2 * np.pi)1.5708
            head_error = yaw - theta
            head_err_norm = math.atan2(math.sin(head_error), math.cos(head_error))

            rospy.loginfo_throttle(1, "yaw == 1")
        elif self.yawing == 2:  # Left
            theta = calc_yaw(curr_pose.pose.orientation)
            yaw = (theta_d - 0.523599)  # % (2 * np.pi) #0.523599
            head_error = yaw - theta
            head_err_norm = math.atan2(math.sin(head_error), math.cos(head_error))

            rospy.loginfo_throttle(1, "yaw == 2")
            # msg = "Yaw:  " + str(yaw) + " theta: " + str(theta) + " theta_d: " + str(theta_d)
            # rospy.loginfo_throttle(1, msg)

        return yaw, head_err_norm

    def update_waypoint(self, error_pos: float, pose_goal: PoseStamped, curr_pose: PoseStamped) -> PoseStamped:
        """

        :param error_pos:
        :param pose_goal:
        :param curr_pose:
        :return:
        """
        # Check if the error between the drone's current pose and the current waypoint is within the error
        # tolerance and there are more waypoints in the sequence
        if error_pos < self.node.error_tol and self.node.waypoint_index < (
                len(self.node.waypoints.poses) - 1):  # TODO tune
            # If the drone has reached the current waypoint and there are more waypoints in the sequence,
            # set the next waypoint as the current one and update the drone's pose
            msg = "Waypoint " + str(self.node.waypoint_index) + " Cleared"
            rospy.loginfo_throttle(1, msg)

            self.node.waypoint_index += 1

            error_pos, theta_d, heading_error_norm = waypoint_pose_error(
                self.node.waypoints.poses[self.node.waypoint_index], self.node.drone_pose)
            q = quaternion_from_euler(0, 0, theta_d)

            pose_goal.pose = curr_pose.pose  # self.node.waypoints.poses[self.node.waypoint_index]
            pose_goal.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            self.yawing = 0
        else:
            # Set the current waypoint as the goal waypoint
            pose_goal.pose = self.node.waypoints.poses[self.node.waypoint_index]
            pose_goal.pose.orientation = curr_pose.pose.orientation

        return pose_goal

    def waypoint_nav(self, curr_pose: PoseStamped) -> Tuple[float, float, PoseStamped]:
        """

        :param curr_pose:
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

            yaw, heading_error_norm = self.rotating(theta_d, heading_error_norm, curr_pose)
            q = quaternion_from_euler(0, 0, yaw)

            # While holding position rotate to direction of new point
            if abs(heading_error_norm) > 0.1 and error_pos > 0.3:
                pose_goal.pose.position = curr_pose.pose.position
                pose_goal.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                rospy.loginfo_throttle(1, "heading_error_norm > 0.1")

            elif self.yawing == 3:

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

                pose_goal = self.update_waypoint(error_pos, pose_goal, curr_pose)

            else:
                for i in range(30):
                    if rospy.is_shutdown():
                        break
                    pose_goal.pose.position = curr_pose.pose.position
                    pose_goal.pose.orientation = curr_pose.pose.orientation
                    if i == 15:
                        msg = Bool()
                        msg.data = True
                        self.add_obs_pub.publish(msg)

                    self.node.local_pos_pub.publish(pose_goal)
                    self.node.r.sleep()

                msg = Bool()
                msg.data = False
                self.add_obs_pub.publish(msg)
                self.yawing += 1
        else:
            # If boolean test is False or waypoints have not been received
            # Set the goal waypoint as the current waypoint
            pose_goal = self.node.waypoint_goal

            # Calculate the error between the goal waypoint and drone's current pose
            error_pos, theta_d, heading_error_norm = waypoint_pose_error(pose_goal.pose, self.node.drone_pose)

        # Set the timestamp of the pose header to the current time
        pose_goal.header.stamp = rospy.Time.now()

        return error_pos, heading_error_norm, pose_goal
