import sys

import rosparam
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import MarkerArray

from drone_control.utils import *
from drone_mapping.grid_mapping import GridMapping
from drone_mapping.utils import p2l

np.set_printoptions(threshold=sys.maxsize)


class LocalPlanner:
    def __init__(self, node):
        """
        Constructor for the LocalPlanner class.
        :param node: Main ROS node
        """
        self.node = node
        self.yawing = 0
        self.sub_points_once = False
        self.sub_points_index = 0
        self.sub_points = PoseArray()

        self.sub_points_N = 5
        for i in range(self.sub_points_N):
            temp_pose = Pose()
            self.sub_points.poses.append(temp_pose)

        self.rotate_angle = rosparam.get_param("/rob498_drone_07/rotate_angle")

        self.map_center_x = rospy.get_param('/drone_mapping/map_center_x', -5)
        self.map_center_y = rospy.get_param('/drone_mapping/map_center_y', -5)
        self.map_size_x = rospy.get_param('/drone_mapping/map_size_x', 10.0)
        self.map_size_y = rospy.get_param('/drone_mapping/map_size_y', 10.0)
        self.map_resolution = rospy.get_param('/drone_mapping/map_resolution', 0.1)

        self.map = GridMapping(self.map_center_x, self.map_center_y, self.map_size_x, self.map_size_y,
                               self.map_resolution)

        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback,
                                        queue_size=1)

        self.add_obs_pub = rospy.Publisher("add_obs", Bool, queue_size=1)

    def map_callback(self, msg):
        # print("Orig: ", msg.data)
        gridmap_p = np.array(msg.data).reshape((self.map.map_rows, self.map.map_cols))  # / 100
        # idx = np.where(gridmap_p == 100)
        self.map.gridmap = p2l(gridmap_p)
        # print("LP: ", idx)

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
            yaw = (theta_d + self.rotate_angle)  # % (2 * np.pi)1.5708
            head_error = yaw - theta
            head_err_norm = math.atan2(math.sin(head_error), math.cos(head_error))

            rospy.loginfo_throttle(1, "yaw == 1")
        elif self.yawing == 2:  # Left
            theta = calc_yaw(curr_pose.pose.orientation)
            yaw = (theta_d - self.rotate_angle)  # % (2 * np.pi) #0.523599
            head_error = yaw - theta
            head_err_norm = math.atan2(math.sin(head_error), math.cos(head_error))

            rospy.loginfo_throttle(1, "yaw == 2")
            # msg = "Yaw:  " + str(yaw) + " theta: " + str(theta) + " theta_d: " + str(theta_d)
            # rospy.loginfo_throttle(1, msg)

        return yaw, head_err_norm

    def update_waypoint(self, error_pos_goal: float, error_pos_sub: float, pose_goal: PoseStamped,
                        curr_pose: PoseStamped) -> PoseStamped:
        """

        :param error_pos_sub:
        :param error_pos_goal:
        :param pose_goal:
        :param curr_pose:
        :return:
        """
        self.vis_waypoints()

        # Check if the error between the drone's current pose and the current waypoint is within the error
        # tolerance and there are more waypoints in the sequence
        if error_pos_sub < self.node.error_tol and self.sub_points_index < (
                self.sub_points_N - 1):
            msg = "Sub Waypoint " + str(self.sub_points_index) + " Cleared"
            rospy.loginfo_throttle(1, msg)

            self.sub_points_index += 1

            pose_goal.pose = self.sub_points.poses[self.sub_points_index]
            pose_goal.pose.orientation = curr_pose.pose.orientation

        elif error_pos_goal < self.node.error_tol and self.node.waypoint_index < (
                len(self.node.waypoints.poses) - 1):  # TODO tune

            # If the drone has reached the current waypoint and there are more waypoints in the sequence,
            # set the next waypoint as the current one and update the drone's pose
            msg = "Waypoint " + str(self.node.waypoint_index) + " Cleared"
            rospy.loginfo_throttle(1, msg)

            self.node.waypoint_index += 1

            error_pos, theta_d, heading_error_norm = waypoint_pose_error(
                self.node.waypoints.poses[self.node.waypoint_index], self.node.drone_pose)
            q = quaternion_from_euler(0, 0, theta_d)

            pose_goal.pose = self.sub_points.poses[self.sub_points_index]
            pose_goal.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])  # yaw towards new target

            self.yawing = 0

            self.sub_points_index = 0
            self.sub_points_once = False

            if self.node.vis:
                # delete
                markerArray_traj = MarkerArray()
                for j, pts in enumerate(self.sub_points.poses):
                    marker_traj = vis_marker(pts, 1, 1, 0, 2, 0.08)
                    markerArray_traj.markers.append(marker_traj)
                    markerArray_traj.markers[j].id = j

                self.node.vis_traj_waypoints_pub.publish(markerArray_traj)

            self.calc_sub_points()

            if self.node.vis:
                # insert
                markerArray_traj = MarkerArray()
                for j, pts in enumerate(self.sub_points.poses):
                    marker_traj = vis_marker(pts, 1, 1, 0, 0, 0.08)
                    markerArray_traj.markers.append(marker_traj)
                    markerArray_traj.markers[j].id = j

                self.node.vis_traj_waypoints_pub.publish(markerArray_traj)
        else:
            # Set the current waypoint as the goal waypoint
            pose_goal.pose = self.sub_points.poses[self.sub_points_index]
            pose_goal.pose.orientation = curr_pose.pose.orientation

        return pose_goal

    def vis_waypoints(self):
        # Visualization: Create markers for all the waypoints, set the ones before the current waypoint as
        # red and the current and remaining waypoints as green

        markerArray = MarkerArray()

        if self.node.vis:
            for i, pt in enumerate(self.node.waypoints.poses):
                if i < self.node.waypoint_index:
                    marker = vis_marker(pt, 1, 0, 0, 2)
                    markerArray.markers.append(marker)
                    markerArray.markers[i].id = i
                elif i == self.node.waypoint_index:
                    marker = vis_marker(pt, 0, 1, 0, 0)
                    markerArray.markers.append(marker)
                    markerArray.markers[i].id = i
                else:
                    marker = vis_marker(pt, 1, 0, 0, 0)
                    markerArray.markers.append(marker)
                    markerArray.markers[i].id = i

            # Publish the marker array for visualization
            self.node.vis_waypoints_pub.publish(markerArray)

    def calc_sub_points(self) -> None:
        if not self.sub_points_once:

            self.sub_points.poses[0] = self.node.drone_pose.pose
            self.sub_points.poses[self.sub_points_N - 1] = self.node.waypoints.poses[self.node.waypoint_index]

            x_diff = (self.sub_points.poses[self.sub_points_N - 1].position.x - self.sub_points.poses[
                0].position.x) / (self.sub_points_N - 1.0)
            y_diff = (self.sub_points.poses[self.sub_points_N - 1].position.y - self.sub_points.poses[
                0].position.y) / (self.sub_points_N - 1.0)

            for i in range(1, (self.sub_points_N - 1)):
                temp_pose = Pose()
                temp_pose.position.x = self.sub_points.poses[0].position.x + x_diff * i
                temp_pose.position.y = self.sub_points.poses[0].position.y + y_diff * i
                temp_pose.position.z = self.sub_points.poses[0].position.z
                self.sub_points.poses[i] = temp_pose

            self.sub_points_once = True

    def waypoint_nav(self, curr_pose: PoseStamped) -> Tuple[float, float, PoseStamped]:
        """

        :param curr_pose:
        :return:
        """
        # Create a PoseStamped object with header frame id set to 'map'
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "map"

        # If new waypoints are received, update the current waypoint and check if the drone has reached the
        # current waypoint
        if self.node.services.bool_test and self.node.WAYPOINTS_RECEIVED:
            # Calc sub-points
            if self.sub_points_index == 0 and self.node.waypoint_index == 0:  # Init
                self.calc_sub_points()

                if self.node.vis:
                    markerArray_traj = MarkerArray()
                    for j, pts in enumerate(self.sub_points.poses):
                        marker_traj = vis_marker(pts, 1, 1, 0, 0, 0.08)
                        markerArray_traj.markers.append(marker_traj)
                        markerArray_traj.markers[j].id = j

                    self.node.vis_traj_waypoints_pub.publish(markerArray_traj)

            # Calculate the error in current sub waypoint
            error_pos_sub, _, _, = waypoint_pose_error(
                self.sub_points.poses[self.sub_points_index], self.node.drone_pose)

            # Calculate the error in current waypoint
            error_pos, theta_d, heading_error_norm = waypoint_pose_error(
                self.node.waypoints.poses[self.node.waypoint_index], self.node.drone_pose)

            yaw, heading_error_norm = self.rotating(theta_d, heading_error_norm, curr_pose)
            # msg = "YAW: " + str(yaw)
            # rospy.loginfo_throttle(1, msg)
            q = quaternion_from_euler(0, 0, yaw)

            # While holding position rotate to direction of new point
            if abs(heading_error_norm) > 0.1 and error_pos > 0.3:
                pose_goal.pose.position = curr_pose.pose.position
                pose_goal.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                rospy.loginfo_throttle(1, "heading_error_norm > 0.1")

            elif self.yawing == 3:

                pose_goal = self.update_waypoint(error_pos, error_pos_sub, pose_goal, curr_pose)

            elif self.yawing < 3:
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

        # # Visualization
        # if self.node.vis:
        #     # If visualization is enabled
        #     # Publish the goal waypoint as a green marker
        #
        #     if self.node.WAYPOINTS_RECEIVED:
        #         marker2 = vis_marker(self.node.waypoints.poses[self.node.waypoint_index], 0, 1, 0, 0)
        #         self.node.vis_goal_pub.publish(marker2)

        return error_pos, heading_error_norm, pose_goal
