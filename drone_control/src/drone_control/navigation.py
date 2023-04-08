import sys

import ros_numpy
import rosparam
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler

from drone_control.utils_math import *
from drone_control.utils_vis import *
from drone_mapping.mapping import Mapping
from drone_mapping.utils import p2l

np.set_printoptions(threshold=sys.maxsize)


class Navigation:
    """
    Class that handles the navigation of the drone and obstacle avoidance
    """

    def __init__(self, node):
        """
        Initializes the Navigation class.

        :param node: The ROS node that the Navigation class belongs to.
        """
        self.node = node

        # Main matrix for all possible trajectories: 9x5x3
        self.traj_matrix = [PoseArray()] * 9

        # Init sub points with 5 intermediate points
        self.sub_points = PoseArray()
        self.sub_points_N = 5
        self.sub_points_once = False
        for i in range(self.sub_points_N):
            temp_pose = Pose()
            self.sub_points.poses.append(temp_pose)

        # Settings for trajectory
        self.length_pause = rosparam.get_param("/rob498_drone_07/length_pause")
        self.rotate_angle = rosparam.get_param("/rob498_drone_07/rotate_angle")
        self.yawing = 0
        self.sub_points_index = 0
        self.collision_traj_idx = []

        # Map settings
        self.map_center_x = rospy.get_param('/drone_mapping/map_center_x', -5)
        self.map_center_y = rospy.get_param('/drone_mapping/map_center_y', -5)
        self.map_size_x = rospy.get_param('/drone_mapping/map_size_x', 10.0)
        self.map_size_y = rospy.get_param('/drone_mapping/map_size_y', 10.0)
        self.map_resolution = rospy.get_param('/drone_mapping/map_resolution', 0.1)
        self.obs_col_rad = rospy.get_param('/drone_mapping/obs_col_rad', 0.4)
        self.drone_col_rad = rospy.get_param('/drone_mapping/drone_col_rad', 0.4)

        # Create map object
        self.map = Mapping(self.map_center_x, self.map_center_y, self.map_size_x, self.map_size_y,
                           self.map_resolution, self.drone_col_rad, self.obs_col_rad)

        # Subscribers
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback,
                                        queue_size=1)

        # Publishers
        self.add_obs_pub = rospy.Publisher("add_obs", Bool, queue_size=1)

    def map_callback(self, msg: OccupancyGrid):
        """
        This function is called when a new OccupancyGrid message is received.
        It converts the grid data in the message to a numpy array and then assigns
        it to the 'gridmap' attribute of the object's 'map' instance variable.

        :param msg: The OccupancyGrid message
        :return: None
        """
        # Convert the 1D occupancy grid data in the message to a 2D numpy array
        gridmap_p = np.array(msg.data).reshape((self.map.map_rows, self.map.map_cols)) / 100.0

        # Convert the probability values in the numpy array to log-odds values and store them in the 'gridmap'
        # attribute of the 'map' object
        self.map.gridmap = p2l(gridmap_p)

    def rotating(self, theta_d: float, heading_error_norm: float, curr_pose: PoseStamped, debug: bool = False) -> Tuple[
        float, float]:
        """
        This function rotates the robot by a certain angle to the left or right, depending on the self.yawing variable.

        :param debug: a boolean variable that enables debug information logging.
        :param heading_error_norm: the heading error of the robot.
        :param theta_d: the desired angle to which the robot must rotate.
        :param curr_pose: the current pose of the robot.
        :return: a tuple containing the new angle of the robot and the normalized heading error.
        """

        # Default Straight
        head_err_norm = heading_error_norm
        yaw = theta_d

        if self.yawing == 1:  # Right
            theta = calc_yaw(curr_pose.pose.orientation)
            yaw = (theta_d + self.rotate_angle)
            head_error = yaw - theta
            head_err_norm = math.atan2(math.sin(head_error), math.cos(head_error))

            if debug:
                rospy.loginfo_throttle(1, "yaw == 1")
                msg = "Yaw:  " + str(yaw) + " theta: " + str(theta) + " theta_d: " + str(theta_d)
                rospy.loginfo_throttle(1, msg)

        elif self.yawing == 2:  # Left
            theta = calc_yaw(curr_pose.pose.orientation)
            yaw = (theta_d - self.rotate_angle)
            head_error = yaw - theta
            head_err_norm = math.atan2(math.sin(head_error), math.cos(head_error))

            if debug:
                rospy.loginfo_throttle(1, "yaw == 2")
                msg = "Yaw:  " + str(yaw) + " theta: " + str(theta) + " theta_d: " + str(theta_d)
                rospy.loginfo_throttle(1, msg)

        return yaw, head_err_norm

    def calc_trajectory(self, traj_type: int, angle: float = 1.0472) -> PoseArray:
        """
        This function calculates a trajectory based on the specified trajectory type and angle
        and returns a PoseArray containing the calculated sub-points

        :param traj_type: left, right, straight
        :param angle: angle to vary the second point by
        :return: PoseArray with the 5 sub points for the trajectory
        """

        # Create a new PoseArray to store the sub-points
        temp_sub_points = PoseArray()

        # Add empty Pose objects to the PoseArray
        for i in range(self.sub_points_N):
            temp_pose = Pose()
            temp_sub_points.poses.append(temp_pose)

        # Set the first and last sub-point to the drone's current position and the next waypoint
        temp_sub_points.poses[0] = self.node.drone_pose.pose
        temp_sub_points.poses[self.sub_points_N - 1] = self.node.waypoints.poses[self.node.waypoint_index]

        # Calculate the x and y differences between the first and last sub-points
        x_diff = (temp_sub_points.poses[self.sub_points_N - 1].position.x - temp_sub_points.poses[
            0].position.x) / (self.sub_points_N - 1.0)
        y_diff = (temp_sub_points.poses[self.sub_points_N - 1].position.y - temp_sub_points.poses[
            0].position.y) / (self.sub_points_N - 1.0)

        # Calculate the remaining sub-points based on the specified trajectory type and angle
        for i in range(1, (self.sub_points_N - 1)):
            temp_pose = Pose()

            if traj_type == 0:  # linear trajectory
                temp_pose.position.x = temp_sub_points.poses[0].position.x + x_diff * i
                temp_pose.position.y = temp_sub_points.poses[0].position.y + y_diff * i
            elif traj_type == 1:  # curve to the right
                x_diff_t = y_diff / math.tan(angle)
                y_diff_t = x_diff / math.tan(angle)
                temp_pose.position.x = temp_sub_points.poses[0].position.x + x_diff * i + x_diff_t
                temp_pose.position.y = temp_sub_points.poses[0].position.y + y_diff * i - y_diff_t
            elif traj_type == 2:  # curve to the left
                x_diff_t = y_diff / math.tan(angle)
                y_diff_t = x_diff / math.tan(angle)
                temp_pose.position.x = temp_sub_points.poses[0].position.x + x_diff * i - x_diff_t
                temp_pose.position.y = temp_sub_points.poses[0].position.y + y_diff * i + y_diff_t

            # Set the z position of the sub-point to be the same as the drone's current z position
            temp_pose.position.z = temp_sub_points.poses[0].position.z
            temp_sub_points.poses[i] = temp_pose

        # Return the PoseArray containing the calculated sub-points
        return temp_sub_points

    def trajectory_rollout(self, debug: bool = False):
        """
        This function calculates the drone trajectories for each trajectory type and selects the best trajectory
        based on collision detection and distance to the goal.

        :param debug: boolean indicating whether to print debug messages
        :return: None
        """
        if not self.sub_points_once:
            # Calculate trajectories
            self.traj_matrix[0] = self.calc_trajectory(0)
            self.traj_matrix[1] = self.calc_trajectory(1, 1.309)
            self.traj_matrix[2] = self.calc_trajectory(1, 1.0472)
            self.traj_matrix[3] = self.calc_trajectory(1, 0.785398)
            self.traj_matrix[4] = self.calc_trajectory(1, 0.523599)
            self.traj_matrix[5] = self.calc_trajectory(2, 1.309)
            self.traj_matrix[6] = self.calc_trajectory(2, 1.0472)
            self.traj_matrix[7] = self.calc_trajectory(2, 0.785398)
            self.traj_matrix[8] = self.calc_trajectory(2, 0.523599)

            valid_opts = range(len(self.traj_matrix))
            temp_pose_matrix = []

            # Convert traj_matrix to numpy  9x5x3
            for traj in self.traj_matrix:  # loop through all trajectories
                temp_pose_list = []
                for pts in traj.poses:  # loop through all points in trajectory
                    temp_pose_list.append(ros_numpy.geometry.vector3_to_numpy(pts.position)[:2])

                temp_pose_matrix.append(np.array(temp_pose_list))

            # Convert current pose into numpy
            curr_pos = ros_numpy.geometry.vector3_to_numpy(self.node.drone_pose.pose.position)

            # Convert all trajectories to pixel coordinates
            temp_pose_matrix = np.array(temp_pose_matrix)
            for i, traj in enumerate(temp_pose_matrix):  # loop through all trajectories
                for j, pts in enumerate(traj):  # loop through all points in trajectory

                    # convert coordinates to pixel coordinates
                    y, x = self.map.coord_to_grid(pts[0], pts[1])
                    temp_pose_matrix[i, j, 0] = y
                    temp_pose_matrix[i, j, 1] = x

            # Convert current pose to pixel coordinates
            y_in_map_pix, x_in_map_pix = self.map.coord_to_grid(curr_pos[0], curr_pos[1])

            # send trajectories to mapping class for collision detection
            self.collision_traj_idx = self.map.check_collision(temp_pose_matrix, y_in_map_pix, x_in_map_pix, debug)

            if debug:
                msg = "collision_traj_idx: " + str(self.collision_traj_idx)
                rospy.loginfo_throttle(1, msg)

            # Remove collided trajectories
            valid_opts = np.delete(np.array(valid_opts), self.collision_traj_idx)

            # Select best trajectory
            best_opt = self.best_path(temp_pose_matrix, valid_opts, debug)
            self.sub_points.poses = self.traj_matrix[best_opt].poses

            # Only Calculate once per waypoint
            self.sub_points_once = True

    @staticmethod
    def best_path(traj_mat: np.ndarray, valid_opts: np.ndarray, debug: bool = False) -> int:
        """
        Find the best path among the given options.

        :param traj_mat: a numpy array of trajectories.
        :param valid_opts: a numpy array of valid options.
        :param debug: a boolean flag indicating whether to print debug information.
        :return: an integer index indicating the best path among the given options.
        """

        # Get the number of options in the given trajectories.
        num_opts = len(traj_mat)

        # Initialize the final cost array to infinity.
        final_cost = np.Inf * np.ones(num_opts)

        # Loop through each option.
        for i in range(num_opts):

            # Check if the current option is valid.
            if i in valid_opts:
                # Get the last point of the current trajectory.
                trajectory_o = traj_mat[i]
                end_pt = trajectory_o[-1, :]

                # Get the position of the robot at the second point of the current trajectory.
                pos = trajectory_o[1, :]

                # Calculate the cost of the current option and store it in the final cost array.
                final_cost[i] = np.linalg.norm(pos - end_pt)

        # Print the final cost array if debug flag is true.
        if debug:
            print("cost to go", final_cost)

        # If all costs are infinity, set the best option to 0.
        if final_cost.min == np.Inf:  # TODO
            best_opt = 0

        # Otherwise, set the best option to the index of the minimum cost in the final cost array.
        else:
            best_opt = final_cost.argmin()

        # Return the best option index.
        return best_opt

    def update_waypoint(self, error_pos_goal: float, error_pos_sub: float, pose_goal: PoseStamped,
                        curr_pose: PoseStamped, debug: bool = False) -> PoseStamped:
        """
        This function updates the current waypoint based on the drone's current
        pose and the desired trajectory.

        :param debug: A boolean flag indicating whether or not to print debugging messages
        :param error_pos_sub: The distance between the drone's current pose and the current sub-waypoint
        :param error_pos_goal: The distance between the drone's current pose and the current goal waypoint
        :param pose_goal: The desired pose for the drone to reach
        :param curr_pose: The current pose of the drone
        :return: The updated desired pose for the drone to reach
        """
        # Visualize the current set of waypoints
        vis_waypoints(self.node.vis, self.node.waypoints.poses, self.node.waypoint_index, self.node.vis_waypoints_pub)

        # Check if the error between the drone's current pose and the current sub-waypoint is within the error
        # tolerance and there are more sub-waypoints in the sequence
        if error_pos_sub < self.node.error_tol and self.sub_points_index < (
                self.sub_points_N - 1):
            if debug:
                msg = "Sub Waypoint " + str(self.sub_points_index) + " Cleared"
                rospy.loginfo_throttle(1, msg)

            # Move to the next sub-waypoint in the sequence
            self.sub_points_index += 1

            # Set the pose_goal to be the next sub-waypoint and maintain the current orientation of the drone
            pose_goal.pose = self.sub_points.poses[self.sub_points_index]
            pose_goal.pose.orientation = curr_pose.pose.orientation

        # Check if the error between the drone's current pose and the current goal waypoint is within the error
        # tolerance and there are more waypoints in the sequence
        elif error_pos_goal < self.node.error_tol and self.node.waypoint_index < (
                len(self.node.waypoints.poses) - 1):  # TODO tune

            if debug:
                msg = "Waypoint " + str(self.node.waypoint_index) + " Cleared"
                rospy.loginfo_throttle(1, msg)

            # Move to the next waypoint in the sequence
            self.node.waypoint_index += 1

            # Calculate the error between the drone's current pose and the new goal waypoint, and the desired heading
            error_pos, theta_d, heading_error_norm = waypoint_pose_error(
                self.node.waypoints.poses[self.node.waypoint_index], self.node.drone_pose)

            # Convert the desired heading to a quaternion
            q = quaternion_from_euler(0, 0, theta_d)

            # Set the pose_goal to be the new goal waypoint and point the drone's yaw towards the new target
            pose_goal.pose = self.sub_points.poses[self.sub_points_index]
            pose_goal.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])  # yaw towards new target

            # Reset the yawing flag and visualize the new set of sub-waypoints
            self.yawing = 0
            vis_sub_points(self.node.vis, 2, self.traj_matrix, self.collision_traj_idx, self.sub_points_N,
                           self.node.vis_traj_waypoints_pub)
            self.sub_points_index = 0
            self.sub_points_once = False

        else:
            # Set the current waypoint as the goal waypoint
            pose_goal.pose = self.sub_points.poses[self.sub_points_index]
            pose_goal.pose.orientation = curr_pose.pose.orientation

        # Return the updated pose_goal
        return pose_goal

    def waypoint_navigation(self, curr_pose: PoseStamped, debug: bool = False) -> Tuple[float, float, PoseStamped]:
        """
        This function is the main navigation algorithm

        :param debug: (bool) flag to print debug information
        :param curr_pose: (PoseStamped) the current pose of the drone
        :return: Tuple containing error in current position, heading error, and goal position (PoseStamped)
        """
        # Create a PoseStamped object with header frame id set to 'map'
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "map"

        # If new waypoints are received, update the current waypoint and check if the drone has reached the
        # current waypoint
        if self.node.services.bool_test and self.node.WAYPOINTS_RECEIVED:

            # Calculate the error in current sub waypoint
            error_pos_sub, _, _, = waypoint_pose_error(
                self.sub_points.poses[self.sub_points_index], self.node.drone_pose)

            # Calculate the error in current waypoint
            error_pos, theta_d, heading_error_norm = waypoint_pose_error(
                self.node.waypoints.poses[self.node.waypoint_index], self.node.drone_pose)

            # Calculate the yaw rotation and heading error normalization
            yaw, heading_error_norm = self.rotating(theta_d, heading_error_norm, curr_pose)

            # Convert the yaw rotation to a quaternion
            q = quaternion_from_euler(0, 0, yaw)

            # While holding position rotate to direction of new point
            if abs(heading_error_norm) > 0.1 and error_pos > 0.3:
                pose_goal.pose.position = curr_pose.pose.position
                pose_goal.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

                # Print debug information
                if debug:
                    rospy.loginfo_throttle(1, "heading_error_norm > 0.1")

            elif self.yawing == 3:

                # Calculate sub-points
                self.trajectory_rollout()

                # Visualize sub-points
                vis_sub_points(self.node.vis, 0, self.traj_matrix, self.collision_traj_idx, self.sub_points_N,
                               self.node.vis_traj_waypoints_pub)

                # Update the goal waypoint
                pose_goal = self.update_waypoint(error_pos, error_pos_sub, pose_goal, curr_pose)

            elif self.yawing < 3:

                # Pause for a specified length of time
                for i in range(self.length_pause):
                    if rospy.is_shutdown():
                        break

                    pose_goal.pose.position = curr_pose.pose.position
                    pose_goal.pose.orientation = curr_pose.pose.orientation

                    # Enable adding obstacles to the map
                    if i == (self.length_pause // 2):
                        msg = Bool()
                        msg.data = True
                        self.add_obs_pub.publish(msg)

                    # Publish the local position goal
                    self.node.local_pos_pub.publish(pose_goal)

                    self.node.r.sleep()

                # Disable adding obstacles to the map
                msg = Bool()
                msg.data = False
                self.add_obs_pub.publish(msg)

                # Change yawing trajectory
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
