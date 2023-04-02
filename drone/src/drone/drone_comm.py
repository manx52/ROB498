#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseArray
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBoolRequest, SetModeRequest
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from drone.utils import *
from drone_perception import Transformation
from drone.vicon import Vicon
from drone.services import Services
from drone.local_planner import LocalPlanner


class DroneComm:
    """
    ROS node that controls a drone.
    """

    def __init__(self):
        """
        Constructor for DroneComm class.
        """

        # Initialize node
        node_name = rospy.get_param("/node_name")
        rospy.init_node(node_name)

        # Get settings from ROS parameter server
        self.offset = rospy.get_param("/offset")
        self.land_offset = rospy.get_param("/land_offset")
        self.launch_height = rospy.get_param("/launch_height")
        self.vis = rospy.get_param("/vis")
        self.sim = rospy.get_param("/simulation")
        self.r = rospy.Rate(rospy.get_param("/rate"))
        self.error_tol = rospy.get_param("/error_tol")
        self.challenge4 = rospy.get_param("/challenge4")

        # Create submodules
        self.vicon = Vicon(self)
        self.services = Services(self, node_name)
        self.local_planner = LocalPlanner(self)

        # Initialize variables
        self.launch = False
        self.test = False
        self.drone_pose = PoseStamped()
        self.path = Path()
        self.WAYPOINTS_RECEIVED = False
        self.waypoint_goal = PoseStamped()
        self.waypoint_goal.header.stamp = rospy.Time.now()
        self.waypoint_goal.header.frame_id = "map"

        self.waypoint_goal.pose.position.x = 0
        self.waypoint_goal.pose.position.y = 0
        self.waypoint_goal.pose.position.z = self.land_offset

        self.waypoints = PoseArray()
        self.waypoints.header.frame_id = "map"
        self.waypoint_index = 0

        self.current_state = State()

        # Subscribers
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        self.sub_waypoints = rospy.Subscriber(node_name + '/comm/waypoints', PoseArray, self.callback_waypoints)

        # Publishers
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.setpoint_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

        if self.vis:
            self.vis_goal_pub = rospy.Publisher(node_name + '/comm/vis_goal', Marker, queue_size=1)
            self.vis_waypoints_pub = rospy.Publisher(node_name + '/comm/vis_waypoints', MarkerArray, queue_size=1)
            self.vis_path_pub = rospy.Publisher(node_name + '/comm/vis_path', Path, queue_size=1)

    # Callback Sensors
    def pose_callback(self, msg):
        """
        Callback function for position subscriber.

        :param msg: ROS message containing the drone pose
        :return: None
        """

        self.drone_pose = msg

        # Publish visualization data if required
        if self.vis:
            self.path.header.stamp = rospy.Time.now()
            self.path.header.frame_id = "map"
            self.path.poses.append(self.drone_pose)
            self.vis_path_pub.publish(self.path)

    def state_callback(self, msg):
        """
        Callback function for state subscriber.

        :param msg: ROS message containing the state
        :return: None
        """

        self.current_state = msg

    def callback_waypoints(self, msg):
        """
        Callback function to receive waypoints and transform them into the Vicon frame

        :param msg: ROS message containing the waypoints
        :return: None
        """

        # Wait for waypoints to be received
        # if self.WAYPOINTS_RECEIVED:
        #     return

        # Wait for Vicon Transform
        while not self.vicon.VICON_RECEIVED and not rospy.is_shutdown():
            rospy.loginfo_throttle(1, "Waiting for Vicon Transformation")
            self.r.sleep()

        rospy.loginfo("Waypoints Received")

        self.waypoint_index = 0
        self.WAYPOINTS_RECEIVED = True
        self.waypoints.header.stamp = rospy.Time.now()

        # Transform waypoints into Drone frame
        for pt in msg.poses:
            waypt = Transformation(position=[pt.position.x, pt.position.y, pt.position.z])

            transformed_position = self.vicon.vicon_transform.rotation_matrix @ waypt.position
            # print("Start")
            # print(self.vicon.vicon_transform.rotation_matrix)
            # print(waypt.position)
            # print(transformed_position)
            temp_pose = Pose()
            temp_pose.position.x = transformed_position[0]
            temp_pose.position.y = transformed_position[1]
            temp_pose.position.z = transformed_position[2]
            self.waypoints.poses.append(temp_pose)

        # Visualization
        if self.vis:
            markerArray = MarkerArray()
            for i, pt in enumerate(self.waypoints.poses):
                marker = vis_marker(pt, 1, 0, 0, 0)
                markerArray.markers.append(marker)
                markerArray.markers[i].id = i

            self.vis_waypoints_pub.publish(markerArray)

    # Main communication node for ground control
    def run(self):
        """
        Main loop to send velocity commands to the drone to follow the specified waypoints

        :return: None
        """

        # Wait for Flight Controller connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.r.sleep()

        # Set the initial waypoint as the goal waypoint
        pose = self.waypoint_goal
        self.local_planner.new_point(pose)

        # Send a few setpoints before starting to ensure smooth takeoff
        for i in range(100):
            if rospy.is_shutdown():
                break

            self.local_pos_pub.publish(pose)
            self.r.sleep()

        # Initialize drone for offboard mode and arm it (only applicable in simulation)
        if self.sim:
            # Create a SetModeRequest object and set the mode to 'OFFBOARD'
            offb_set_mode = SetModeRequest()
            offb_set_mode.custom_mode = 'OFFBOARD'

            # Create a CommandBoolRequest object and set the value to True to arm the drone
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = True

            last_req = rospy.Time.now()

        # Main loop to follow waypoints
        while not rospy.is_shutdown():
            # If in simulation mode, set the mode to 'OFFBOARD' and arm the drone
            if self.sim:
                # Set the mode to 'OFFBOARD' if it is not already set and more than 5 seconds have passed since last
                # request
                if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.services.set_mode_client.call(offb_set_mode).mode_sent:
                        rospy.loginfo("OFFBOARD enabled")

                    last_req = rospy.Time.now()

                # Arm the drone if it is not already armed and more than 5 seconds have passed since last request
                else:
                    if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                        if self.services.arm_client.call(arm_cmd).success:
                            rospy.loginfo("Vehicle armed")

                        last_req = rospy.Time.now()

            # Create a PoseStamped object with header frame id set to 'map'
            pose = PoseStamped()
            pose.header.frame_id = "map"
            twist_msg = Twist()

            # Create a MarkerArray object for visualization
            markerArray = MarkerArray()

            # If new waypoints are received, update the current waypoint and check if the drone has reached the
            # current waypoint
            if self.services.bool_test and self.WAYPOINTS_RECEIVED:
                # Set the current waypoint as the goal waypoint
                pose.pose = self.waypoints.poses[self.waypoint_index]
                if not self.test:
                    self.test = True
                    self.local_planner.new_point(pose)
                # Calculate the error in current waypoint
                error_pose = waypoint_error(pose, self.drone_pose)

                # Visualization: Create markers for all the waypoints, set the ones before the current waypoint as
                # red and the current and remaining waypoints as green
                if self.vis:
                    for i, pt in enumerate(self.waypoints.poses):
                        if i <= self.waypoint_index:
                            marker = vis_marker(pt, 1, 0, 0, 2)
                            markerArray.markers.append(marker)
                            markerArray.markers[i].id = i
                        else:
                            marker = vis_marker(pt, 1, 0, 0, 0)
                            markerArray.markers.append(marker)
                            markerArray.markers[i].id = i

                    # Publish the marker array for visualization
                    self.vis_waypoints_pub.publish(markerArray)

                # Check if the error between the drone's current pose and the current waypoint is within the error
                # tolerance and there are more waypoints in the sequence
                if error_pose < self.error_tol and self.waypoint_index < (len(self.waypoints.poses) - 1):  # TODO tune
                    # If the drone has reached the current waypoint and there are more waypoints in the sequence,
                    # set the next waypoint as the current one and update the drone's pose
                    self.waypoint_index += 1
                    pose.pose = self.waypoints.poses[self.waypoint_index]

                    A = np.array((self.drone_pose.pose.position.x, self.drone_pose.pose.position.y))
                    B = np.array((pose.pose.position.x, pose.pose.position.y))
                    q = calc_quaternion(A, B, self.drone_pose.pose.orientation)
                    #q = quaternion_from_euler(0, 0, self.waypoint_index * 0.872665)
                    pose.pose.orientation = q
                    self.local_planner.new_point(pose)

                twist_msg = self.local_planner.velocity_command(self.drone_pose)

            else:
                # If boolean test is False or waypoints have not been received
                # Set the goal waypoint as the current waypoint
                pose = self.waypoint_goal

                if self.services.bool_launch and not self.launch:
                    self.launch = True
                    self.local_planner.new_point(pose)

                twist_msg = self.local_planner.velocity_command(self.drone_pose)
                # Calculate the error between the goal waypoint and drone's current pose
                error_pose = waypoint_error(pose, self.drone_pose)

            # Set the timestamp of the pose header to the current time
            pose.header.stamp = rospy.Time.now()
            euler = euler_from_quaternion(
                [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            # Log the waypoint index and error to the console
            msg = "Waypoint {index:} Pos.x {x:} Pos.y {y:} Pos.z {z:} YAW: {yaw:} error: {error:}"
            rospy.loginfo_throttle(1, msg.format(index=self.waypoint_index, error=error_pose,
                                                 x=pose.pose.position.x,
                                                 y=pose.pose.position.y,
                                                 z=pose.pose.position.z,
                                                 yaw=euler[2],
                                                 #twist=twist_msg,
                                                 ))

            # Publish the goal waypoint to the drone's local position topic
            # self.local_pos_pub.publish(pose)
            self.setpoint_vel_pub.publish(twist_msg)
            # Visualization
            if self.vis:
                # If visualization is enabled
                # Publish the goal waypoint as a green marker
                marker = vis_marker(pose.pose, 0, 1, 0, 0)
                self.vis_goal_pub.publish(marker)

            # Sleep for a short period of time to ensure that the drone's position is updated before the next loop
            # iteration
            self.r.sleep()
