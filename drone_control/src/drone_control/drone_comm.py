#!/usr/bin/env python3
from geometry_msgs.msg import Twist, PoseArray
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBoolRequest, SetModeRequest
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
import tf
from drone_control.services import Services
from drone_control.utils import *
from drone_control.vicon import Vicon
from drone_common.transformation import Transformation
from drone_control.local_planner import LocalPlanner


class DroneComm:
    """
    ROS node that controls a drone.
    """

    def __init__(self):
        """
        Constructor for DroneComm class.
        """

        # Initialize node
        node_name = rospy.get_param("/rob498_drone_07/node_name")
        rospy.init_node(node_name)

        # Get settings from ROS parameter server
        self.offset = rospy.get_param("/rob498_drone_07/offset")
        self.land_offset = rospy.get_param("/rob498_drone_07/land_offset")
        self.launch_height = rospy.get_param("/rob498_drone_07/launch_height")
        self.vis = rospy.get_param("/rob498_drone_07/vis")
        self.sim = rospy.get_param("/simulation")
        self.r = rospy.Rate(rospy.get_param("/rob498_drone_07/rate"))
        self.error_tol = rospy.get_param("/rob498_drone_07/error_tol")

        # Create submodules
        self.vicon = Vicon(self)
        self.services = Services(self, node_name)
        self.local_planner = LocalPlanner(self)

        # Initialize variables
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
            self.vis_traj_goal_pub = rospy.Publisher(node_name + '/comm/vis_traj_goal', Marker, queue_size=1)
            self.vis_goal_pub = rospy.Publisher(node_name + '/comm/vis_goal', Marker, queue_size=1)
            self.vis_waypoints_pub = rospy.Publisher(node_name + '/comm/vis_waypoints', MarkerArray, queue_size=1)
            self.vis_path_pub = rospy.Publisher(node_name + '/comm/vis_path', Path, queue_size=1)
            self.vis_traj_waypoints_pub = rospy.Publisher(node_name + '/comm/vis_traj_waypoints', MarkerArray,
                                                          queue_size=1)

    # Callback Sensors
    def pose_callback(self, msg):
        """
        Callback function for position subscriber.

        :param msg: ROS message containing the drone pose
        :return: None
        """

        self.drone_pose = msg

        # Create a transform broadcaster
        br = tf.TransformBroadcaster()

        # Broadcast the transform
        br.sendTransform(
            (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
            (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
            rospy.Time.now(),
            "base_link",
            "map"
        )
        # Publish visualization data if required
        if self.vis and self.sim:
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
        if self.WAYPOINTS_RECEIVED:
            return

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
        Main loop to send position commands to the drone to follow the specified waypoints

        :return: None
        """

        # Wait for Flight Controller connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.r.sleep()

        # Set the initial waypoint as the goal waypoint
        pose = self.waypoint_goal

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

            error_pos, heading_error_norm, pose = self.local_planner.waypoint_nav(pose)

            yaw = calc_yaw(pose.pose.orientation)

            # Log the waypoint index and error to the console
            # msg = "Waypoint {index:} Pos.x {x:} Pos.y {y:} Pos.z {z:} YAW: {yaw:} Error Position: {error:} Error Yaw: {er:}"
            # rospy.loginfo_throttle(1, msg.format(index=self.waypoint_index, error=error_pos,
            #                                      x=pose.pose.position.x,
            #                                      y=pose.pose.position.y,
            #                                      z=pose.pose.position.z,
            #                                      yaw=yaw,
            #                                      er=heading_error_norm,
            #                                      ))

            # Publish the goal waypoint to the drone's local position topic
            self.local_pos_pub.publish(pose)
            # self.setpoint_vel_pub.publish(twist_msg)

            # Visualization
            if self.vis:
                # If visualization is enabled
                # Publish the goal waypoint as a green marker
                marker = vis_marker(pose.pose, 0, 1, 0, 0, 0.08)
                self.vis_goal_pub.publish(marker)

            # Sleep for a short period of time to ensure that the drone's position is updated before the next loop
            # iteration
            self.r.sleep()
