#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseArray, TransformStamped, Point, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int8
import numpy as np
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import tf

class DroneComm:
    def __init__(self):
        node_name = rospy.get_param("/node_name")
        rospy.init_node(node_name)

        # Settings
        self.offset = rospy.get_param("/offset")
        self.land_offset = rospy.get_param("/land_offset")
        self.launch_height = rospy.get_param("/launch_height")
        self.vis = rospy.get_param("/vis")

        # Simulation
        self.sim = rospy.get_param("/simulation")
        if self.sim:
            self.test_waypoints = rospy.Subscriber(node_name + '/comm/test_waypoints', Int8,
                                                   self.callback_test_waypoints)
        #     # self.gazebo_odom_sub = rospy.Subscriber('mavros/local_position/pose', Odometry, self.odom_callback)
        #     # self.vio_odom_sub = rospy.Subscriber('/camera/odom/sample_throttled', Odometry, self.odom_callback)
        #     self.mavros_odom_pub_ = rospy.Publisher("/mavros/odometry/out", Odometry, queue_size=10)

        # Flags
        self.bool_launch = False
        self.bool_test = False
        self.bool_land = False
        self.bool_abort = False
        self.vicon_enabled = rospy.get_param("/vicon_enabled")
        self.challenge4 = rospy.get_param("/challenge4")

        # Var
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

        self.r = rospy.Rate(rospy.get_param("/rate"))

        # Services
        self.srv_launch = rospy.Service(node_name + '/comm/launch', Empty, self.callback_launch)
        self.srv_test = rospy.Service(node_name + '/comm/test', Empty, self.callback_test)
        self.srv_land = rospy.Service(node_name + '/comm/land', Empty, self.callback_land)
        self.srv_abort = rospy.Service(node_name + '/comm/abort', Empty, self.callback_abort)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arm_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # Subscribers
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        self.vicon_sub = rospy.Subscriber("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, self.vicon_callback)
        self.sub_waypoints = rospy.Subscriber(node_name + '/comm/waypoints', PoseArray, self.callback_waypoints)

        # Publishers
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.setpoint_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        self.vicon_pose_pub = rospy.Publisher("mavros/vision_pose/pose", PoseStamped, queue_size=10)

        self.br = tf.TransformBroadcaster()
        if self.vis:
            self.vis_goal_pub = rospy.Publisher(node_name + '/comm/vis_goal', Marker, queue_size=1)
            self.vis_waypoints_pub = rospy.Publisher(node_name + '/comm/vis_waypoints', MarkerArray, queue_size=1)
            self.vis_path_pub = rospy.Publisher(node_name + '/comm/vis_path', Path, queue_size=1)

    # Callback Sensors
    def pose_callback(self, msg):
        self.drone_pose = msg

        self.br.sendTransform(
            [self.drone_pose.pose.position.x, self.drone_pose.pose.position.y , self.drone_pose.pose.position.z],
            [self.drone_pose.pose.orientation.x,self.drone_pose.pose.orientation.y,self.drone_pose.pose.orientation.z,self.drone_pose.pose.orientation.w,],
            msg.header.stamp,
            "drone/base_link",
            "odom",
        )
        # Visualization
        if self.vis:
            self.path.header.stamp = rospy.Time.now()
            self.path.header.frame_id = "map"
            self.path.poses.append(self.drone_pose)
            self.vis_path_pub.publish(self.path)

    def state_callback(self, msg):
        self.current_state = msg

    def vicon_callback(self, msg):
        if self.vicon_enabled:
            self.drone_pose.header = msg.header
            self.drone_pose.pose.position.x = msg.transform.translation.x
            self.drone_pose.pose.position.y = msg.transform.translation.y
            self.drone_pose.pose.position.z = msg.transform.translation.z
            self.drone_pose.pose.orientation = msg.transform.rotation
            self.vicon_pose_pub.publish(self.drone_pose)

    def callback_test_waypoints(self, msg):
        rospy.loginfo("Test Waypoints")

        waypoints_test = PoseArray()
        waypoints_test.header.stamp = rospy.Time.now()
        if msg.data == 4:
            # 4 pt square
            pt1 = Point(1, 0, self.launch_height - self.offset)
            pt2 = Point(1, 1, self.launch_height - self.offset)
            pt3 = Point(0, 1, self.launch_height - self.offset)
            pt4 = Point(0, 0, self.launch_height - self.offset)
            q1 = Quaternion(0, 0, 0, 1)
            waypoints_test.poses = [
                Pose(pt1, q1),
                Pose(pt2, q1),
                Pose(pt3, q1),
                Pose(pt4, q1)
            ]
        elif msg.data == 8:
            # 8 pt square
            pt1 = Point(1, 0, self.launch_height - self.offset)
            pt2 = Point(1, 1, self.launch_height - self.offset)
            pt3 = Point(1, 2, self.launch_height - self.offset)
            pt4 = Point(0, 2, self.launch_height - self.offset)
            pt5 = Point(-1, 2, self.launch_height - self.offset)
            pt6 = Point(-1, 1, self.launch_height - self.offset)
            pt7 = Point(-1, 0, self.launch_height - self.offset)
            pt8 = Point(0, 0, self.launch_height - self.offset)

            q1 = Quaternion(0, 0, 0, 1)
            waypoints_test.poses = [
                Pose(pt1, q1),
                Pose(pt2, q1),
                Pose(pt3, q1),
                Pose(pt4, q1),
                Pose(pt5, q1),
                Pose(pt6, q1),
                Pose(pt7, q1),
                Pose(pt8, q1),
            ]
        self.callback_waypoints(waypoints_test)

    # Callback handlers

    def handle_launch(self):
        print('Launch Requested. Your drone should take off.')
        self.bool_launch = True
        self.bool_test = False
        self.bool_land = False
        self.bool_abort = False

        self.waypoint_goal.header.stamp = rospy.Time.now()
        self.waypoint_goal.pose.position.x = 1
        self.waypoint_goal.pose.position.y = 0
        self.waypoint_goal.pose.position.z = self.launch_height - self.offset

    def handle_test(self):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        self.bool_launch = False
        self.bool_test = True
        self.bool_land = False
        self.bool_abort = False

    def handle_land(self):  # TODO fix landing bug
        print('Land Requested. Your drone should land.')
        self.bool_launch = False
        self.bool_test = False
        self.bool_land = True
        self.bool_abort = False

        self.waypoint_goal.header.stamp = rospy.Time.now()
        self.waypoint_goal.pose.position.x = 0  # self.drone_onboard_pose.pose.position.x
        self.waypoint_goal.pose.position.y = 0  # self.drone_onboard_pose.pose.position.y
        self.waypoint_goal.pose.position.z = self.land_offset

    def handle_abort(self):  # TODO Figure out logic
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        self.bool_launch = False
        self.bool_test = False
        self.bool_land = False
        self.bool_abort = True

        self.waypoint_goal.header.stamp = rospy.Time.now()
        self.waypoint_goal.pose.position.x = 0  # self.drone_onboard_pose.pose.position.x
        self.waypoint_goal.pose.position.y = 0  # self.drone_onboard_pose.pose.position.y
        self.waypoint_goal.pose.position.z = self.land_offset

    # Service callbacks
    def callback_launch(self, request):
        self.handle_launch()
        return EmptyResponse()

    def callback_test(self, request):
        self.handle_test()
        return EmptyResponse()

    def callback_land(self, request):
        self.handle_land()
        return EmptyResponse()

    def callback_abort(self, request):
        self.handle_abort()
        return EmptyResponse()

    def callback_waypoints(self, msg):
        if self.WAYPOINTS_RECEIVED:
            return

        rospy.loginfo("Waypoints Received")

        self.waypoint_index = 0
        self.WAYPOINTS_RECEIVED = True
        self.waypoints.header.stamp = rospy.Time.now()
        self.waypoints.poses = msg.poses

        # Visualization
        if self.vis:
            markerArray = MarkerArray()
            for i, pt in enumerate(self.waypoints.poses):
                marker = self.vis_marker(pt, 1, 0, 0, 0)
                markerArray.markers.append(marker)
                markerArray.markers[i].id = i

            self.vis_waypoints_pub.publish(markerArray)

    # Util
    def positon_diff(self, pose):
        x_diff = pose.pose.position.x - self.drone_pose.pose.position.x
        y_diff = pose.pose.position.y - self.drone_pose.pose.position.y
        z_diff = pose.pose.position.z - self.drone_pose.pose.position.z

        return x_diff, y_diff, z_diff

    def waypoint_error(self, pose):
        x_diff, y_diff, z_diff = self.positon_diff(pose)
        error_pose = math.sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)

        return error_pose

    def velocity_command(self, pose):
        vel = Twist()
        x_diff, y_diff, z_diff = self.positon_diff(pose)

        max_vel = 0.5
        x_clip = np.clip(x_diff, -max_vel, max_vel)
        y_clip = np.clip(y_diff, -max_vel, max_vel)
        z_clip = np.clip(z_diff, -max_vel, max_vel)
        vel.linear.x = x_clip
        vel.linear.y = y_clip
        vel.linear.z = z_clip

        return vel

    def vis_marker(self, pose, r, g, b, action):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.SPHERE
        marker.action = action
        marker.scale.x = 0.125
        marker.scale.y = 0.125
        marker.scale.z = 0.125
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z
        return marker

    # Main communication node for ground control
    def run(self):

        # Wait for Flight Controller connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.r.sleep()

        pose = self.waypoint_goal

        # Send a few setpoints before starting
        for i in range(100):
            if rospy.is_shutdown():
                break

            self.local_pos_pub.publish(pose)
            self.r.sleep()

        if self.sim:
            offb_set_mode = SetModeRequest()
            offb_set_mode.custom_mode = 'OFFBOARD'

            arm_cmd = CommandBoolRequest()
            arm_cmd.value = True

            last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.sim:
                if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.set_mode_client.call(offb_set_mode).mode_sent:
                        rospy.loginfo("OFFBOARD enabled")

                    last_req = rospy.Time.now()
                else:
                    if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                        if self.arm_client.call(arm_cmd).success:
                            rospy.loginfo("Vehicle armed")

                        last_req = rospy.Time.now()
            pose = PoseStamped()
            pose.header.frame_id = "map"
            markerArray = MarkerArray()
            # Update waypoint
            if self.bool_test and self.WAYPOINTS_RECEIVED:
                # Are we there yet?
                pose.pose = self.waypoints.poses[self.waypoint_index]
                error_pose = self.waypoint_error(pose)

                # Visualization
                if self.vis:
                    for i, pt in enumerate(self.waypoints.poses):
                        if i <= self.waypoint_index:
                            marker = self.vis_marker(pt, 1, 0, 0, 2)
                            markerArray.markers.append(marker)
                            markerArray.markers[i].id = i
                        else:
                            marker = self.vis_marker(pt, 1, 0, 0, 0)
                            markerArray.markers.append(marker)
                            markerArray.markers[i].id = i

                    self.vis_waypoints_pub.publish(markerArray)

                if error_pose < 0.1 and self.waypoint_index < (len(self.waypoints.poses) - 1):  # TODO tune
                    self.waypoint_index += 1
                    pose.pose = self.waypoints.poses[self.waypoint_index]


            else:
                pose = self.waypoint_goal
                error_pose = self.waypoint_error(pose)

            pose.header.stamp = rospy.Time.now()
            msg = "Waypoint {index:} error: {error:}"
            rospy.loginfo_throttle(1, msg.format(index=self.waypoint_index, error=error_pose))

            # Send goal waypoint
            self.local_pos_pub.publish(pose)

            # Visualization
            if self.vis:
                marker = self.vis_marker(pose.pose, 0, 1, 0, 0)
                self.vis_goal_pub.publish(marker)

        self.r.sleep()
