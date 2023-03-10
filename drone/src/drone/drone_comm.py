#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, TransformStamped, Point, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import Odometry


class DroneComm:
    def __init__(self):
        print('This is a dummy drone node to test communication with the ground control')
        print('node_name should be rob498_drone_TeamID. Service topics should follow the name convention below')
        print('The TAs will test these service calls prior to flight')
        print('Your own code should be integrated into this node')

        node_name = 'rob498_drone_07'
        rospy.init_node(node_name)

        # Settings
        self.offset = 0.1
        self.launch_height = 1.5

        # Simulation
        self.sim = rospy.get_param("/simulation")
        if self.sim:
            # self.gazebo_odom_sub = rospy.Subscriber('mavros/local_position/pose', Odometry, self.odom_callback)
            # self.vio_odom_sub = rospy.Subscriber('/camera/odom/sample_throttled', Odometry, self.odom_callback)
            self.mavros_odom_pub_ = rospy.Publisher("/mavros/odometry/out", Odometry, queue_size=10)

        # Flags
        self.bool_launch = False
        self.bool_test = False
        self.bool_land = False
        self.bool_abort = False
        self.vicon_enabled = False
        self.challenge2 = False

        # Var
        self.drone_onboard_pose = PoseStamped()
        self.drone_vicon_pose = PoseStamped()

        self.waypoint_goal = PoseStamped()
        self.waypoint_goal.header.stamp = rospy.Time.now()
        # self.waypoint_goal.header.frame_id = "world"  # TODO
        self.waypoint_goal.pose.position.x = 0
        self.waypoint_goal.pose.position.y = 0
        self.waypoint_goal.pose.position.z = 0
        self.waypoints = PoseArray()
        self.waypoints.header.stamp = rospy.Time.now()
        # self.waypoints.header.frame_id = "world"  # TODO
        self.waypoint_index = 0

        self.current_state = State()
        self.r = rospy.Rate(30)

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

        # Publishers
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.vicon_pose_pub = rospy.Publisher("mavros/vision_pose/pose", PoseStamped, queue_size=10)

    # Callback Sensors
    def pose_callback(self, msg):
        self.drone_onboard_pose = msg

    def state_callback(self, msg):
        self.current_state = msg

    def vicon_callback(self, msg):
        if self.vicon_enabled:
            self.drone_vicon_pose.header = msg.header
            self.drone_vicon_pose.pose.position.x = msg.transform.translation.x
            self.drone_vicon_pose.pose.position.y = msg.transform.translation.y
            self.drone_vicon_pose.pose.position.z = msg.transform.translation.z
            self.drone_vicon_pose.pose.orientation = msg.transform.rotation
            self.vicon_pose_pub.publish(self.drone_vicon_pose)

    # Callback handlers

    def handle_launch(self):
        print('Launch Requested. Your drone should take off.')
        self.bool_launch = True
        self.bool_test = False
        self.bool_land = False
        self.bool_abort = False

        self.waypoint_goal.header.stamp = rospy.Time.now()
        self.waypoint_goal.pose.position.x = 0
        self.waypoint_goal.pose.position.y = 0
        self.waypoint_goal.pose.position.z = self.launch_height - self.offset

    def handle_test(self):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        self.bool_launch = False
        self.bool_test = True
        self.bool_land = False
        self.bool_abort = False

        if self.challenge2:
            self.waypoint_goal.header.stamp = rospy.Time.now()
            self.waypoint_goal.pose.position.x = 0
            self.waypoint_goal.pose.position.y = 0
            self.waypoint_goal.pose.position.z = self.launch_height - self.offset
        else:
            self.waypoint_index = 0

            # 4 pt square
            pt1 = Point(1, 0, self.launch_height - self.offset)
            pt2 = Point(1, 1, self.launch_height - self.offset)
            pt3 = Point(0, 1, self.launch_height - self.offset)
            pt4 = Point(0, 0, self.launch_height - self.offset)
            q1 = Quaternion(0, 0, 0, 1)
            self.waypoints.poses = [
                Pose(pt1, q1),
                Pose(pt2, q1),
                Pose(pt3, q1),
                Pose(pt4, q1)
            ]
            # 8 pt square
            # pt1 = Point(1, 0, self.launch_height - self.offset)
            # pt2 = Point(1, 1, self.launch_height - self.offset)
            # pt3 = Point(1, 2, self.launch_height - self.offset)
            # pt4 = Point(0, 2, self.launch_height - self.offset)
            # pt5 = Point(-1, 2, self.launch_height - self.offset)
            # pt6 = Point(-1, 1, self.launch_height - self.offset)
            # pt7 = Point(-1, 0, self.launch_height - self.offset)
            # pt8 = Point(0, 0, self.launch_height - self.offset)
            #
            # q1 = Quaternion(0, 0, 0, 1)
            # self.waypoints.poses = [
            #     Pose(pt1, q1),
            #     Pose(pt2, q1),
            #     Pose(pt3, q1),
            #     Pose(pt4, q1),
            #     Pose(pt5, q1),
            #     Pose(pt6, q1),
            #     Pose(pt7, q1),
            #     Pose(pt8, q1),
            # ]


    def handle_land(self):
        print('Land Requested. Your drone should land.')
        self.bool_launch = False
        self.bool_test = False
        self.bool_land = True
        self.bool_abort = False

        self.waypoint_goal.header.stamp = rospy.Time.now()
        self.waypoint_goal.pose.position.x = self.drone_onboard_pose.pose.position.x
        self.waypoint_goal.pose.position.y = self.drone_onboard_pose.pose.position.y
        self.waypoint_goal.pose.position.z = -0.3 - self.offset

    def handle_abort(self):
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        self.bool_launch = False
        self.bool_test = False
        self.bool_land = False
        self.bool_abort = True

        self.waypoint_goal.header.stamp = rospy.Time.now()
        self.waypoint_goal.pose.position.x = self.drone_onboard_pose.pose.position.x
        self.waypoint_goal.pose.position.y = self.drone_onboard_pose.pose.position.y
        self.waypoint_goal.pose.position.z = -0.3 - self.offset

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

    # Util
    def waypoint_error(self, pose):
        x_diff = pose.pose.position.x - self.drone_onboard_pose.pose.position.x
        y_diff = pose.pose.position.y - self.drone_onboard_pose.pose.position.y
        z_diff = pose.pose.position.z - self.drone_onboard_pose.pose.position.z
        error_pose = math.sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)

        return error_pose

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

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while not rospy.is_shutdown():

            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.arm_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()
            # Update waypoint
            if self.bool_test:
                # Are we there yet?
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.pose = self.waypoints.poses[self.waypoint_index]
                error_pose = self.waypoint_error(pose)

                if error_pose < 0.1 and self.waypoint_index < (len(self.waypoints.poses)-1):  # TODO tune
                    self.waypoint_index += 1
                    pose.header.stamp = rospy.Time.now()
                    pose.pose = self.waypoints.poses[self.waypoint_index]
            else:
                pose = self.waypoint_goal

            error_pose = self.waypoint_error(pose)
            msg = "Waypoint {index:} error: {error:}"
            rospy.loginfo_throttle(1, msg.format(index=self.waypoint_index, error=error_pose))

            # Send goal waypoint
            self.local_pos_pub.publish(pose)

            self.r.sleep()


if __name__ == "__main__":
    comm = DroneComm()
    try:
        comm.run()
    except rospy.exceptions.ROSException as ex:
        exit(0)
