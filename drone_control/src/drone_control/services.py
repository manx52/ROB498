#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray, TransformStamped, Point, Quaternion, Pose
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Int8
from std_srvs.srv import Empty, EmptyResponse

from drone_control.utils_math import calc_yaw


class Services:
    """
    This class handles the services for launching, testing, landing, and aborting the drone. It contains callback functions
    for each of these services that update boolean flags, which in turn trigger the handle functions that actually perform
    the requested action. It also subscribes to the 'test_waypoints' topic to receive commands for performing different
    waypoint tests.
    """

    def __init__(self, node, node_name):
        """
        Constructor function for the Services class.

        :param node: The main node that this service belongs to.
        :param node_name: The name of the node.
        """

        # Initialize attributes
        self.node = node
        self.bool_launch = False
        self.bool_test = False
        self.bool_land = False
        self.bool_abort = False

        # Services
        self.srv_launch = rospy.Service(node_name + '/comm/launch', Empty, self.callback_launch)
        self.srv_test = rospy.Service(node_name + '/comm/test', Empty, self.callback_test)
        self.srv_land = rospy.Service(node_name + '/comm/land', Empty, self.callback_land)
        self.srv_abort = rospy.Service(node_name + '/comm/abort', Empty, self.callback_abort)

        # Wait for service clients to be available
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arm_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.test_vicon = rospy.get_param("/rob498_drone_07/test_vicon")

        self.unlimited_test = rospy.get_param("/rob498_drone_07/unlimited_waypts")

        # Subscribe to 'test_waypoints' topic
        self.test_env = rospy.Subscriber(node_name + '/comm/test_env', Int8,
                                         self.callback_test_env)
        self.test_waypoints = rospy.Subscriber(node_name + '/comm/test_waypoints', Int8,
                                               self.callback_test_waypoints) \
 \
            # Publisher
        self.send_vicon = rospy.Publisher("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, queue_size=10)
        self.send_waypts = rospy.Publisher(node_name + '/comm/test_waypoints', Int8, queue_size=10)

    def handle_launch(self):
        """

        Helper function that sets the boolean flags and sets the goal position for the drone to takeoff.
        """
        print('Launch Requested. Your drone should take off.')
        self.bool_launch = True
        self.bool_test = False
        self.bool_land = False
        self.bool_abort = False

        # Set goal position for takeoff
        self.node.waypoint_goal.header.stamp = rospy.Time.now()
        self.node.waypoint_goal.pose.position.x = 0
        self.node.waypoint_goal.pose.position.y = 0
        self.node.waypoint_goal.pose.position.z = self.node.launch_height - self.node.offset

    def handle_test(self):
        """

        Helper function that sets the boolean flags for the drone to perform a test.
        """
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

        if self.unlimited_test:
            self.node.waypoint_index = 0
            self.node.navigation.sub_points_once = False
            self.node.navigation.sub_points_index = 0
        self.bool_launch = False
        self.bool_test = True
        self.bool_land = False
        self.bool_abort = False

    def handle_land(self):  # TODO fix landing bug
        """

        Helper function that sets the boolean flags and sets the goal position for the drone to land.
        """
        print('Land Requested. Your drone should land.')
        self.bool_launch = False
        self.bool_test = False
        self.bool_land = True
        self.bool_abort = False

        # Set goal position for landing
        self.node.waypoint_goal.header.stamp = rospy.Time.now()
        self.node.waypoint_goal.pose.position.x = 0  # self.drone_onboard_pose.pose.position.x
        self.node.waypoint_goal.pose.position.y = 0  # self.drone_onboard_pose.pose.position.y
        self.node.waypoint_goal.pose.position.z = self.node.land_offset

    def handle_abort(self):  # TODO Figure out logic
        """

        Helper function that sets the boolean flags and sets the goal position for the drone to land.
        """
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        self.bool_launch = False
        self.bool_test = False
        self.bool_land = False
        self.bool_abort = True

        # Set goal position for landing
        self.node.waypoint_goal.header.stamp = rospy.Time.now()
        self.node.waypoint_goal.pose.position.x = 0  # self.drone_onboard_pose.pose.position.x
        self.node.waypoint_goal.pose.position.y = 0  # self.drone_onboard_pose.pose.position.y
        self.node.waypoint_goal.pose.position.z = self.node.land_offset

    # Service callbacks
    def callback_launch(self, request):
        """
        This function handles the launch service, which triggers the launch action of the drone.
        """
        self.handle_launch()
        return EmptyResponse()

    def callback_test(self, request):
        """
        This function handles the test service, which tests the drone's behavior without actually launching it.
        """
        self.handle_test()
        return EmptyResponse()

    def callback_land(self, request):
        """
        This function handles the land service, which triggers the landing action of the drone.
        """
        self.handle_land()
        return EmptyResponse()

    def callback_abort(self, request):
        """
        This function handles the abort service, which triggers the abort action of the drone.
        """
        self.handle_abort()
        return EmptyResponse()

    def callback_test_waypoints(self, msg):
        """
        This function handles the test_waypoints service, which tests the drone's behavior by sending it to a sequence of
        pre-defined waypoints.

        :param msg: Type of test waypoints
        """
        rospy.loginfo("Test Waypoints")

        # set the test waypoints based on the requested type
        waypoints_test = PoseArray()
        waypoints_test.header.stamp = rospy.Time.now()
        if msg.data == 4:
            # 4 pt square
            if self.test_vicon:
                pt1 = Point(-2, -2, self.node.launch_height - self.node.offset)
                pt2 = Point(-2, 2, self.node.launch_height - self.node.offset)
                pt3 = Point(2, 2, self.node.launch_height - self.node.offset)
                pt4 = Point(2, -2, self.node.launch_height - self.node.offset)
            else:
                pt1 = Point(2, 0, self.node.launch_height - self.node.offset)
                pt2 = Point(2, 2, self.node.launch_height - self.node.offset)
                pt3 = Point(0, 2, self.node.launch_height - self.node.offset)
                pt4 = Point(0, 0, self.node.launch_height - self.node.offset)

            q1 = Quaternion(0, 0, 0, 1)
            waypoints_test.poses = [
                Pose(pt1, q1),
                Pose(pt2, q1),
                Pose(pt3, q1),
                Pose(pt4, q1)
            ]
        elif msg.data == 8:
            # 8 pt square

            pt1 = Point(1, 0, self.node.launch_height - self.node.offset)
            pt2 = Point(2, 0, self.node.launch_height - self.node.offset)
            pt3 = Point(1.25, -3.5, self.node.launch_height - self.node.offset)
            pt4 = Point(3, -5, self.node.launch_height - self.node.offset)
            pt5 = Point(0.5, -6, self.node.launch_height - self.node.offset)
            pt6 = Point(0, -5, self.node.launch_height - self.node.offset)
            pt7 = Point(-1.8, -4, self.node.launch_height - self.node.offset)
            pt8 = Point(0, 0, self.node.launch_height - self.node.offset)

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

        # Send waypoints
        self.node.callback_waypoints(waypoints_test)

    def callback_test_env(self, msg):
        """
        Callback function to setup a test environment.

        :param msg: Message received by the callback function.
        :return: None
        """
        rospy.loginfo("Test Setup")

        # Set up vicon_pose with default values.
        if self.test_vicon:
            vicon_pose = TransformStamped()
            vicon_pose.transform.translation.x = -3.0
            vicon_pose.transform.translation.y = 3.0
            vicon_pose.transform.translation.z = 0.0
            vicon_pose.transform.rotation.x = 0.0
            vicon_pose.transform.rotation.y = 0.0
            # Yaw = 90 degree
            vicon_pose.transform.rotation.z = 0.7071068  # 0.0
            vicon_pose.transform.rotation.w = 0.7071068  # 1.0
            print(calc_yaw(vicon_pose.transform.rotation))
            # Publish vicon_pose message to send_vicon topic.
        else:
            vicon_pose = TransformStamped()
            vicon_pose.transform.translation.x = 0
            vicon_pose.transform.translation.y = 0
            vicon_pose.transform.translation.z = 0.0
            vicon_pose.transform.rotation.x = 0.0
            vicon_pose.transform.rotation.y = 0.0
            # Yaw = 90 degree
            vicon_pose.transform.rotation.z = 0.0
            vicon_pose.transform.rotation.w = 1.0

        self.send_vicon.publish(vicon_pose)

        # Set up waypt_num with the data received from the message.
        waypt_num = Int8()
        waypt_num.data = msg.data

        # Publish waypt_num message to send_waypts topic.
        self.send_waypts.publish(waypt_num)

        # Call handle_launch function to launch drone.
        self.handle_launch()
