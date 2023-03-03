#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_srvs.srv import Empty, EmptyResponse


class DroneComm:
    def __init__(self):
        print('This is a dummy drone node to test communication with the ground control')
        print('node_name should be rob498_drone_TeamID. Service topics should follow the name convention below')
        print('The TAs will test these service calls prior to flight')
        print('Your own code should be integrated into this node')

        node_name = 'rob498_drone_07'
        rospy.init_node(node_name)
        self.srv_launch = rospy.Service(node_name + '/comm/launch', Empty, self.callback_launch)
        self.srv_test = rospy.Service(node_name + '/comm/test', Empty, self.callback_test)
        self.srv_land = rospy.Service(node_name + '/comm/land', Empty, self.callback_land)
        self.srv_abort = rospy.Service(node_name + '/comm/abort', Empty, self.callback_abort)

        self.drone_onboard_pose = PoseStamped()
        self.drone_vicon_pose = PoseStamped()
        self.current_state = State()

        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.vicon_sub = rospy.Subscriber("/vicon/ROB498_Drone/ROB498_Drone", PoseStamped, self.vicon_callback)

        self.vicon_enabled = False

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arm_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.r = rospy.Rate(20)

    # Callback handlers
    def pose_callback(self, msg):
        self.drone_onboard_pose = msg

    def state_callback(self, msg):
        self.current_state = msg

    def vicon_callback(self, msg):
        if self.vicon_enabled:
            self.drone_vicon_pose = msg

    def handle_launch(self):
        print('Launch Requested. Your drone should take off.')
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.r.sleep()

        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 1.5

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
        x_diff = pose.pose.position.x - self.drone_onboard_pose.pose.position.x
        y_diff = pose.pose.position.y - self.drone_onboard_pose.pose.position.y
        z_diff = pose.pose.position.z - self.drone_onboard_pose.pose.position.z
        error_pose = math.sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)
        while error_pose > 0.01:

            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.arm_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()
            x_diff = pose.pose.position.x - self.drone_onboard_pose.pose.position.x
            y_diff = pose.pose.position.y - self.drone_onboard_pose.pose.position.y
            z_diff = pose.pose.position.z - self.drone_onboard_pose.pose.position.z
            error_pose = math.sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)
            print(error_pose)
            self.local_pos_pub.publish(pose)

            self.r.sleep()
        # Send a few setpoints before starting
        for i in range(100):
            if rospy.is_shutdown():
                break

            self.local_pos_pub.publish(pose)
            self.r.sleep()

    def handle_test(self):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 1.5
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
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
        while not rospy.is_shutdown(): # TODO need a time limit
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.arm_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
            self.local_pos_pub.publish(pose)

            self.r.sleep()

    def handle_land(self):
        print('Land Requested. Your drone should land.')
        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0

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
        x_diff = pose.pose.position.x - self.drone_onboard_pose.pose.position.x
        y_diff = pose.pose.position.y - self.drone_onboard_pose.pose.position.y
        z_diff = pose.pose.position.z - self.drone_onboard_pose.pose.position.z
        error_pose = math.sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)
        while error_pose > 0.01:
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.arm_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
            x_diff = pose.pose.position.x - self.drone_onboard_pose.pose.position.x
            y_diff = pose.pose.position.y - self.drone_onboard_pose.pose.position.y
            z_diff = pose.pose.position.z - self.drone_onboard_pose.pose.position.z
            error_pose = math.sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)
            self.local_pos_pub.publish(pose)

            self.r.sleep()
        # Send a few setpoints before starting
        for i in range(100):
            if rospy.is_shutdown():
                break

            self.local_pos_pub.publish(pose)
            self.r.sleep()

    def handle_abort(self):
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0

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
        x_diff = pose.pose.position.x - self.drone_onboard_pose.pose.position.x
        y_diff = pose.pose.position.y - self.drone_onboard_pose.pose.position.y
        z_diff = pose.pose.position.z - self.drone_onboard_pose.pose.position.z
        error_pose = math.sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)
        while error_pose > 0.01:
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.arm_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
            x_diff = pose.pose.position.x - self.drone_onboard_pose.pose.position.x
            y_diff = pose.pose.position.y - self.drone_onboard_pose.pose.position.y
            z_diff = pose.pose.position.z - self.drone_onboard_pose.pose.position.z
            error_pose = math.sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)

            self.local_pos_pub.publish(pose)

            self.r.sleep()

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

    # Main communication node for ground control
    def run(self):
        # Your code goes below
        while not rospy.is_shutdown():
            self.r.sleep()


if __name__ == "__main__":
    comm = DroneComm()
    try:
        comm.run()
    except rospy.exceptions.ROSException as ex:
        exit(0)
