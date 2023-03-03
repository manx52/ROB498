#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


class DroneControl:
    def __init__(self):
        self.drone_onboard_pose = PoseStamped()
        self.drone_vicon_pose = PoseStamped()
        self.current_state = State()

        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.vicon_sub = rospy.Subscriber("/vicon/VICON_NAME/VICON_NAME", PoseStamped, self.vicon_callback)

        self.vicon_enabled = False

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arm_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    def pose_callback(self, msg):
        self.drone_onboard_pose = msg

    def state_callback(self, msg):
        self.current_state = msg

    def vicon_callback(self, msg):
        if self.vicon_enabled:
            self.drone_vicon_pose = msg

    def run(self):
        r = rospy.Rate(20)
        # Wait for Flight Controller connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            r.sleep()

        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2

        # Send a few setpoints before starting
        for i in range(100):
            if rospy.is_shutdown():
                break

            self.local_pos_pub.publish(pose)
            r.sleep()

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

            self.local_pos_pub.publish(pose)

            r.sleep()

