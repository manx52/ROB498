import rospy
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

from drone_perception import PID


class LocalPlanner:
    def __init__(self, node):
        """
        Constructor for the LocalPlanner class.
        :param node: Main ROS node
        """
        self.set_yaw = 0
        self.node = node

        self.max_vel = 0.35
        self.set_x = 0
        self.set_y = 0
        self.set_z = 0

        self.linear_x_pid = PID(
            Kp=rospy.get_param("linear_x_Kp", 1.0),
            Kd=rospy.get_param("linear_x_Kd", 0.0),
            Ki=rospy.get_param("linear_x_Ki", 0.0),
            setpoint=0.0,
            output_limits=(-self.max_vel, self.max_vel),
        )

        self.linear_y_pid = PID(
            Kp=rospy.get_param("linear_y_Kp", 1.0),
            Kd=rospy.get_param("linear_y_Kd", 0.0),
            Ki=rospy.get_param("linear_y_Ki", 0.0),
            setpoint=0.0,
            output_limits=(-self.max_vel, self.max_vel),
        )

        self.linear_z_pid = PID(
            Kp=rospy.get_param("linear_z_Kp", 1.0),
            Kd=rospy.get_param("linear_z_Kd", 0.0),
            Ki=rospy.get_param("linear_z_Ki", 0.0),
            setpoint=0.0,
            output_limits=(-1.0, 1.0),
        )

        self.angular_z_pid = PID(
            Kp=rospy.get_param("angular_z_Kp", 2.0),
            Kd=rospy.get_param("angular_z_Kd", 0.0),
            Ki=rospy.get_param("angular_z_Ki", 0.0),
            setpoint=0.0,
            output_limits=(-2.0, 2.0),
        )

    def new_point(self, pose: PoseStamped):
        self.set_x = pose.pose.position.x
        self.set_y = pose.pose.position.y
        self.set_z = pose.pose.position.z
        Q = pose.pose.orientation
        euler = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])
        self.set_yaw = euler[2]
        self.linear_x_pid.reset(self.set_x)
        self.linear_y_pid.reset(self.set_y)
        self.linear_z_pid.reset(self.set_z)
        self.angular_z_pid.reset(self.set_yaw)

    def velocity_command(self, pose):
        """
        Calculate the velocity command required to move the drone towards a given pose

        :param pose: ROS Pose message indicating the target position
        :return: ROS Twist message indicating the required velocity command
        """
        vel = Twist()
        vel.linear.x = self.linear_x_pid.update(pose.pose.position.x)
        vel.linear.y = self.linear_y_pid.update(pose.pose.position.y)
        vel.linear.z = self.linear_z_pid.update(pose.pose.position.z)

        Q = pose.pose.orientation
        euler = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])

        vel.angular.z = self.angular_z_pid.update(euler[2])

        return vel
