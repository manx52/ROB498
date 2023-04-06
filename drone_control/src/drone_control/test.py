# !/usr/bin/env python3
from geometry_msgs.msg import Point

from drone_control.utils import *

A = PoseStamped()
B = PoseStamped()

A.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
A.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

A.pose.position = Point(x=0, y=0, z=1)
B.pose.position = Point(x=1, y=0, z=1)
waypoint_pose_error(A.pose, B, True)

A.pose.position = Point(x=1, y=0, z=1)
B.pose.position = Point(x=1, y=1, z=1)
waypoint_pose_error(A.pose, B, True)

A.pose.position = Point(x=1, y=1, z=1)
B.pose.position = Point(x=0, y=1, z=1)
waypoint_pose_error(A.pose, B, True)

A.pose.position = Point(x=0, y=1, z=1)
B.pose.position = Point(x=0, y=0, z=1)
waypoint_pose_error(A.pose, B, True)
