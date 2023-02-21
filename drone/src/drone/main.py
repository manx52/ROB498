#!/usr/bin/env python3
import os
import numpy as np
import rospy
from drone.drone_control import DroneControl
np.set_printoptions(precision=3)

if __name__ == "__main__":
    rospy.init_node("drone")
    rospy.loginfo("Initializing Drone Code")
    drone = DroneControl()
    rospy.loginfo("Starting Control Loop")
    try:
        drone.run()
    except rospy.exceptions.ROSException as ex:
        exit(0)
