#!/usr/bin/env python
import os
import numpy as np
import rospy
from drone.drone_comm import DroneComm
np.set_printoptions(precision=3)

if __name__ == "__main__":
    rospy.loginfo("Initializing Drone Comm")
    drone = DroneComm()
    try:
        drone.run()
    except rospy.exceptions.ROSException as ex:
        exit(0)
