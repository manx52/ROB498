#!/usr/bin/env python3
import os
import numpy as np
import rospy
from drone.drone_comm import DroneComm
np.set_printoptions(precision=3)

if __name__ == "__main__":
    # Log an informative message to the ROS console
    rospy.loginfo("Initializing Drone Comm")

    # Create a new instance of the DroneComm class
    drone = DroneComm()

    # Try to run the drone communication code
    try:
        drone.run()

    # Catch any ROS exceptions and exit gracefully
    except rospy.exceptions.ROSException as ex:
        exit(0)
