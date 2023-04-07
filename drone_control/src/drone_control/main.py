#!/usr/bin/env python3
import rospy

from drone_control.drone_comm import DroneComm

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
