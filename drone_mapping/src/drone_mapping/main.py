#!/usr/bin/env python3
import numpy as np
import rospy
from drone_mapping.grid_mapping_ros import GridMappingROS

if __name__ == "__main__":
    # Log an informative message to the ROS console
    rospy.loginfo("Initializing Drone Map")

    # Create a new instance of the DroneComm class
    gm = GridMappingROS()
    while not rospy.is_shutdown():
        gm.run()
        rospy.spin()

