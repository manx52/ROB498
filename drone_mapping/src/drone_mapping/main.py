#!/usr/bin/env python3
import rospy

from drone_mapping.mapping_ros import MappingROS

if __name__ == "__main__":
    # Log an informative message to the ROS console
    rospy.loginfo("Initializing Drone Map")

    # Create a new instance of the DroneComm class
    gm = MappingROS()
    gm.run()


