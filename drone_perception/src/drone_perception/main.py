#!/usr/bin/env python3
import numpy as np
import rospy

from drone_perception.detector_obstacles import DetectorObstacles

np.set_printoptions(precision=3)

if __name__ == "__main__":
    # Log an informative message to the ROS console
    rospy.init_node("detector_obstacles")

    # Try to run the drone communication code
    try:
        obstacle_detector = DetectorObstacles()
        rospy.spin()

    # Catch any ROS exceptions and exit gracefully
    except rospy.exceptions.ROSException as ex:
        exit(0)