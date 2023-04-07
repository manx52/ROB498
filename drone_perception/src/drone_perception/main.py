#!/usr/bin/env python3
import rospy

from drone_perception.detector_obstacles import DetectorObstacles

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