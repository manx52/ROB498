cmake_minimum_required(VERSION 3.0.2)
project(drone_msgs)

find_package(
    catkin
    REQUIRED
    COMPONENTS message_generation message_runtime roscpp rospy std_msgs
               geometry_msgs
)

add_message_files(
    FILES BoundingBox.msg BoundingBoxes.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    CATKIN_DEPENDS
    message_generation
    roscpp
    rospy
    std_msgs
    message_runtime
    geometry_msgs
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)
