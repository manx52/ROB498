cmake_minimum_required(VERSION 3.0.2)
project(drone)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(
    catkin
    REQUIRED
    COMPONENTS
    rospy
    roscpp
    dynamic_reconfigure
      tf
      pcl_ros
      mavros
      mavros_extras
      mavros_msgs
      mavlink
      gazebo_ros
      gazebo_msgs
      rotors_description
      px4_realsense_bridge
      drone_perception
      drone_msgs
      realsense2_camera

)
find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/sim_odom.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_package(DEPENDS
      gazebo_ros
      gazebo_msgs
    CATKIN_DEPENDS
    roscpp rospy std_msgs mavros_msgs geometry_msgs mav_msgs sensor_msgs  tf drone_perception
      drone_msgs realsense2_camera px4_realsense_bridge
    INCLUDE_DIRS
    LIBRARIES)
catkin_python_setup()
