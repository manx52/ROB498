#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <vector>
//#include "/home/manx52/catkin_ws/src/Firmware/build/px4_sitl_default/build_gazebo-classic/Odometry.pb.h"
//#include <Odometry.pb.h>
//typedef const boost::shared_ptr<const gz_geometry_msgs::Odometry> GzOdometryMsgPtr;
//ros::Publisher pub;
//typedef const boost::shared_ptr<const nav_msgs::Odometry> OdomPtr;
//// Position callback function
//void odom_callback(GzOdometryMsgPtr& msg){
//     nav_msgs::Odometry output;
//     output.pose = msg.pose;
//     output.twist = msg.twist;
//     output.header.frame_id = msg.header.frame_id;
//     output.child_frame_id = msg.child_frame_id;
////     output.pose = msg->pose;
////     output.twist = msg->twist;
////     output.header.frame_id = msg->header.frame_id;
////     output.child_frame_id = msg->child_frame_id;
//     pub.publish(output);
//}

int main(int _argc, char **_argv){
    // Load Gazebo & ROS
//    gazebo::client::setup(_argc, _argv);
//    ros::init(_argc, _argv, "sim_odom");
//
//    // Create Gazebo node and init
//    gazebo::transport::NodePtr node;
//    node = gazebo::transport::NodePtr(new gazebo::transport::Node());
//    node->Init();
//
//    // Create ROS node and init
//    ros::NodeHandle n;
//    pub = n.advertise<nav_msgs::Odometry>("/camera/odom/sample_throttled", 10); // Same topic as VIO
//
//    // Listen to Gazebo contacts topic
//    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/iris_vision/vision_odom", odom_callback);
//
//    // Busy wait loop...replace with your own code as needed.
//  while (true){
//     gazebo::common::Time::MSleep(20);
//    }
//  // Spin ROS (needed for publisher) // (nope its actually for subscribers-calling callbacks ;-) )
//  ros::spinOnce();
//    gazebo::client::shutdown();
}