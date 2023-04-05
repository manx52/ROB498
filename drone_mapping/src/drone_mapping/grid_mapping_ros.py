#!/usr/bin/env python3
import numpy as np
import ros_numpy
import rospy
import time
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
import tf

from drone_mapping.grid_mapping import GridMapping, l2p


class GridMappingROS:
    def __init__(self):
        self.gridmapping = None
        rospy.init_node('RosGridMapping', anonymous=True)
        self.is_gridmapping_initialized = False
        self.map_last_publish = rospy.Time()
        self.prev_robot_x = -99999999
        self.prev_robot_y = -99999999

        self.robot_frame = rospy.get_param('robot_frame', 'base_link')
        self.map_frame = rospy.get_param('map_frame', 'map')
        self.map_center_x = rospy.get_param('map_center_x', -5)
        self.map_center_y = rospy.get_param('map_center_y', -5)
        self.map_size_x = rospy.get_param('map_size_x', 10.0)
        self.map_size_y = rospy.get_param('map_size_y', 10.0)
        self.map_resolution = rospy.get_param('map_resolution', 0.1)
        self.map_publish_freq = rospy.get_param('map_publish_freq', 1.0)
        self.update_movement = rospy.get_param('update_movement', 0.1)

        # Creata a OccupancyGrid message template
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.map_frame
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(self.map_size_x / self.map_resolution)
        self.map_msg.info.height = int(self.map_size_y / self.map_resolution)
        self.map_msg.info.origin.position.x = self.map_center_x
        self.map_msg.info.origin.position.y = self.map_center_y

        self.laser_sub = rospy.Subscriber("green_mask_point_cloud", PointCloud2, self.green_obs_callback, queue_size=2)
        self.map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=2)
        self.tf_sub = tf.TransformListener()

    def init_gridmapping(self):
        self.gridmapping = GridMapping(self.map_center_x, self.map_center_y, self.map_size_x, self.map_size_y,
                                       self.map_resolution)
        self.is_gridmapping_initialized = True

    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion
    def quarternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return np.arctan2(siny_cosp, cosy_cosp)

    def publish_occupancygrid(self, gridmap, stamp):
        # Convert gridmap to ROS supported data type : int8[]
        # http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
        # The map data, in row-major order, starting with (0,0).  Occupancy probabilities are in the range [0,100].  Unknown is -1.
        gridmap_p = l2p(gridmap)
        # unknown_mask = (gridmap_p == self.sensor_model_p_prior)  # for setting unknown cells to -1
        gridmap_int8 = (gridmap_p * 100).astype(dtype=np.int8)
        # gridmap_int8[unknown_mask] = -1  # for setting unknown cells to -1

        # Publish map
        self.map_msg.data = gridmap_int8
        self.map_msg.header.stamp = stamp
        self.map_pub.publish(self.map_msg)
        rospy.loginfo_once("Published map!")

    def run(self):
        if (self.map_last_publish.to_sec() + 1.0 / self.map_publish_freq < rospy.Time.now().to_sec()):
            self.map_last_publish = rospy.Time.now()
            self.publish_occupancygrid(self.gridmapping.gridmap, rospy.Time.now())

    def green_obs_callback(self, msg):
        if not self.is_gridmapping_initialized:
            self.init_gridmapping()

        self.tf_sub.waitForTransform(self.map_frame, self.robot_frame, msg.header.stamp, rospy.Duration(1.0))
        try:
            # get the robot position associated with the current laserscan
            (x, y, _), (qx, qy, qz, qw) = self.tf_sub.lookupTransform(self.map_frame, self.robot_frame,
                                                                      msg.header.stamp)
            theta = self.quarternion_to_yaw(qx, qy, qz, qw)
            xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

            # check the movement if update is needed
            # if ((x - self.prev_robot_x) ** 2 + (y - self.prev_robot_y) ** 2 >= self.update_movement ** 2):

            gridmap = self.gridmapping.update(x, y, theta, xyz_array).flatten()  # update map
            self.prev_robot_x = x
            self.prev_robot_y = y

            # publish map (with the specified frequency)
            if (self.map_last_publish.to_sec() + 1.0 / self.map_publish_freq < rospy.Time.now().to_sec()):
                self.map_last_publish = rospy.Time.now()
                self.publish_occupancygrid(gridmap, msg.header.stamp)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)
