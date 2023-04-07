#!/usr/bin/env python3
import sys

import numpy as np
import ros_numpy
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2

from drone_mapping.grid_mapping import GridMapping
from drone_mapping.utils import l2p
np.set_printoptions(threshold=sys.maxsize)

class GridMappingROS:
    """

    """

    def __init__(self) -> None:
        """

        """
        self.gridmapping = None
        rospy.init_node('drone_mapping', anonymous=True)
        self.is_gridmapping_initialized = False
        self.map_last_publish = rospy.Time()
        self.prev_robot_x = -99999999
        self.prev_robot_y = -99999999

        self.robot_frame = rospy.get_param('/drone_mapping/robot_frame', 'base_link')
        self.map_frame = rospy.get_param('/drone_mapping/map_frame', 'map')
        self.map_center_x = rospy.get_param('/drone_mapping/map_center_x', -5)
        self.map_center_y = rospy.get_param('/drone_mapping/map_center_y', -5)
        self.map_size_x = rospy.get_param('/drone_mapping/map_size_x', 10.0)
        self.map_size_y = rospy.get_param('/drone_mapping/map_size_y', 10.0)
        self.map_resolution = rospy.get_param('/drone_mapping/map_resolution', 0.1)
        self.map_publish_freq = rospy.get_param('/drone_mapping/map_publish_freq', 1.0)
        self.update_movement = rospy.get_param('/drone_mapping/update_movement', 0.1)

        # Creata a OccupancyGrid message template
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.map_frame
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(self.map_size_x / self.map_resolution)
        self.map_msg.info.height = int(self.map_size_y / self.map_resolution)
        self.map_msg.info.origin.position.x = self.map_center_x
        self.map_msg.info.origin.position.y = self.map_center_y

        self.point_cloud_green_sub = rospy.Subscriber("green_mask_point_cloud", PointCloud2, self.green_obs_callback,
                                                      queue_size=1)
        self.point_cloud_red_sub = rospy.Subscriber("red_mask_point_cloud", PointCloud2, self.red_obs_callback,
                                                    queue_size=1)
        self.map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=2)
        self.tf_sub = tf.TransformListener()
        self.r = rospy.Rate(20)

    def init_gridmapping(self) -> None:
        """

        :return:
        """
        self.gridmapping = GridMapping(self.map_center_x, self.map_center_y, self.map_size_x, self.map_size_y,
                                       self.map_resolution)
        self.is_gridmapping_initialized = True

    def publish_occupancygrid(self, gridmap: np.ndarray, stamp) -> None:
        """
        Convert gridmap to ROS supported data type : int8[]
        https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html The map data, in row-major order,
        starting with (0,0).  Occupancy probabilities are in the range [0,100].  Unknown is -1.

        :param gridmap:
        :param stamp:
        :return:
        """

        gridmap_p = l2p(gridmap)
        gridmap_int8 = (gridmap_p * 100).astype(dtype=np.int8)

        # Publish map
        self.map_msg.data = gridmap_int8
        self.map_msg.header.stamp = stamp
        self.map_pub.publish(self.map_msg)
        rospy.loginfo_once("Published map!")

    def run(self) -> None:
        """

        :return:
        """
        while not rospy.is_shutdown():
            if not self.is_gridmapping_initialized:
                self.init_gridmapping()

            # TODO is it needed ?
            elif self.map_last_publish.to_sec() + 1.0 / self.map_publish_freq < rospy.Time.now().to_sec():
                self.map_last_publish = rospy.Time.now()
                # temp = self.gridmapping.get_map()
                # idx = np.where(temp == 100)
                # print("Map: ", idx)
                gridmap = self.gridmapping.get_map().flatten()
                self.publish_occupancygrid(gridmap, rospy.Time.now())

            self.r.sleep()

    def add_obs(self, msg: PointCloud2) -> None:
        if not self.is_gridmapping_initialized:
            self.init_gridmapping()

        try:
            xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

            # print(xyz_array, msg)
            gridmap = self.gridmapping.update(xyz_array).flatten()  # update map

            # publish map (with the specified frequency)
            if self.map_last_publish.to_sec() + 1.0 / self.map_publish_freq < rospy.Time.now().to_sec():
                self.map_last_publish = rospy.Time.now()
                self.publish_occupancygrid(gridmap, msg.header.stamp)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)

    def green_obs_callback(self, msg: PointCloud2) -> None:
        """

        :param msg:
        :return:
        """
        self.add_obs(msg)

    def red_obs_callback(self, msg: PointCloud2) -> None:
        """

        :param msg:
        :return:
        """
        self.add_obs(msg)
