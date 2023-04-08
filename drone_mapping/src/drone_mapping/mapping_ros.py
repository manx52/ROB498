#!/usr/bin/env python3
import sys

import numpy as np
import ros_numpy
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2

from drone_mapping.mapping import Mapping
from drone_mapping.utils import l2p

np.set_printoptions(threshold=sys.maxsize)


class MappingROS:
    """
    The MappingROS class is responsible for creating and publishing the occupancy grid map.
    """

    def __init__(self):
        """
        The constructor of MappingROS class.

        Initializes the attributes of the class and creates a OccupancyGrid message template.
        """

        self.gridmapping = None

        # Initializes the ROS node with name "drone_mapping"
        rospy.init_node('drone_mapping', anonymous=True)

        # Flag to check whether the gridmapping is initialized or not
        self.is_gridmapping_initialized = False

        # Time at which the last map was published
        self.map_last_publish = rospy.Time()

        # Map settings
        self.robot_frame = rospy.get_param('/drone_mapping/robot_frame', 'base_link')
        self.map_frame = rospy.get_param('/drone_mapping/map_frame', 'map')
        self.map_center_x = rospy.get_param('/drone_mapping/map_center_x', -5)
        self.map_center_y = rospy.get_param('/drone_mapping/map_center_y', -5)
        self.map_size_x = rospy.get_param('/drone_mapping/map_size_x', 10.0)
        self.map_size_y = rospy.get_param('/drone_mapping/map_size_y', 10.0)
        self.map_resolution = rospy.get_param('/drone_mapping/map_resolution', 0.1)
        self.map_publish_freq = rospy.get_param('/drone_mapping/map_publish_freq', 1.0)
        self.update_movement = rospy.get_param('/drone_mapping/update_movement', 0.1)
        self.obs_col_rad = rospy.get_param('/drone_mapping/obs_col_rad', 0.4)
        self.drone_col_rad = rospy.get_param('/drone_mapping/drone_col_rad', 0.4)

        # Creata a OccupancyGrid message template
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.map_frame
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(self.map_size_x / self.map_resolution)
        self.map_msg.info.height = int(self.map_size_y / self.map_resolution)
        self.map_msg.info.origin.position.x = self.map_center_x
        self.map_msg.info.origin.position.y = self.map_center_y

        # The subscribers to PointCloud2 for green and red mask point clouds
        self.point_cloud_green_sub = rospy.Subscriber("green_mask_point_cloud", PointCloud2, self.green_obs_callback,
                                                      queue_size=1)
        self.point_cloud_red_sub = rospy.Subscriber("red_mask_point_cloud", PointCloud2, self.red_obs_callback,
                                                    queue_size=1)

        # Publisher for OccupancyGrid map
        self.map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=2)

        # ROS Rate
        self.r = rospy.Rate(20)

    def init_mapping(self):
        """
        Initializes the gridmapping object, which is responsible for mapping out the
        environment of the drone based on its sensors and dimensions.

        :return: None
        """

        # Create a new Mapping object with the specified parameters
        self.gridmapping = Mapping(self.map_center_x, self.map_center_y, self.map_size_x, self.map_size_y,
                                   self.map_resolution, self.drone_col_rad, self.obs_col_rad)

        # Set the flag indicating that the gridmapping object has been initialized
        self.is_gridmapping_initialized = True

    def publish_occupancygrid(self, gridmap: np.ndarray, stamp):
        """
        Convert gridmap to ROS supported data type : int8[]
        https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html The map data, in row-major order,
        starting with (0,0).  Occupancy probabilities are in the range [0,100].  Unknown is -1.

        :param gridmap: the occupancy grid map as a numpy array
        :param stamp: the timestamp of the map
        :return: None
        """

        # Convert occupancy grid map to probability map
        gridmap_p = l2p(gridmap)

        # Scale probability map from [0,1] to [0,100] and convert to int8
        gridmap_int8 = (gridmap_p * 100).astype(dtype=np.int8)

        # Publish map
        self.map_msg.data = gridmap_int8
        self.map_msg.header.stamp = stamp
        self.map_pub.publish(self.map_msg)
        rospy.loginfo_once("Published map!")

    def run(self):
        """
        The run method is used to continuously execute the main loop of the node.

        :return: None
        """

        while not rospy.is_shutdown():
            if not self.is_gridmapping_initialized:
                self.init_mapping()

            # If the map needs to be published
            elif self.map_last_publish.to_sec() + 1.0 / self.map_publish_freq < rospy.Time.now().to_sec():
                self.map_last_publish = rospy.Time.now()

                # Get the grid map data and flatten it
                gridmap = self.gridmapping.get_map().flatten()

                self.publish_occupancygrid(gridmap, rospy.Time.now())

            self.r.sleep()

    def add_obs(self, msg: PointCloud2):
        """
        This method adds an incoming point cloud message to the grid map, updates it, and publishes the map with the
        specified frequency.

        :param msg: PointCloud2 message containing the point cloud data.
        :return: None
        """

        # If grid mapping is not initialized, initialize it
        if not self.is_gridmapping_initialized:
            self.init_mapping()

        try:
            # Convert PointCloud2 message to a numpy array of XYZ coordinates
            xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

            # Update the grid map with the incoming point cloud
            gridmap = self.gridmapping.update(xyz_array).flatten()  # update map

            # Publish the occupancy grid map with the specified frequency
            if self.map_last_publish.to_sec() + 1.0 / self.map_publish_freq < rospy.Time.now().to_sec():
                self.map_last_publish = rospy.Time.now()
                self.publish_occupancygrid(gridmap, msg.header.stamp)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)

    def green_obs_callback(self, msg: PointCloud2):
        """
        A callback function for processing green obstacles.

        :param msg: A PointCloud2 message containing information about the green obstacles.
        :return: None
        """

        # Call the add_obs method with the msg argument to add the green obstacle data to the map
        self.add_obs(msg)

    def red_obs_callback(self, msg: PointCloud2):
        """
        A callback function for processing red obstacles.

        :param msg: A PointCloud2 message containing information about the red obstacles.
        :return: None
        """

        # Call the add_obs method with the msg argument to add the red obstacle data to the map
        self.add_obs(msg)
