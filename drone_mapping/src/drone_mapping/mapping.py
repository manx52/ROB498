#!/usr/bin/env python3
import math
from typing import Tuple

import numpy as np
import scipy.spatial as sp
from skimage.draw import circle


class Mapping:
    """
    This class represents a grid map used for obstacle avoidance and path planning in robotics.
    The map is centered at a given location and has a fixed size in meters, resolution in meters per pixel,
    and maximum collision radii for obstacles and the drone.
    """

    def __init__(self, map_center_x: float, map_center_y: float, map_size_x: float, map_size_y: float,
                 map_resolution: float, drone_col_rad: float, obs_col_rad: float, dir_enable: bool,
                 fake_obs: bool) -> None:
        """
        Initializes a new instance of the GridMapping class.

        :param map_center_x: the x-coordinate of the center of the map, in meters
        :param map_center_y: the y-coordinate of the center of the map, in meters
        :param map_size_x: the width of the map, in meters
        :param map_size_y: the height of the map, in meters
        :param map_resolution: the resolution of the map, in meters per pixel
        :param drone_col_rad: the maximum collision radius of the drone, in meters
        :param obs_col_rad: the maximum collision radius of obstacles, in meters
        :param dir_enable: Enable collision checking based on type of obstacle
        :param fake_obs: Enable fake obstacles in environment
        """

        # Enable dir
        self.fake_obs = fake_obs
        self.dir_enable = dir_enable

        # Map setting
        self.map_center_x = map_center_x  # set the x-coordinate of the center of the map
        self.map_center_y = map_center_y  # set the y-coordinate of the center of the map
        self.map_size_x = map_size_x  # set the width of the map
        self.map_size_y = map_size_y  # set the height of the map
        self.map_resolution = map_resolution  # set the resolution of the map, in meters per pixel
        self.map_rows = int(map_size_y / map_resolution)  # calculate the number of rows in the map
        self.map_cols = int(map_size_x / map_resolution)  # calculate the number of columns in the map

        # create an empty grid map with the specified dimensions
        self.gridmap = np.zeros((self.map_rows, self.map_cols))

        # set boundary edge of map
        for row in range(self.map_rows):
            for col in range(self.map_cols):
                if row == 0 or row == (self.map_rows - 1) or col == 0 or col == (self.map_cols - 1):
                    self.gridmap[row, col] = 80

        # create an empty list to store the indices of non-zero elements in the grid map
        self.map_nonzero_idxes = []

        # convert the maximum collision radii from meters to pixels, and round up to the nearest integer
        self.obs_collision_radius_pix = int(math.ceil(obs_col_rad / self.map_resolution))
        self.drone_collision_radius_pix = int(math.ceil(drone_col_rad / self.map_resolution))

        # Update fake obstacles
        if self.fake_obs:
            self.update_fake_obs()

    def grid_to_coord(self, i: int, j: int) -> Tuple[float, float]:
        """
        Convert the grid coordinates (i, j) to real-world coordinates (x, y) using the map resolution and center.

        :param i: the row index of the grid
        :param j: the column index of the grid
        :return: a tuple containing the (x, y) coordinates in the real-world
        """

        x = j * self.map_resolution + self.map_center_x
        y = i * self.map_resolution + self.map_center_y
        return x, y

    def coord_to_grid(self, x: float, y: float) -> Tuple[float, float]:
        """
        Converts a set of (x,y) coordinates to grid indices based on the given map center and resolution.

        :param x: The x coordinate.
        :param y: The y coordinate.
        :return: A tuple of the corresponding (i,j) grid indices.
        """

        i = (y - self.map_center_y) / self.map_resolution
        j = (x - self.map_center_x) / self.map_resolution
        return i, j

    def update(self, pts: np.ndarray, typ: int) -> np.ndarray:
        """
        This function converts a given trajectory to robot circle points with the provided collision radius and then
        converts these points to the indices of the corresponding grid cells. It sets the corresponding grid cells to
        100 to represent obstacles.

        :param typ: type of obstacle to add to the map
        :param pts: numpy array of shape (n_points, 2) representing the x and y coordinates of the trajectory points
        :return: numpy array of shape (map_height, map_width) representing the updated grid map
        """

        # Convert trajectory points to robot circle points
        traj_rr, traj_cc = self.points_to_robot_circle(pts, self.obs_collision_radius_pix)

        # Combine x and y coordinates into a single array
        footprint = np.moveaxis(np.array([traj_rr, traj_cc]), 0, 2)

        # Clip points to be within the boundaries of the grid map
        temp_x = np.clip(footprint[..., 0], 0, self.gridmap.shape[0] - 1).astype(int)
        temp_y = np.clip(footprint[..., 1], 0, self.gridmap.shape[1] - 1).astype(int)

        # Set the corresponding grid cells to 100 (representing an obstacle)
        self.gridmap[temp_x, temp_y] = typ

        # Return the updated grid map
        return self.gridmap

    def check_collision(self, traj_mat: np.ndarray, y_in_map_pix: float, x_in_map_pix: float,
                        debug: bool = False) -> list:
        """
        Check if any trajectory in the given matrix of trajectories collides with any obstacle on the map.

        :param traj_mat: matrix of drone trajectories, with shape (num_trajectories, num_timesteps, 2)
        :param y_in_map_pix: y-coordinate of the drone's current position in pixels on the map
        :param x_in_map_pix: x-coordinate of the drone's current position in pixels on the map
        :param debug: whether to print debug information or not
        :return: list of indices of trajectories that collide with obstacles
        """

        # Initialize empty list to store indices of trajectories that collide with obstacles
        collision_traj_idx = []

        # Update obstacle positions in map by finding all non-zero indices
        nonzero = np.nonzero(self.gridmap)
        self.map_nonzero_idxes = np.transpose(nonzero)

        # Calculate distance towards nearest obstacle and filter based on norm distance
        close_wall = [i for i in self.map_nonzero_idxes if
                      np.linalg.norm(i - [y_in_map_pix, x_in_map_pix]) < 300]

        if self.dir_enable:
            green_idxes = np.transpose(np.where(self.gridmap == 50))
            red_idxes = np.transpose(np.where(self.gridmap == 100))
            close_green = [i for i in green_idxes if
                           np.linalg.norm(i - [y_in_map_pix, x_in_map_pix]) < 300]

            close_red = [i for i in red_idxes if
                         np.linalg.norm(i - [y_in_map_pix, x_in_map_pix]) < 300]

        closest = 'red'

        # if debug:
        #     print("self.map_nonzero_idxes: ", self.map_nonzero_idxes)
        #     print("close_wall ", close_wall)
        # print("dir_enable: ", self.dir_enable)
        # # print("green_idxes: ",green_idxes)
        # print("Length green: ", len(close_green))
        # # print("green_idxes: ", red_idxes)
        # print("Length red: ", len(close_red))

        if len(close_wall) > 0:

            # Going through all trajectories using kdtrees calculate distance from each point to obstacles
            for opt in range(traj_mat.shape[0]):
                for timestep in range(traj_mat.shape[1]-1):
                    x_diff = (traj_mat[opt, timestep + 1, 0] - traj_mat[opt, timestep, 0]) / 9
                    y_diff = (traj_mat[opt, timestep + 1, 1] - traj_mat[opt, timestep, 1]) / 9
                    for i in range(10):
                        temp_pose_x = traj_mat[opt, timestep, 0] + x_diff * i
                        temp_pose_y = traj_mat[opt, timestep, 1] + y_diff * i

                        # Get current location of the drone
                        curr_loc = np.array(
                            [int(temp_pose_x), int(temp_pose_y)])

                        # Use kdtree to find the closest obstacle to the current location of the drone
                        if timestep == 2 and self.dir_enable and opt == 0:
                            if len(close_green) > 0:
                                kdtree_green = sp.cKDTree(close_green)
                                green_d, green_i = kdtree_green.query(curr_loc.T, k=1)
                            else:
                                green_d = float('inf')

                            if len(close_red) > 0:
                                kdtree_red = sp.cKDTree(close_red)
                                red_d, red_i = kdtree_red.query(curr_loc.T, k=1)
                            else:
                                red_d = float('inf')

                            if green_d < red_d:
                                closest = 'green'
                            else:
                                closest = 'red'

                            if debug:
                                print("opt: ", opt, "D: ", green_d, red_d, len(close_green), len(close_red))

                        if opt in [0, 8, 9, 10, 11, 12,13] and closest == 'green' and self.dir_enable:
                            collision_traj_idx.append(opt)
                            break

                        elif opt in [0, 1, 2, 3, 4, 5, 6,7] and closest == 'red' and self.dir_enable:
                            collision_traj_idx.append(opt)
                            break

                        kdtree = sp.cKDTree(close_wall)
                        d, i = kdtree.query(curr_loc.T, k=1)

                        if debug:
                            print("opt: ", opt, "D: ", d)

                        # If distance to the closest obstacle is less than the drone's collision radius, add index of
                        # trajectory to the collision_traj_idx list and break out of inner loop
                        if d < self.drone_collision_radius_pix:
                            collision_traj_idx.append(opt)
                            break

        if len(collision_traj_idx) == traj_mat.shape[0]:
            collision_traj_idx = []
            if len(close_wall) > 0:

                # Going through all trajectories using kdtrees calculate distance from each point to obstacles
                for opt in range(traj_mat.shape[0]):
                    for timestep in range(traj_mat.shape[1] - 1):
                        x_diff = (traj_mat[opt, timestep + 1, 0] - traj_mat[opt, timestep, 0]) / 4
                        y_diff = (traj_mat[opt, timestep + 1, 1] - traj_mat[opt, timestep, 1]) / 4
                        for i in range(5):
                            temp_pose_x = traj_mat[opt, timestep, 0] + x_diff * i
                            temp_pose_y = traj_mat[opt, timestep, 1] + y_diff * i

                            # Get current location of the drone
                            curr_loc = np.array(
                                [int(temp_pose_x), int(temp_pose_y)])
                            kdtree = sp.cKDTree(close_wall)

                            d, i = kdtree.query(curr_loc.T, k=1)

                            if debug:
                                print("opt: ", opt, "D: ", d)

                            # If distance to the closest obstacle is less than the drone's collision radius, add index of
                            # trajectory to the collision_traj_idx list and break out of inner loop
                            if d < self.drone_collision_radius_pix:
                                collision_traj_idx.append(opt)
                                break

        # Return the list of indices of trajectories that collide with obstacles
        return collision_traj_idx

    def points_to_robot_circle(self, points: np.ndarray, collision_radius: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        This function takes in an array of points and a collision radius, and returns the indices of a circle with that radius
        centered at each point in the input array.

        :param points: An array of points of shape (n,2), where n is the number of points and the second dimension
                       consists of x,y coordinates.
        :param collision_radius: The radius of the circle to be generated at each point.
        :return: A tuple containing two arrays. The first is an array of row indices for the circle around each point,
                 and the second is an array of column indices.
        """

        # Initialize empty lists to store row and column indices of circles around each point
        rr = []
        cc = []

        # Iterate over each point in the input array
        for pt in points:
            # Convert the x,y coordinates to grid coordinates
            i, j = self.coord_to_grid(pt[0], pt[1])

            # Generate row and column indices for circle
            temp_rr, temp_cc = circle(int(i), int(j), collision_radius)
            rr.append(temp_rr)
            cc.append(temp_cc)

        # Convert the lists of row and column indices to numpy arrays and return them as a tuple
        return np.array(rr), np.array(cc)

    def get_map(self) -> np.ndarray:
        """
        This method returns the gridmap of the object.

        :return: The numpy array representing the gridmap.
        """
        return self.gridmap

    def update_fake_obs(self):
        """

        :return:
        """
        # obs 1: 1,0 red
        temp_pts = np.array([-3*0.61,-3*0.61]).reshape((1, 2))
        self.update(temp_pts, 100)

        # obs 2: 2,1 green
        temp_pts = np.array([4.1*0.61, -2*0.61]).reshape((1, 2))
        self.update(temp_pts, 50)

        # obs 3: 1,2 red
        temp_pts = np.array([1.5*0.61, 3.5*0.61]).reshape((1, 2))
        self.update(temp_pts, 100)

        # obs 4: 0,1 green
        temp_pts = np.array([-4*0.61, 2.5*0.61]).reshape((1, 2))
        self.update(temp_pts, 50)
