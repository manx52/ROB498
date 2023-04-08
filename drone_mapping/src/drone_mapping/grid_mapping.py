#!/usr/bin/env python3
import math
from typing import Tuple

import numpy as np
import scipy.spatial as sp
from skimage.draw import circle

from drone_mapping.utils import cost_to_come


class GridMapping:
    """

    """

    def __init__(self, map_center_x: float, map_center_y: float, map_size_x: float, map_size_y: float,
                 map_resolution: float, drone_col_rad: float, obs_col_rad: float) -> None:
        """

        :param map_center_x:
        :param map_center_y:
        :param map_size_x:
        :param map_size_y:
        :param map_resolution:
        """
        self.pos = None
        self.traj_matrix = None
        self.map_center_x = map_center_x  # meter
        self.map_center_y = map_center_y  # meter
        self.map_size_x = map_size_x  # meter
        self.map_size_y = map_size_y  # meter
        self.map_resolution = map_resolution
        self.map_rows = int(map_size_y / map_resolution)
        self.map_cols = int(map_size_x / map_resolution)

        self.gridmap = np.zeros((self.map_rows, self.map_cols))
        self.map_nonzero_idxes = []

        self.obs_collision_radius_pix = int(math.ceil(obs_col_rad / self.map_resolution))
        self.drone_collision_radius_pix = int(math.ceil(drone_col_rad / self.map_resolution))

    def to_xy(self, i: int, j: int) -> Tuple[float, float]:
        """

        :param i:
        :param j:
        :return:
        """
        x = j * self.map_resolution + self.map_center_x
        y = i * self.map_resolution + self.map_center_y
        return x, y

    def to_ij(self, x: float, y: float) -> Tuple[float, float]:
        """

        :param x:
        :param y:
        :return:
        """
        i = (y - self.map_center_y) / self.map_resolution
        j = (x - self.map_center_x) / self.map_resolution
        return i, j

    def calc_idx(self, pts: np.ndarray, collision_radius: int) -> Tuple[np.ndarray, np.ndarray]:
        traj_rr, traj_cc = self.points_to_robot_circle(pts, collision_radius)

        # print(pts, traj_rr,traj_cc)
        footprint = np.moveaxis(np.array([traj_rr, traj_cc]), 0, 2)
        temp_x = np.clip(footprint[..., 0], 0, self.gridmap.shape[0] - 1).astype(int)
        temp_y = np.clip(footprint[..., 1], 0, self.gridmap.shape[1] - 1).astype(int)

        return temp_x, temp_y

    def update(self, pts: np.ndarray) -> np.ndarray:
        # test by printing robot trajectory

        temp_x, temp_y = self.calc_idx(pts, self.obs_collision_radius_pix)
        self.gridmap[temp_x, temp_y] = 100

        return self.gridmap

    # def check_collision(self, pts: np.ndarray, ) -> bool:
    #
    #     temp_x, temp_y = self.calc_idx(pts, self.drone_collision_radius_pix)
    #
    #     return np.any(np.any(self.gridmap[temp_x, temp_y] == 100, axis=-1))

    def check_collision(self, traj_mat: np.ndarray, pos: np.ndarray) -> list:
        collision_traj_idx = []
        self.pos = pos[:2]
        self.traj_matrix = traj_mat
        y_in_map_pix, x_in_map_pix = self.to_ij(pos[0], pos[1])

        # convert all trajectories to pixel coord
        for i, traj in enumerate(traj_mat):  # All trajectories
            for j, pts in enumerate(traj):  # All pts in trajectory
                y, x = self.to_ij(pts[0], pts[1])
                traj_mat[i, j, 0] = y
                traj_mat[i, j, 1] = x

        nonzero = np.nonzero(self.gridmap)
        # print(self.gridmap)
        self.map_nonzero_idxes = np.transpose(nonzero)

        close_wall = [i for i in self.map_nonzero_idxes if
                      np.linalg.norm(i - [y_in_map_pix, x_in_map_pix]) < 300]
        # print("self.map_nonzero_idxes: ", self.map_nonzero_idxes)
        # print("close_wall ",close_wall)
        if len(close_wall) > 0:
            for opt in range(traj_mat.shape[0]):
                for timestep in range(traj_mat.shape[1]):
                    curr_loc = np.array(
                        [int(traj_mat[opt, timestep, 0]), int(traj_mat[opt, timestep, 1])])
                    kdtree = sp.cKDTree(close_wall)
                    d, i = kdtree.query(curr_loc.T, k=1)
                    # print("opt: ", opt,"D: ", d)
                    if d < self.drone_collision_radius_pix:
                        collision_traj_idx.append(opt)
                        break

        return collision_traj_idx

    def best_path(self, valid_opts, num_opts):
        final_cost = np.Inf * np.ones(num_opts)
        for i in range(num_opts):
            if i in valid_opts:
                final_cost[i] = cost_to_come(self.traj_matrix[i])

                # print("in else")
        # print("cost to go", final_cost)
        # print(final_cost)
        # print(local_paths)
        if final_cost.min == np.Inf:  # TODO
            best_opt = 0
        else:
            best_opt = final_cost.argmin()

        return best_opt

    def points_to_robot_circle(self, points: np.ndarray, collision_radius: int) -> Tuple[np.ndarray, np.ndarray]:
        rr = []
        cc = []

        for pt in points:
            i, j = self.to_ij(pt[0], pt[1])

            # self.gridmap[int(i), int(j)] = 100
            temp_rr, temp_cc = circle(int(i), int(j), collision_radius)
            rr.append(temp_rr)
            cc.append(temp_cc)

        return np.array(rr), np.array(cc)

    def get_map(self):
        return self.gridmap
