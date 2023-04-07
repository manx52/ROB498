#!/usr/bin/env python3
from typing import Tuple

import numpy as np
from skimage.draw import circle


class GridMapping:
    """

    """

    def __init__(self, map_center_x: float, map_center_y: float, map_size_x: float, map_size_y: float,
                 map_resolution: float) -> None:
        """

        :param map_center_x:
        :param map_center_y:
        :param map_size_x:
        :param map_size_y:
        :param map_resolution:
        """
        self.map_center_x = map_center_x  # meter
        self.map_center_y = map_center_y  # meter
        self.map_size_x = map_size_x  # meter
        self.map_size_y = map_size_y  # meter
        self.map_resolution = map_resolution
        self.map_rows = int(map_size_y / map_resolution)
        self.map_cols = int(map_size_x / map_resolution)

        self.gridmap = np.zeros((self.map_rows, self.map_cols))
        self.collision_radius_pix = 0.35 // self.map_resolution

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

    def is_inside(self, i: float, j: float) -> bool:  # TODO use for outliers
        """

        :param i:
        :param j:
        :return:
        """

        return 0 <= i < self.gridmap.shape[0] < self.gridmap.shape[1] and j >= 0

    def update(self, pts: np.ndarray) -> np.ndarray:
        # test by printing robot trajectory
        rr = []
        cc = []

        for pt in pts:
            i, j = self.to_ij(pt[0], pt[1])
            # self.gridmap[int(i), int(j)] = 100
            temp_rr, temp_cc = circle(int(i), int(j), int(self.collision_radius_pix))
            rr.append(temp_rr)
            cc.append(temp_cc)
        traj_rr = np.array(rr)
        traj_cc = np.array(cc)
        # print(pts, traj_rr,traj_cc)
        footprint = np.moveaxis(np.array([traj_rr, traj_cc]), 0, 2)
        temp_x = np.clip(footprint[..., 0], 0, self.gridmap.shape[0] - 1).astype(int)
        temp_y = np.clip(footprint[..., 1], 0, self.gridmap.shape[1] - 1).astype(int)

        self.gridmap[temp_x, temp_y] = 100

        return self.gridmap

    def get_map(self):
        return self.gridmap
