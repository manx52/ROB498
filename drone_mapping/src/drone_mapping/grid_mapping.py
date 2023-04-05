#!/usr/bin/env python3

import numpy as np
from skimage.draw import circle_perimeter


# p(x) = 1 - \frac{1}{1 + e^l(x)}
def l2p(l):
    return 1 - (1 / (1 + np.exp(l)))


# l(x) = log(\frac{p(x)}{1 - p(x)})
def p2l(p):
    return np.log(p / (1 - p))


class GridMapping:
    def __init__(self, map_center_x, map_center_y, map_size_x, map_size_y, map_resolution):
        self.map_center_x = map_center_x  # meter
        self.map_center_y = map_center_y  # meter
        self.map_size_x = map_size_x  # meter
        self.map_size_y = map_size_y  # meter
        self.map_resolution = map_resolution
        map_rows = int(map_size_y / map_resolution)
        map_cols = int(map_size_x / map_resolution)

        self.gridmap = np.zeros((map_rows, map_cols))
        self.collision_radius_pix = 0.3 // self.map_resolution
    def to_xy(self, i, j):
        x = j * self.map_resolution + self.map_center_x
        y = i * self.map_resolution + self.map_center_y
        return x, y

    def to_ij(self, x, y):
        i = (y - self.map_center_y) / self.map_resolution
        j = (x - self.map_center_x) / self.map_resolution
        return i, j

    def is_inside(self, i, j):
        return 0 <= i < self.gridmap.shape[0] < self.gridmap.shape[1] and j >= 0

    def raycast_update(self, x0, y0, theta, d):
        # see: https://www.ros.org/reps/rep-0117.html
        # Detections that are too close to the sensor to quantify shall be represented by -Inf. 
        # Erroneous detections shall be represented by quiet (non-signaling) NaNs. 
        # Finally, out of range detections will be represented by +Inf.
        if np.isinf(d) and np.sign(d) == +1:
            d = self.laser_max_dist
        elif np.isinf(d) or np.isnan(d):
            return

        x1 = x0 + d * np.cos(theta)
        y1 = y0 + d * np.sin(theta)
        i0, j0 = self.to_ij(x0, y0)
        i1, j1 = self.to_ij(x1, y1)
        d_cells = d / self.map_resolution
        ip, jp, is_hit = self.bresenham(i0, j0, i1, j1, d_cells)
        if not np.isnan(d) and d != self.laser_max_dist and self.is_inside(int(ip), int(jp)):
            # Hit!
            self.gridmap[int(ip), int(jp)] += self.sensor_model_l_occ - self.sensor_model_l_prior
        return

    # bresenham method is used to plot the lines
    def bresenham(self, i0, j0, i1, j1, d, debug=False):  # i0, j0 (starting point)
        dx = np.absolute(j1 - j0)
        dy = -1 * np.absolute(i1 - i0)
        sx = -1
        if j0 < j1:
            sx = 1
        sy = -1
        if i0 < i1:
            sy = 1
        jp, ip = j0, i0
        err = dx + dy  # error value e_xy
        while True:  # loop
            if (jp == j1 and ip == i1) or (np.sqrt((jp - j0) ** 2 + (ip - i0) ** 2) >= d) or not self.is_inside(ip, jp):
                return ip, jp, False
            elif self.gridmap[int(ip), int(jp)] == 100:
                return ip, jp, True

            if self.is_inside(ip, jp):
                # miss:
                self.gridmap[int(ip), int(jp)] += self.sensor_model_l_free - self.sensor_model_l_prior

            e2 = 2 * err
            if e2 >= dy:  # e_xy+e_x > 0
                err += dy
                jp += sx
            if e2 <= dx:  # e_xy+e_y < 0
                err += dx
                ip += sy

    def update(self, x, y, theta, pts):
        # test by printing robot trajectory
        rr = []
        cc = []
        print(pts)
        for pt in pts:
            i, j = self.to_ij(pt[0], pt[1])
            self.gridmap[int(i), int(j)] = 100
        # temp_rr, temp_cc = circle_perimeter(int(i), int(j), int(self.collision_radius_pix))
        # rr.append(temp_rr)
        # cc.append(temp_cc)
        # traj_rr = np.array(rr)
        # traj_cc = np.array(cc)
        # footprint = np.moveaxis(np.array([traj_rr, traj_cc]), 0, 2)
        # temp_x = np.clip(footprint[..., 1], 0, self.map_size_x - 2).astype(int)
        # temp_y = np.clip(footprint[..., 0], 0, self.map_size_y - 2).astype(int)
        # print(temp_x, temp_y)
        # self.gridmap[temp_x, temp_y] = 100



        return self.gridmap
