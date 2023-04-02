# !/usr/bin/env python3

from drone.utils import *

A = np.array((0, 0))
B = np.array((1, 0))
q1 = calc_quaternion(A, B, Quaternion(x=0, y=0, z=0, w=1), True)

A = np.array((1, 0))
B = np.array((1, 1))
q2 = calc_quaternion(A, B, Quaternion(x=0, y=0, z=0, w=1), True)
A = np.array((1, 1))
B = np.array((0, 1))
q3 = calc_quaternion(A, B, Quaternion(x=0, y=0, z=0, w=1), True)
A = np.array((0, 1))
B = np.array((0, 0))
q4 = calc_quaternion(A, B, Quaternion(x=0, y=0, z=0, w=1), True)
