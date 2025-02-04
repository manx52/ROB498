from unittest import TestCase

import numpy as np
from drone_common.transformation import Transformation


class TestCommon(TestCase):
    def test_transformation(self):
        t = Transformation(quaternion=[0, 0, 1, 0]) @ Transformation(position=[1, 0, 0])
        assert np.all(t.quaternion == [0, 0, 1, 0])
        assert np.all(t.position == [-1, 0, 0])

    def test_vicon_transformation(self):
        # q = t.quaternion
        # [roll, pitch, yaw] = Transformation.get_euler_from_quaternion([q.w, q.x, q.y, q.z])

        vicon_quaternion = Transformation.get_quaternion_from_euler([0, 0, 1.5708])
        vicon_position = [1,1,0]
        vicon_transform = Transformation(vicon_position, vicon_quaternion)

        transformed_position = vicon_transform.rotation_matrix @ vicon_transform.position
        transformed_position2 = vicon_transform.rotation_matrix.T @ transformed_position
        #assert np.all(transformed_position == np.array([1, -1.00000367, 0.99999633]))
        print(transformed_position)
        print(transformed_position2)
        #print(vicon_transform.rotation_matrix)

    # def test_calculate_bounding_boxes_from_ball(self):
    #     rospy.init_node("test")
    #
    #     for cam_angle in [0, 0.1, -0.1]:
    #
    #         for cam_position in [[0, 0, 0], [0, 0, 0.1], [0, 0, -0.1]]:
    #             p = Transformation(cam_position, euler=[cam_angle, 0, 0])
    #             c = Camera("robot1")
    #             c.pose = p
    #             ci = CameraInfo()
    #             ci.height = 360
    #             ci.width = 240
    #             c.camera_info = ci
    #
    #             positions = [[0.5, 0, 0.1], [0.5, 0, 0], [0.5, 0, 0.1]]
    #             for position in positions:
    #                 ball_pose = Transformation(position)
    #                 ball_radius = 0.07
    #
    #                 bounding_boxes = c.calculateBoundingBoxesFromBall(ball_pose, ball_radius)
    #                 # [[135.87634651355825, 75.87634651355823], [224.12365348644175, 164.12365348644175]]
    #                 position = c.calculateBallFromBoundingBoxes(ball_radius, bounding_boxes)
    #
    #                 self.assertAlmostEqual(position.position[0], ball_pose.position[0], delta=0.001)
    #                 self.assertAlmostEqual(position.position[1], ball_pose.position[1], delta=0.001)
    #                 self.assertAlmostEqual(position.position[2], ball_pose.position[2], delta=0.001)
