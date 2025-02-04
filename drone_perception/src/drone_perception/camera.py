from functools import cached_property

import rospy
from geometry_msgs.msg import PoseStamped
from rospy import Subscriber
from sensor_msgs.msg import CameraInfo
from tf import TransformListener
from tf.transformations import *

from drone_common.transformation import Transformation
import numpy as np


class Camera:
    """
    This is a reusable class that instantiates an instance of a Camera object that listens to the camera related topics
    related to a robot and has useful functions that use geometry to determine the 3d/2d projection and location of things

    """

    def __init__(self, HORIZONTAL_FOV=1.047, focal_length_x=2.77191356, focal_length_y=2.77191356, resolution_y=480,
                 resolution_x=640):
        """
        Initializes the camera object

        :param robot_name: Name of the robot, to be used in subscribers
        """

        self.pose = Transformation()  #: Pose of the camera
        self.sim = rospy.get_param("/simulation")
        self.camera_info = CameraInfo()  #: Camera info object recieved from the subscriber
        self.resolution_y = resolution_y
        self.resolution_x = resolution_x
        self.horizontalFOV = HORIZONTAL_FOV

        # : Focal length of the camera (meters) distance to the camera plane as
        # projected in 3D
        self.focal_length_x = focal_length_x
        self.focal_length_y = focal_length_y

        self.pose_base_link_straight = Transformation()
        self.pose = Transformation()
        self.camera_info_subscriber = Subscriber(rospy.get_param("/detector_obstacles/camera_topic_mono_info"),
                                                 CameraInfo,
                                                 self.cameraInfoCallback)
        self.tf_listener = TransformListener()
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        trans = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        rot = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

        if trans[2] < 0.02:
            trans[2] = 0.02
        world_to_base_link = Transformation(trans, rot)
        e = world_to_base_link.orientation_euler
        e[1] = 0
        e[2] = 0
        world_to_base_link.orientation_euler = e

        self.pose = world_to_base_link

    def ready(self) -> bool:
        """
        Function to determine when the camera object has recieved the necessary information and is ready to be used

        :return: True if the camera is ready, else False
        """
        return self.pose is not None and self.resolution_x is not None and self.resolution_y is not None and self.camera_info is not None

    def cameraInfoCallback(self, camera_info: CameraInfo):
        """
        Callback function for the camera info subscriber

        :param camera_info: from the camera info topic
        """
        self.camera_info = camera_info
        self.focal_length_x = self.camera_info.K[0]
        self.focal_length_y = self.camera_info.K[4]
        self.resolution_y = float(self.camera_info.height)
        self.resolution_x = float(self.camera_info.width)

    @cached_property
    def verticalFOV(self):
        """
        The vertical field of vision of the camera.
        See `Field of View <https://en.wikipedia.org/wiki/Field_of_view>`_
        """
        return 2 * math.atan(math.tan(self.horizontalFOV * 0.5) * (self.resolution_y / self.resolution_x))

    @cached_property
    def imageSensorHeight(self):
        """
        The height of the image sensor (m)
        """
        return math.tan(self.verticalFOV / 2.0) * 2.0 * self.focal_length_y

    @cached_property
    def imageSensorWidth(self):
        """
        The width of the image sensor (m)
        """
        return math.tan(self.horizontalFOV / 2.0) * 2.0 * self.focal_length_x

    @cached_property
    def pixelHeight(self):
        """
        The height of a pixel in real 3d measurements (m)
        """
        return self.imageSensorHeight / self.resolution_y

    @cached_property
    def pixelWidth(self):
        """
        The wdith of a pixel in real 3d measurements (m)
        """
        return self.imageSensorWidth / self.resolution_x
        pass

    def imageToWorldFrame(self, pixel_x: int, pixel_y: int) -> tuple:
        """
        From image pixel coordinates, get the coordinates of the pixel as if they have been projected ot the camera plane, which is
        positioned at (0,0) in 3D world coordinates
        https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163680c589a_0_0

        :param pixel_x: x pixel of the camera
        :param pixel_y: y pixel of the camera
        :return: 3D position (X, Y) of the pixel in meters
        """
        return (
            (self.resolution_x / 2.0 - (pixel_x + 0.5)) * self.pixelWidth,
            (self.resolution_y / 2.0 - (pixel_y + 0.5)) * self.pixelHeight,
        )

    def worldToImageFrame(self, pos_x: float, pos_y: float) -> tuple:
        """
        Reverse function for  :func:`~drone_perception.Camera.imageToWorldFrame`, takes the 3D world coordinates of the camera plane
        and returns pixels

        :param pos_x: X position of the pixel on the world plane in meters
        :param pos_y: Y position of the pixel on the world plane in meters
        :return: Tuple (x, y) of the pixel coordinates of in the image
        """
        return (
            (self.resolution_x / 2.0 + pos_x / self.pixelWidth) - 0.5,
            (self.resolution_y / 2.0 + pos_y / self.pixelHeight) - 0.5,
        )

    def calculateBallFromBoundingBoxes(self, ball_radius: float = 0.3, bounding_boxes: [float] = []) -> Transformation:
        """
        Reverse function for  :func:`~drone_perception.Camera.calculateBoundingBoxesFromBall`, takes the bounding boxes
        of the ball as seen on the camera and return the 3D position of the ball assuming that the ball is on the ground

        :param ball_radius: The radius of the ball in meters
        :param bounding_boxes: The bounding boxes of the ball on the camera in the format [[x1,y1], [x1,y1]] which are the top left and bottom right of the bounding box respectively
        :return: 3D coordinates of the ball stored in the :class:`Transformation` format
        """

        # bounding boxes [(y1, z1), (y2, z2)]
        r = ball_radius

        y1 = bounding_boxes[0][0]
        z1 = bounding_boxes[0][1]
        y2 = bounding_boxes[1][0]
        z2 = bounding_boxes[1][1]

        # Assuming the ball is a sphere, the bounding box must be a square, averaging the borders
        ym = (y1 + y2) / 2
        zm = (z1 + z2) / 2
        length = z2 - z1
        width = y2 - y1
        y1 = ym - (width / 2)
        z1 = zm - (length / 2)
        y2 = ym + (width / 2)
        z2 = zm + (length / 2)

        # Convert pixels to coordinates
        y1w, z1w = self.imageToWorldFrame(y1, z1)
        y2w, z2w = self.imageToWorldFrame(y2, z2)
        # print("POS: ", y1," : ",y1w," : ",z1," : ", z1w," : ", y2," : ",y2w," : ",z2," : ", z2w)
        y1w = -y1w
        z1w = -z1w
        y2w = -y2w
        z2w = -z2w

        f = self.focal_length_x

        thetay1 = math.atan2(y1w, self.focal_length_x)
        thetay2 = math.atan2(y2w, self.focal_length_x)

        thetayy = (thetay2 - thetay1) / 2
        thetay = thetay1 + thetayy
        dy = r / math.sin(thetayy)

        thetay1_y = math.atan2(y1w, self.focal_length_y)
        thetay2_y = math.atan2(y2w, self.focal_length_y)

        thetayy_y = (thetay2_y - thetay1_y) / 2
        thetay_y = thetay1_y + thetayy_y
        dy_y = r / math.sin(thetayy_y)

        xy = (math.cos(thetay) * dy, math.sin(thetay_y) * dy_y)

        thetaz1 = math.atan2(z1w, f)
        thetaz2 = math.atan2(z2w, f)

        thetazz = (thetaz2 - thetaz1) / 2
        thetaz = thetaz1 + thetazz

        dz = r / math.sin(thetazz)

        xz = (math.cos(thetaz) * dz, math.sin(thetaz) * dz)
        if not self.sim:
            ball_x = xy[0] - abs((xy[1]) / 3.592853)
            ball_y = xy[1] / 10.0
            ball_z = xz[1]
        else:
            ball_x = xy[0]
            ball_y = xy[1]
            ball_z = xz[1]
        # print("orig_x: ", xy[0]," ball_x ", ball_x, " ball_y ", ball_y," ball_z ", ball_z)
        tr = Transformation([ball_x, -ball_y, -ball_z])
        tr_cam = self.pose @ tr
        # print("tr_cam ", tr_cam.position)
        return tr_cam
