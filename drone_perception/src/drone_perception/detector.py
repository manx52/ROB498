import cv2
import rospy
import tf

from drone_perception.camera import Camera


class Detector:
    """
    The Detector class is responsible for detecting objects in images from a camera feed.
    """
    def __init__(self):
        """
        Initializes a new instance of the Detector class.
        """
        self.sim = rospy.get_param("/simulation")
        if not self.sim:
            self.camera = Camera(HORIZONTAL_FOV=2.70772)
        else:
            self.camera = Camera()

        self.br = tf.TransformBroadcaster()

    def circular_mask(self, radius: int):
        """
        Returns a numpy array representing a circular mask of the given radius.

        Args:
        - radius: an integer representing the radius of the circular mask

        Returns:
        - A numpy array representing a circular mask of the given radius.
        """

        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius, radius))
