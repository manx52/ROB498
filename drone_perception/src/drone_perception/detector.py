import cv2
import rospy
import tf

from drone_perception.camera import Camera


class Detector:
    def __init__(self):
        self.camera = Camera()

        self.br = tf.TransformBroadcaster()


    def circular_mask(self, radius: int):
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius, radius))
