from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from example_interfaces.srv import SetBool
from typing import Optional
import numpy as np
import cv2
import datetime
import os
import logging


logger = logging.getLogger('copilot_daa_perception')


class SyncedStereoImageSaver(Node):
    """A class to represent the synchronised stereo image save
    """
    def __init__(self, img_topic_name, save_img_service_name, dir_to_save=None):
        # type: (str, str, Optional[str]) -> None
        """Initialise the class.
        Args:
            img_topic_name: ROS topic of the stereo image pair to subscribe the from
            save_img_service_name: ROS service name to save stereo images
            dir_to_save: absolute directory to saved captured stereo images
        """
        super().__init__('synced_stereo_image_saver')
        if dir_to_save is None:
            dir_to_save = os.getcwd()
        self.__image_subscriber = \
            self.create_subscription(CompressedImage, img_topic_name, self.callback, 10)
        self.__image_saver = self.create_service(
            SetBool, save_img_service_name, self.save_image)
        self.__left_image = None
        self.__right_image = None
        self.__dir_to_save = dir_to_save

    def callback(self, msg):
        # type: (CompressedImage) -> None
        """A function to receive subscribed image
        Args:
            msg: ROS message that contains the compressed image
        """
        fused_image = SyncedStereoImageSaver.decode_image(msg.data)
        self.__left_image, self.__right_image = np.split(fused_image, 2, axis=1)
        SyncedStereoImageSaver.show_image("left", self.__left_image)
        SyncedStereoImageSaver.show_image("right", self.__right_image)

    @staticmethod
    def decode_image(encoded_image):
        # type: (np.ndarray) -> np.ndarray
        """A function to decode image
        Args:
            encoded_image: an encoded image that was passed over ROS2 topic
        Returns:
            decoded_image: the decoded image
        """
        return cv2.imdecode(np.array(np.array(encoded_image)), cv2.IMREAD_COLOR)

    def save_image(self, req, resp):
        # type: (SetBool.req, SetBool.resp) -> SetBool.resp
        """ A function to save the stereo image pair
        Args:
            req: ROS service request that indicates whether to save the stereo image pair or not
            resp: ROS service response that indicates whether images are successfully saved or not
        Returns:
            resp: ROS service response that indicates whether images are successfully saed or not
        """
        if self.__left_image is not None and self.__right_image is not None:
            if not os.path.exists(os.path.join(self.__dir_to_save, "left")):
                os.makedirs(os.path.join(self.__dir_to_save, "left"))
            if not os.path.exists(os.path.join(self.__dir_to_save, "right")):
                os.makedirs(os.path.join(self.__dir_to_save, "right"))
            timestamp = (datetime.datetime.now()).strftime("%m/%d/%Y, %H:%M:%S")
            filename = (timestamp + ".jpg").replace("/", "-").replace(",", "")
            cv2.imwrite(os.path.join(self.__dir_to_save, "left", filename), self.__left_image)
            cv2.imwrite(os.path.join(self.__dir_to_save, "right", filename), self.__right_image)
            resp.success = True
        else:
            logger.warning("No image to save")
            resp.success = False
            resp.message = "No image to save"
        return resp

    @staticmethod
    def show_image(window_name, image):
        # type: (str, np.ndarray) -> None
        """A function to display image on OpenCV GUI."""
        cv2.imshow(window_name, image)
        cv2.waitKey(1)
