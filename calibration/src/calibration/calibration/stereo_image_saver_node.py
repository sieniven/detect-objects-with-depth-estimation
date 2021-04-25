import copilot_daa_app
import os
import logging.config
import rclpy
import argparse
from copilot_daa_calibration.stereo_image_saver import StereoImageSaver
from copilot_daa_calibration.synced_stereo_image_saver import SyncedStereoImageSaver
import cv2


def main():
    """
    main function to run stereo image saver node (unsync)
    """
    logging.config.fileConfig(os.path.join(os.path.dirname(copilot_daa_app.__file__),
                                           "python_logging.conf"))
    # logger = logging.getLogger('copilot_daa_perception')
    rclpy.init(args=None)
    parser = argparse.ArgumentParser()
    parser.add_argument('--synced', action="store", choices=[True, False], default=False, type=bool)
    parser.add_argument('--img-topic-name', action="store",
                        help="ROS topic to subscribe image from", type=str)
    parser.add_argument('--left-img-topic-name', action="store",
                        help="ROS topic to subscribe the image from", type=str)
    parser.add_argument('--right-img-topic-name', action="store",
                        help="ROS topic to subscribe the image from", type=str)
    parser.add_argument('--save-img-service-name', action="store",
                        help="ROS topic to subscribe the image from", required=True, type=str)
    parser.add_argument('--save-dir', action="store", help="directory to save calib_data",
                        default=None, type=str)
    args = parser.parse_args()
    is_stereo_camera_synced, save_img_service_name, save_dir = \
        args.synced, args.save_img_service_name, args.save_dir

    if is_stereo_camera_synced:
        img_topic_name = args.img_topic_name
        if img_topic_name:
            stereo_image_saver = SyncedStereoImageSaver(
                img_topic_name=img_topic_name,
                save_img_service_name=save_img_service_name,
                dir_to_save=save_dir)
        else:
            parser.error('missing positional argument img-topic-name')
    else:
        left_img_topic_name, right_img_topic_name = args.left_img_topic_name, args.right_img_topic_name
        if left_img_topic_name and right_img_topic_name:
            stereo_image_saver = StereoImageSaver(left_img_topic_name=left_img_topic_name,
                                                  right_img_topic_name=right_img_topic_name,
                                                  save_img_service_name=save_img_service_name,
                                                  dir_to_save=save_dir)
        else:
            parser.error('missing positional argument left-img-topic-name and right-img-topic-name')
    rclpy.spin(stereo_image_saver)
    cv2.destroyAllWindows()
    stereo_image_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
