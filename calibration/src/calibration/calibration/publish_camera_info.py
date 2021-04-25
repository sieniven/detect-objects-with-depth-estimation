# basic Python frameworks
import argparse
import yaml
import numpy as np

# ROS2 libraries
import rclpy

# import messages
from sensor_msgs.msg import CameraInfo


def main():
    """
    main function to publish camera info ros2 messages
    """
    rclpy.init(args=None)
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic-to-publish', action="store",
                        help="ROS topic to publish the camera info", required=True, type=str)
    topic_to_publish = parser.parse_args().topic_to_publish
    if "left" in topic_to_publish:
        identifier = "left"
    else:
        identifier = "right"
    node = rclpy.create_node('%s_camera_info_publisher' % identifier)
    publisher = node.create_publisher(CameraInfo, topic_to_publish, 10)

    msg = CameraInfo()

    with open(r'/home/garuda/dev_ws/src/copilot_daa_calibration/data/intrinsic/camera_info.yaml') \
            as config_file:
        camera_info_file_path = yaml.load(config_file, Loader=yaml.FullLoader)["config"]["intrinsic"]
    with open(camera_info_file_path, 'r') as camera_info_file:
        camera_info = yaml.load(camera_info_file, Loader=yaml.FullLoader)
    msg.height = camera_info["image"]["height"]
    msg.width = camera_info["image"]["width"]
    msg.distortion_model = "plumb bob"
    msg.d = camera_info["distortion"][identifier][0]
    msg.k = np.array(camera_info["intrinsic"][identifier]).flatten().tolist()
    msg.r = np.array(camera_info["rectification"][identifier]).flatten().tolist()
    msg.p = np.array(camera_info["projection_mtx"][identifier]).flatten().tolist()
    frame_id = 1

    def timer_callback():
        nonlocal frame_id
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.header.frame_id = str(frame_id)
        publisher.publish(msg)
        frame_id += 1

    timer_period = 0.5
    timer = node.create_timer(timer_period, timer_callback)

    rclpy.spin(node)

    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
