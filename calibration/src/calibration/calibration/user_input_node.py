import rclpy
from example_interfaces.srv import SetBool
import logging.config
import copilot_daa_app
import os
import argparse


def main():
    logging.config.fileConfig(os.path.join(os.path.dirname(copilot_daa_app.__file__),
                                           "python_logging.conf"))
    logger = logging.getLogger('copilot_daa_perception')
    parser = argparse.ArgumentParser()
    parser.add_argument('--client-service', action="store",
                        help="ROS service to make a call to", required=True, type=str)
    client_service = parser.parse_args().client_service
    rclpy.init(args=None)
    user_input_node = rclpy.create_node('user_input_node')
    client = user_input_node.create_client(SetBool, client_service)
    get_images = True
    req = SetBool.Request()
    while not client.wait_for_service(timeout_sec=1.0):
        logger.info('service not available, waiting again...')
    while get_images:
        user_input = input("Save current frame (y/n): ")
        if user_input.lower() == "y":
            req.data = True
            future = client.call_async(req)
            while rclpy.ok():
                rclpy.spin_once(user_input_node)
                if future.done():
                    try:
                        result = future.result()
                    except Exception as e:
                        logger.info('Service call failed %r' % (e,))
                    else:
                        logger.info(
                            'Saving current frame success: %s' % result.success)
                        if result.success:
                            logger.warning('Err: %s' % result.message)
                    break
        user_input = input("Get more calib_data (y/n): ")
        if user_input.lower() != "y":
            get_images = False
    user_input_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

