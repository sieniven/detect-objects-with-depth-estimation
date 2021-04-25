# ROS2 libraries
import rclpy

# local imported code
from detection.detection_node import ImageDetectorNode


def main():
    """
    executable to run the detector node 
    """
    rclpy.init()
    
    # initialize copilot ImageDetectorNode
    print("Initializing Image Detector Node...")
    
    # to use tensorRT or tensorflow-gpu
    # detector_node = ImageDetectorNode()

    # to use tflite
    detector_node = ImageDetectorNode(flag="tflite")
    
    print("Successful!")
    
    # execute detection callback until shutdown
    rclpy.spin(detector_node)

    detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()