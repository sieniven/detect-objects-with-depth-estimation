# basi Python frameworks
import os
import time
import math
import numpy as np

# ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

# import ROS2 messages and CvBridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from detector_msg.msg import DispInfo

# import Tensorflow and Yolov4 frameworks
import cv2
from PIL import Image as PILImage
import tensorflow as tf
from tensorflow.python.saved_model import tag_constants
from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession
import detection.core.utils as utils
from detection.core.yolov4 import filter_boxes
from detection.core.config import cfg


class ImageDetectorNode(Node):
    """
    Obstacle and object detection node for DAA capabilities
    """
    def __init__(self, flag="trt"):
        super().__init__("ImageDetectorNode")
        self.frame = None
        self.image = None
        self.output = None
        self.frame_id = 0
        self.bridge = CvBridge()
        self.flag = flag
        
        self.disparity_map = None
        self.point_cloud = None
        self.image3D = None
        self.depths = None

        # initialize parameters
        self.height = 480
        self.width = 640
        self.input_size = 416
        self.disparity_to_depth = np.array([[1.0, 0.0, 0.0, -320.1807384490967],
                                    [0.0, 1.0, 0.0, -247.1542568206787],
                                    [0.0, 0.0, 0.0, 382.4308123701471],
                                    [0.0, 0.0, 0.4089051474345526, -0.0]])
        # cv2.namedWindow("Detected Obstacles Frame", cv2.WINDOW_AUTOSIZE)

        # initialize tflite variables
        self.interpreter = None
        self.input_details = None
        self.output_details = None
        
        # initialize trt / tensorflow variables
        self.saved_model_loaded = None
        self.infer = None

        # initialize tensorflow interactive session
        self.config = ConfigProto()
        self.config.gpu_options.allow_growth = True
        self.session = InteractiveSession(config=self.config)

        # read in all class names from config (from coco dataset)
        self.class_names = utils.read_class_names(cfg.YOLO.CLASSES)
        self.class_names = list(self.class_names.values())

        # initialize model
        if self.flag == 'tflite':
            # initialize model using tflite
            self.interpreter = tf.lite.Interpreter(
                model_path=os.path.join(os.path.expanduser('~'),
                'garuda-copilot-daa/copilot-daa/copilot-daa/src/copilot_daa_detection/checkpoints',
                'yolov4-tiny-416-fp16.tflite'))
            
            # allocate tensors
            self.interpreter.allocate_tensors()
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            print(self.input_details)
            print(self.output_details)
        
        else:
            # initialize model using tensorRT
            # model path for tensorRT framework
            model_path = os.path.join(os.path.expanduser('~'),
                'garuda-copilot-daa/copilot-daa/copilot-daa/src/copilot_daa_detection/checkpoints',
                'yolov4-tiny-trt-fp16-416')

            # initialize model using tensorflow-gpu
            # model path for tensorflow-gpu framework
            # model_path = os.path.join(os.path.expanduser('~'),
            #     'garuda-copilot-daa/copilot-daa/copilot-daa/src/copilot_daa_detection/checkpoints',
            #     'yolov4-tiny-416')

            # initialize saved model (tensorRT or tensorflow-gpu)
            self.saved_model_loaded = tf.saved_model.load(model_path, tags=[tag_constants.SERVING])
            self.infer = self.saved_model_loaded.signatures['serving_default']

        # create subscriber to the topic name "copilot_daa/disparity_info"
        self.disp_sub = self.create_subscription(
            DispInfo, "copilot_daa/disparity_info", self.detect_obstacles, 10)
        # prevent unused variable warning
        self.disp_sub

        # create publisher to the topic name "copilot_daa/detected_image"
        self.detect_image_pub = self.create_publisher(Image, "copilot_daa/detected_image", 10)
        # prevent used variable warning
        self.detect_image_pub


    def detect_obstacles(self, msg):
        """
        callback function to detect objects using Yolov4-tiny
        """
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding="passthrough")
            self.disparity_map = np.array(msg.disparity.data)
            self.disparity_map = np.reshape(self.disparity_map, (msg.disparity.layout.dim[0].size, msg.disparity.layout.dim[1].size))
        except CvBridgeError as e:
            print(e)
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
        self.image = PILImage.fromarray(self.frame)

        # convert RGB images into image data of size 416x416
        image_data = cv2.resize(self.frame, (self.input_size, self.input_size))
        image_data = image_data / 255.
        image_data = image_data[np.newaxis, ...].astype(np.float32)
        start_time = time.time()

        if self.flag == 'tflite':
            # set tensor for model
            self.interpreter.set_tensor(self.input_details[0]['index'], image_data)
            self.interpreter.invoke()
            pred = [self.interpreter.get_tensor(self.output_details[i]['index']) for i in range(len(self.output_details))]

            # get boxes and prediction confidence from model output
            boxes, pred_conf = filter_boxes(pred[0], pred[1], score_threshold=0.25,
                                            input_shape=tf.constant([self.input_size, self.input_size]))
        else:
            batch_data = tf.constant(image_data)
            pred_bbox = self.infer(batch_data)
            for key, value in pred_bbox.items():
                boxes = value[:, :, 0:4]
                pred_conf = value[:, :, 4:]
        
        boxes, scores, classes, valid_detections = tf.image.combined_non_max_suppression(
            boxes=tf.reshape(boxes, (tf.shape(boxes)[0], -1, 1, 4)),
            scores=tf.reshape(pred_conf, (tf.shape(pred_conf)[0], -1, tf.shape(pred_conf)[-1])),
            max_output_size_per_class=50,
            max_total_size=50,
            iou_threshold=0.45,
            score_threshold=0.25)

        # get depth map
        self.get_point_cloud()

        # cv2.imshow("3D Frame", self.image3D)
        
        # get depth estimation of detected objects
        depths, skip_list = self.get_depth_estimation(boxes.numpy(), classes.numpy(), valid_detections.numpy())

        # draw bounding boxes on detected obstacles and their depth estimations
        pred_bbox = [boxes.numpy(), scores.numpy(), classes.numpy(), valid_detections.numpy(), depths]
        self.image, self.depths = utils.draw_bbox(self.frame, pred_bbox, skip_list)
        self.output = np.asarray(self.image)
        
        # get detection runtime
        fps = 1.0 / (time.time() - start_time)
        print("Fps: %.2f" % fps)

        # show detected obstavle frames on cv2 gui
        result = cv2.cvtColor(self.output, cv2.COLOR_RGB2BGR)
        # cv2.imshow("Detected Obstacles Frame", result)

        out_img_msg = self.bridge.cv2_to_imgmsg(result, encoding="bgr8")
        self.detect_image_pub.publish(out_img_msg)
        self.frame_id += 1

        cv2.waitKey(1)

    def get_point_cloud(self):
        # get 3D point cloud coordinates
        self.disparity_map = np.float32(self.disparity_map)
        self.disparity_map = self.disparity_map / 16
        self.point_cloud = cv2.reprojectImageTo3D(self.disparity_map, self.disparity_to_depth)

        # visualize point cloud
        # self.image3D = cv2.normalize(self.point_cloud, 0, 255, cv2.NORM_MINMAX)
        # self.image3D = np.uint8(self.image3D)


    # row going downwards (top to bottom), col going sideways (left to right)    
    def get_point_depth(self, row, col):
        point = self.point_cloud[row][col]

        # # using absolute distance
        # point = point / (100 * 2.3) # use 2.3cm for the unit measurement (space between each chessboard corner during calibration)
        # distance = math.sqrt((point[0])**2 + (point[1])**2 + (point[2])**2)

        # use z-axis distance
        distance = point[2] / (20)

        return distance


    def get_depth_estimation(self, out_boxes, out_classes, num_boxes, classes=utils.read_class_names(cfg.YOLO.CLASSES)):
        # disparity and depth maps will be "blacked" in this area from pixel values 0-78 due to stereo camera usage. 
        # thus, depth estimations will not be possible in this region.
        limit = 79  
        depths = []
        skip_list = []
        num_classes = len(classes)
        for i in range(num_boxes[0]):
            if int(out_classes[0][i]) < 0 or int(out_classes[0][i]) > num_classes: continue
            coor = out_boxes[0][i]
            coor[0] = int(coor[0] * self.height)
            coor[2] = int(coor[2] * self.height)
            coor[1] = int(coor[1] * self.width)
            coor[3] = int(coor[3] * self.width)
            
            obj_box_height = int((coor[2] - coor[0]) / 4)
            obj_box_width = int((coor[3] - coor[1]) / 4)

            coor[0] = coor[0] + obj_box_height
            coor[2] = coor[2] - obj_box_height
            coor[1] = coor[1] + obj_box_width
            coor[3] = coor[3] - obj_box_width

            if coor[1] < limit:
                if coor[3] < limit:
                    skip_list.append(i)
                    continue
                else:
                    coor[1] = limit
            
            depth_list = []
            total_depth = 0
            num = 0
            for row in range(int(coor[0]), int(coor[2] + 1)):
                for col in range(int(coor[1]), int(coor[3] + 1)):
                    total_depth += self.get_point_depth(row, col)
                    num += 1

            depth = total_depth / num
            depths.append(depth)

        return depths, skip_list