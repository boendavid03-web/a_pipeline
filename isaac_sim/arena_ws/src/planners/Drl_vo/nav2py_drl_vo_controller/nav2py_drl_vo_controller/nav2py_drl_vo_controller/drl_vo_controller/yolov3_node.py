#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber

class YoloObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_object_detector')
        self.get_logger().info("Starting YOLO Object Detector Node")

        # Parameters (adjust these as needed or load them from parameter server)
        self.declare_parameter('yolo_cfg', 'yolov3.cfg')
        self.declare_parameter('yolo_weights', 'yolov3.weights')
        self.declare_parameter('yolo_names', 'coco.names')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('input_width', 416)
        self.declare_parameter('input_height', 416)

        # Load parameters
        cfg_path = self.get_parameter('yolo_cfg').value
        weights_path = self.get_parameter('yolo_weights').value
        names_path = self.get_parameter('yolo_names').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value

        # Load class names
        with open(names_path, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        # Load YOLO model using OpenCV DNN
        self.net = cv2.dnn.readNetFromDarknet(cfg_path, weights_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        # Get names of output layers
        layer_names = self.net.getLayerNames()
        self.output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers().flatten()]

        # CvBridge for converting images
        self.bridge = CvBridge()

        # Subscribers using message_filters to synchronize image and depth topics
        image_sub = Subscriber(self, Image, '/zed/zed_node/rgb_raw/image_raw_color')
        depth_sub = Subscriber(self, PointCloud2, '/zed/zed_node/point_cloud/cloud_registered')

        # Adjust queue size and slop as needed
        ats = ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.callback)

        # Publishers for detection image and (optionally) bounding boxes.
        self.detection_pub = self.create_publisher(Image, 'detection_image', 10)
        # self.boxes_pub = self.create_publisher(BoundingBoxes, 'bounding_boxes', 10)

    def callback(self, image_msg, depth_msg):
        # Convert ROS Image message to OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error("CV Bridge Error: " + str(e))
            return

        # Prepare blob for YOLO
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (self.input_width, self.input_height), swapRB=True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        frame_height, frame_width = frame.shape[:2]
        boxes = []
        confidences = []
        class_ids = []

        # Process YOLO outputs
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > self.conf_threshold:
                    center_x = int(detection[0] * frame_width)
                    center_y = int(detection[1] * frame_height)
                    width = int(detection[2] * frame_width)
                    height = int(detection[3] * frame_height)
                    x = int(center_x - width / 2)
                    y = int(center_y - height / 2)
                    boxes.append([x, y, width, height])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply non-maximum suppression to suppress weak overlapping boxes
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)

        # Optionally, use depth information here. For example, you could extract
        # the point at the center of the bounding box from the depth point cloud.
        # This example does not process the PointCloud2 message, but you can use
        # packages such as 'ros2_numpy' to convert it to a NumPy array.

        # Draw bounding boxes on the image
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                label = f"{self.classes[class_ids[i]]}: {confidences[i]:.2f}"
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, label, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish detection image
        detection_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        detection_msg.header = image_msg.header
        self.detection_pub.publish(detection_msg)

        # Here you could also publish bounding box info as a custom message.

    def destroy_node(self):
        self.get_logger().info("Shutting")
