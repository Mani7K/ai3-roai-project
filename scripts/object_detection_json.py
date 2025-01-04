#!/usr/bin/env python3

import json  # Import JSON for structured data
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String  # For publishing detection results as strings
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os

class TinyYoloClassifier:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('tiny_yolo_classifier', anonymous=True)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, 'yolov4-tiny.weights')
        cfg_path = os.path.join(script_dir, 'yolov4-tiny.cfg')
        coco_names_path = os.path.join(script_dir, 'coco.names')

        # Load Tiny YOLO model and configuration files
        self.net = cv2.dnn.readNet(weights_path, cfg_path)
        self.classes = []
        with open(coco_names_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

        # Create a CvBridge for image conversion
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)

        # Publisher for object detection results
        self.results_pub = rospy.Publisher('/object_detection', String, queue_size=10)

        # Target objects
        self.target_objects = {"chair", "diningtable"}

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Process the image for object detection
            self.classify_objects(cv_image)

        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")

    def classify_objects(self, image):
        # Prepare the image for YOLO
        height, width, channels = image.shape
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)

        # Process the outputs
        detections = []  # To store detection results
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    # Object detected
                    detected_class = self.classes[class_id]
                    if detected_class in self.target_objects:  # Check if target object is detected
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)

                        # Rectangle coordinates
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)

                        # Add detection to the results
                        detections.append({
                            "class": detected_class,
                            "confidence": round(float(confidence), 2),
                            "bbox": [x, y, w, h]
                        })

        # Publish the detection results as JSON if target objects are detected
        if detections:
            json_detections = json.dumps(detections)  # Convert list to JSON string
            rospy.loginfo(f"Publishing detections: {json_detections}")
            self.results_pub.publish(json_detections)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        classifier = TinyYoloClassifier()
        classifier.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Tiny YOLO classifier node terminated.")
    finally:
        cv2.destroyAllWindows()

