#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('labrob_detector')
import sys
import rospy
import cv2
import numpy as np
import time
import os
from cv_bridge import CvBridge
from ultralytics import YOLO
import rospkg
from cv_bridge import CvBridge
# TODO 1: Import proper ROS image messages
from sensor_msgs.msg import CompressedImage
# TODO 3: Import proper ROS detection messages
from vision_msgs.msg import ObjectHypothesisWithPose, BoundingBox2D, Detection2D
# from std_msgs.msg import Header


class image_converter:
    def __init__(self):
        # TODO 1: Add image subscriber
        self.subscriber = rospy.Subscriber("/camera/raw_image/compressed", CompressedImage, self.callback,  queue_size = 1)
        # TODO 2: Add image publisher
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size = 1)
        # TODO 3: Add 2D detections publisher
        self.msg_pub = rospy.Publisher("trt_detection", Detection2D, queue_size = 1)

        # initialize a list of colors to represent each possible class label
        np.random.seed(42)
        self.COLORS = np.random.randint(0, 255, size=(1000, 3), dtype="uint8")
        self.bridge = CvBridge()


    def callback(self,data):

		# TODO 1: Add code that converts ROS image to OpenCV image. Remove image=np.zeros(1,1,1)
        np_arr = np.frombuffer(data.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        model = YOLO("yolov8n.pt") 
        results = model(image)[0]
        for result in results:
            # extract the bounding box coordinates
            (x0, y0, x1, y1) = result.boxes.xyxy.cpu().numpy().astype(int)[0]
            class_id = int(result.boxes.cls.cpu().numpy()[0])
            confidence =  result.boxes.conf.cpu().numpy()[0]
            # draw a bounding box rectangle and label on the image
            color = [int(c) for c in self.COLORS[class_id]]
            cv2.rectangle(image, (x0, y0), (x1, y1), color, 2)
            text = "{}: {:.4f}".format(result.names[class_id],confidence)
            cv2.putText(image, text, (x0, y0 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            # TODO 3 Fill 2D detections based on information here
            object_hypothesis_with_pose = ObjectHypothesisWithPose()
            # object_hypothesis_with_pose.id = str(text)
            object_hypothesis_with_pose.id = class_id
            object_hypothesis_with_pose.score = float(confidence)

            bounding_box = BoundingBox2D()
            bounding_box.center.x = float((x0 + x1)/2)
            bounding_box.center.y = float((y0 + y1)/2)
            bounding_box.center.theta = 0.0
            
            bounding_box.size_x = float(2*(bounding_box.center.x - x0))
            bounding_box.size_y = float(2*(bounding_box.center.y - y0))

            detection = Detection2D()
            detection.header = data.header
            detection.results.append(object_hypothesis_with_pose)
            detection.bbox = bounding_box
            # TODO 3: Add the code that publishes raw detections
            self.msg_pub.publish(detection)
            # TODO 2: Convert the annotated image to ROS image and publish it.
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
            self.image_pub.publish(msg)


def main(args):
    ic = image_converter()
    rospy.init_node('labrob_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)


