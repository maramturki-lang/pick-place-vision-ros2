#!/usr/bin/env python3
"""Detection YOLOv8 en temps reel sur le flux de la camera RGB-D.

S'abonne a /rgbd/image, publie les detections sur /detections et une
image annotee sur /detections/image pour verification visuelle.
"""

import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

CLASS_NAMES = ["cube_rouge", "cube_bleu", "cylindre_vert", "sphere_jaune"]

COLORS = {
    "cube_rouge": (0, 0, 255),
    "cube_bleu": (255, 0, 0),
    "cylindre_vert": (0, 200, 0),
    "sphere_jaune": (0, 200, 255),
}


class Detector(Node):
    def __init__(self):
        super().__init__("detector")

        self.declare_parameter("model_path", "")
        self.declare_parameter("conf", 0.5)
        self.declare_parameter("publish_image", True)

        model_path = self.get_parameter("model_path").value
        if not model_path:
            share = get_package_share_directory("pick_place")
            model_path = os.path.join(share, "models", "yolov8n_pick_place.pt")

        if not os.path.exists(model_path):
            self.get_logger().error(f"Modele introuvable : {model_path}")
            raise SystemExit(1)

        self.conf = self.get_parameter("conf").value
        self.publish_image = self.get_parameter("publish_image").value

        self.get_logger().info(f"Chargement du modele : {model_path}")
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.pub_det = self.create_publisher(
            Detection2DArray, "/detections", 10)
        self.pub_img = self.create_publisher(
            Image, "/detections/image", 10)

        self.create_subscription(Image, "/rgbd/image", self.on_image, 10)

        self.n_frames = 0
        self.get_logger().info("Detecteur pret, en attente d'images")

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame, conf=self.conf, verbose=False)[0]

        det_array = Detection2DArray()
        det_array.header = msg.header

        annotated = frame.copy() if self.publish_image else None

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cls_id = int(box.cls[0])
            score = float(box.conf[0])
            name = CLASS_NAMES[cls_id]

            det = Detection2D()
            det.header = msg.header

            bbox = BoundingBox2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1
            det.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = score
            det.results.append(hyp)

            det_array.detections.append(det)

            if annotated is not None:
                color = COLORS[name]
                cv2.rectangle(annotated, (int(x1), int(y1)),
                              (int(x2), int(y2)), color, 2)
                cv2.putText(annotated, f"{name} {score:.2f}",
                            (int(x1), int(y1) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        self.pub_det.publish(det_array)

        if annotated is not None:
            out = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            out.header = msg.header
            self.pub_img.publish(out)

        self.n_frames += 1
        if self.n_frames % 30 == 0:
            self.get_logger().info(
                f"{self.n_frames} images, {len(det_array.detections)} objets")


def main():
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
