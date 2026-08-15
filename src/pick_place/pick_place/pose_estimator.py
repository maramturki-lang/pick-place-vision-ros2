#!/usr/bin/env python3
"""Estimation de la pose 3D des objets detectes.

Combine les detections 2D de YOLOv8 avec l'image de profondeur pour
remonter aux coordonnees monde exploitables par le bras.

Chaine : pixels -> repere camera (deprojection) -> repere monde.
"""

import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Pose
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

# Intrinseques (voir /rgbd/camera_info)
FX = FY = 554.3827128226441
CX, CY = 320.0, 240.0

# Pose de la camera dans le monde
CAM_X, CAM_Y, CAM_Z = 0.55, 0.0, 1.40

TABLE_Z = 0.70

# Demi-hauteur de chaque classe : la profondeur mesuree correspond au
# SOMMET de l'objet, pas a son centre. Sans cette correction, la pince
# viserait trop haut et raterait la saisie.
HALF_HEIGHT = {
    "cube_rouge": 0.015,
    "cube_bleu": 0.015,
    "cylindre_vert": 0.020,
    "sphere_jaune": 0.022,
}


class PoseEstimator(Node):
    def __init__(self):
        super().__init__("pose_estimator")

        self.declare_parameter("use_depth", True)
        self.declare_parameter("verbose", True)
        self.use_depth = self.get_parameter("use_depth").value
        self.verbose = self.get_parameter("verbose").value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(PoseArray, "/object_poses", 10)

        # Les deux flux n'arrivent pas exactement en meme temps :
        # on les synchronise sur leurs timestamps.
        sub_det = Subscriber(self, Detection2DArray, "/detections")
        sub_depth = Subscriber(self, Image, "/rgbd/depth_image")
        sync = ApproximateTimeSynchronizer(
            [sub_det, sub_depth], queue_size=10, slop=0.15)
        sync.registerCallback(self.on_data)

        self.get_logger().info(
            f"Estimateur de pose pret (profondeur : {self.use_depth})")

    def read_depth(self, depth_img, u, v, window=5):
        """Mediane sur une petite fenetre : robuste au bruit et aux trous."""
        h, w = depth_img.shape[:2]
        u, v = int(round(u)), int(round(v))
        r = window // 2
        patch = depth_img[max(0, v-r):min(h, v+r+1),
                          max(0, u-r):min(w, u+r+1)]
        valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 3.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def deproject(self, u, v, depth):
        """Pixels + profondeur -> coordonnees monde."""
        # repere camera (convention optique)
        xc = (u - CX) * depth / FX
        yc = (v - CY) * depth / FY

        # camera a 1.40 m regardant vers le bas, inverse de la projection
        # utilisee pour generer le dataset
        x = CAM_X - yc
        y = CAM_Y - xc
        z = CAM_Z - depth
        return x, y, z

    def on_data(self, det_msg, depth_msg):
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

        poses = PoseArray()
        poses.header = det_msg.header
        poses.header.frame_id = "world"

        lines = []
        for det in det_msg.detections:
            if not det.results:
                continue
            name = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y

            half_h = HALF_HEIGHT.get(name, 0.015)

            if self.use_depth:
                d = self.read_depth(depth_img, u, v)
                if d is None:
                    if self.verbose:
                        self.get_logger().warn(
                            f"{name} : profondeur invalide, ignore")
                    continue
                x, y, z_top = self.deproject(u, v, d)
                # la profondeur vise le sommet -> descendre au centre
                z = z_top - half_h
                src = "depth"
            else:
                # repli : objet suppose pose sur la table
                z = TABLE_Z + half_h
                d = CAM_Z - (z + half_h)
                x, y, _ = self.deproject(u, v, d)
                src = "plan"

            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation.w = 1.0
            poses.poses.append(p)

            lines.append(f"  {name:15s} conf={score:.2f}  "
                         f"({x:.3f}, {y:.3f}, {z:.3f})  [{src}]")

        self.pub.publish(poses)

        if self.verbose and lines:
            self.get_logger().info(
                f"{len(lines)} objets localises\n" + "\n".join(lines),
                throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = PoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
