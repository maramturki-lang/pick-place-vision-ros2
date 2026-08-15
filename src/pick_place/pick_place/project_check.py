#!/usr/bin/env python3
"""Verifie la projection monde -> pixels en dessinant les boites sur l'image.

Lit les poses reelles des objets depuis Gazebo, les projette avec les
intrinsegues de la camera, et sauve une image annotee dans /tmp.
"""

import re
import subprocess

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

WORLD = "pick_place"

# Intrinseques (voir /rgbd/camera_info)
FX = FY = 554.3827128226441
CX, CY = 320.0, 240.0
W, H = 640, 480

# Pose de la camera dans le monde
CAM_X, CAM_Y, CAM_Z = 0.55, 0.0, 1.40

# rayon apparent (m) par prefixe de classe
RADIUS = {
    "cube_rouge": 0.021,      # demi-diagonale d'un cube de 3 cm
    "cube_bleu": 0.021,
    "cylindre_vert": 0.0125,
    "sphere_jaune": 0.022,
}

COLORS = {
    "cube_rouge": (0, 0, 255),
    "cube_bleu": (255, 0, 0),
    "cylindre_vert": (0, 200, 0),
    "sphere_jaune": (0, 200, 255),
}


def gazebo_poses():
    """Retourne {nom: (x, y, z)} pour les objets de la scene."""
    out = subprocess.run(
        ["ign", "model", "--list"], capture_output=True, text=True
    ).stdout

    names = [
        n.strip("- ").strip()
        for n in out.splitlines()
        if any(c in n for c in RADIUS)
    ]

    poses = {}
    for name in names:
        info = subprocess.run(
            ["ign", "model", "-m", name, "-p"],
            capture_output=True, text=True
        ).stdout
        nums = re.findall(r"\[([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)\]", info)
        if nums:
            x, y, z = (float(v) for v in nums[0])
            poses[name] = (x, y, z)
    return poses


def project(x, y, z):
    """Monde -> pixels. Camera a 1.40 m regardant vers -Z du monde."""
    depth = CAM_Z - z              # distance le long de l'axe optique
    dx = x - CAM_X                 # ecart dans le plan de la table
    dy = y - CAM_Y
    # convention optique : u suit -dy, v suit -dx (a verifier visuellement)
    u = CX - dy * FX / depth
    v = CY - dx * FY / depth
    return u, v, depth


class Checker(Node):
    def __init__(self):
        super().__init__("project_check")
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", value=True)]
        )
        self.bridge = CvBridge()
        self.done = False
        self.create_subscription(Image, "/rgbd/image", self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        poses = gazebo_poses()

        print(f"\n{len(poses)} objets trouves :")
        for name, (x, y, z) in poses.items():
            cls = next(c for c in RADIUS if name.startswith(c))
            u, v, depth = project(x, y, z)
            r_px = RADIUS[cls] * FX / depth

            print(f"  {name:16s} monde=({x:.3f},{y:.3f},{z:.3f}) "
                  f"-> pixel=({u:.0f},{v:.0f}) r={r_px:.0f}px")

            p1 = (int(u - r_px), int(v - r_px))
            p2 = (int(u + r_px), int(v + r_px))
            cv2.rectangle(img, p1, p2, COLORS[cls], 2)
            cv2.putText(img, cls, (p1[0], p1[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLORS[cls], 1)

        cv2.imwrite("/tmp/projection.png", img)
        print("\nImage annotee : /tmp/projection.png")
        self.done = True


def main():
    rclpy.init()
    n = Checker()
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=1.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
