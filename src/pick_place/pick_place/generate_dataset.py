#!/usr/bin/env python3
"""Genere un jeu de donnees YOLO annote automatiquement.

Boucle : nettoie la scene, spawn des objets aleatoires, attend la
stabilisation physique, capture l'image, et calcule les bounding boxes
a partir des poses REELLES lues dans Gazebo.

Attention : Gazebo se degrade au fil des scenes (les services repondent
de plus en plus lentement, jusqu'au timeout). Generer par lots de ~50 en
redemarrant la simulation entre chaque, avec --start pour la numerotation.
"""

import argparse
import math
import os
import random
import re
import subprocess
import tempfile
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

WORLD = "pick_place"

FX = FY = 554.3827128226441
CX, CY = 320.0, 240.0
IMG_W, IMG_H = 640, 480

CAM_X, CAM_Y, CAM_Z = 0.55, 0.0, 1.40

TABLE_Z = 0.70
X_MIN, X_MAX = 0.38, 0.72
Y_MIN, Y_MAX = -0.32, 0.32
MIN_DIST = 0.09

# ordre = identifiant de classe YOLO (0, 1, 2, 3)
CLASS_NAMES = ["cube_rouge", "cube_bleu", "cylindre_vert", "sphere_jaune"]

# (rgba, forme, demi-hauteur, rayon apparent)
SPECS = {
    "cube_rouge":    ((0.8, 0.1, 0.1, 1),  "box",      0.015, 0.021),
    "cube_bleu":     ((0.1, 0.2, 0.8, 1),  "box",      0.015, 0.021),
    "cylindre_vert": ((0.1, 0.6, 0.15, 1), "cylinder", 0.020, 0.0125),
    "sphere_jaune":  ((0.9, 0.75, 0.1, 1), "sphere",   0.022, 0.022),
}

_tmp = []


def geom(shape):
    return {
        "box": "<box><size>0.03 0.03 0.03</size></box>",
        "cylinder": "<cylinder><radius>0.0125</radius><length>0.04</length></cylinder>",
        "sphere": "<sphere><radius>0.022</radius></sphere>",
    }[shape]


def object_sdf(name, rgba, shape):
    g = geom(shape)
    r, gr, b, a = rgba
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <link name="link">
      <inertial><mass>0.05</mass>
        <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry>{g}</geometry>
        <surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry>{g}</geometry>
        <material><ambient>{r} {gr} {b} {a}</ambient>
                  <diffuse>{r} {gr} {b} {a}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def ign_service(service, reqtype, req, timeout=8000):
    return subprocess.run(
        ["ign", "service", "-s", f"/world/{WORLD}/{service}",
         "--reqtype", f"ignition.msgs.{reqtype}",
         "--reptype", "ignition.msgs.Boolean",
         "--timeout", str(timeout), "--req", req],
        capture_output=True, text=True,
    ).stdout


def spawn(sdf, name, x, y, z, yaw):
    fd, path = tempfile.mkstemp(suffix=".sdf")
    with os.fdopen(fd, "w") as f:
        f.write(sdf)
    _tmp.append(path)
    req = (f'sdf_filename: "{path}", name: "{name}", '
           f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{z: {math.sin(yaw/2):.6f}, w: {math.cos(yaw/2):.6f}}}}}')
    return "true" in ign_service("create", "EntityFactory", req).lower()


def remove(name):
    ign_service("remove", "Entity", f'name: "{name}", type: MODEL', 4000)


def list_objects():
    """Noms des objets de tri actuellement dans la scene."""
    out = subprocess.run(["ign", "model", "--list"],
                         capture_output=True, text=True).stdout
    return [n.strip("- ").strip() for n in out.splitlines()
            if any(n.strip("- ").strip().startswith(c + "_")
                   for c in CLASS_NAMES)]


def remove_all_objects():
    for name in list_objects():
        remove(name)


def gazebo_poses():
    """Lit les poses REELLES apres stabilisation physique.

    Indispensable : entre le spawn et la capture, les objets tombent et
    roulent. Annoter les positions demandees produit des boites decalees,
    et le modele apprend sur une verite terrain fausse.
    """
    poses = {}
    for name in list_objects():
        info = subprocess.run(["ign", "model", "-m", name, "-p"],
                              capture_output=True, text=True).stdout
        nums = re.findall(r"\[([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)\]", info)
        if nums:
            poses[name] = tuple(float(v) for v in nums[0])
    return poses


def project(x, y, z):
    depth = CAM_Z - z
    u = CX - (y - CAM_Y) * FX / depth
    v = CY - (x - CAM_X) * FY / depth
    return u, v, depth


def scatter(n):
    pts = []
    for _ in range(n * 300):
        if len(pts) == n:
            break
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        if all((x - px) ** 2 + (y - py) ** 2 >= MIN_DIST ** 2 for px, py in pts):
            pts.append((x, y))
    return pts


class Capture(Node):
    """Fournit la derniere image recue sur demande."""

    def __init__(self):
        super().__init__("dataset_capture")
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.bridge = CvBridge()
        self.frame = None
        self.create_subscription(Image, "/rgbd/image", self._cb, 10)

    def _cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def grab(self, timeout=5.0):
        """Vide le buffer puis attend une image fraiche."""
        self.frame = None
        t0 = time.time()
        while self.frame is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.frame


def make_sample(node, idx, out_img, out_lbl, n_objects):
    """Genere une scene, capture, et ecrit image + annotations.

    Retourne False si l'echantillon doit etre rejete.
    """
    remove_all_objects()
    if list_objects():
        print(f"  [{idx}] nettoyage incomplet, rejete")
        return False

    positions = scatter(n_objects)
    n_spawned = 0
    for i, (x, y) in enumerate(positions):
        cls = random.choice(CLASS_NAMES)
        rgba, shape, half_h, _ = SPECS[cls]
        z = TABLE_Z + half_h + 0.002
        yaw = random.uniform(0, math.pi)
        if spawn(object_sdf(f"{cls}_{i}", rgba, shape),
                 f"{cls}_{i}", x, y, z, yaw):
            n_spawned += 1

    if n_spawned < len(positions):
        print(f"  [{idx}] {n_spawned}/{len(positions)} spawns, rejete")
        return False

    time.sleep(1.2)                      # stabilisation physique
    frame = node.grab()
    if frame is None:
        print(f"  [{idx}] pas d'image, rejete")
        return False

    real = gazebo_poses()                # positions APRES stabilisation

    # Si des poses manquent, l'image serait annotee comme vide et
    # polluerait le dataset avec des faux negatifs.
    if len(real) < n_spawned:
        print(f"  [{idx}] {len(real)}/{n_spawned} poses lues, rejete")
        return False

    lines = []
    for name, (x, y, z) in real.items():
        cls = next(c for c in CLASS_NAMES if name.startswith(c + "_"))
        radius = SPECS[cls][3]
        u, v, depth = project(x, y, z)
        r_px = radius * FX / depth
        xc, yc = u / IMG_W, v / IMG_H
        bw, bh = 2 * r_px / IMG_W, 2 * r_px / IMG_H
        if not (0 < xc < 1 and 0 < yc < 1):
            continue                     # objet tombe hors champ
        lines.append(
            f"{CLASS_NAMES.index(cls)} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}")

    if not lines:
        print(f"  [{idx}] aucun objet dans le champ, rejete")
        return False

    cv2.imwrite(os.path.join(out_img, f"{idx:04d}.png"), frame)
    with open(os.path.join(out_lbl, f"{idx:04d}.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--samples", type=int, default=50,
                    help="nombre d'echantillons de ce lot")
    ap.add_argument("--start", type=int, default=0,
                    help="index de depart (pour generer par lots)")
    ap.add_argument("--val-every", type=int, default=5,
                    help="1 echantillon sur N va dans le split validation")
    ap.add_argument("--min-objects", type=int, default=2)
    ap.add_argument("--max-objects", type=int, default=6)
    ap.add_argument("-o", "--out",
                    default=os.path.expanduser("~/pick_place_dataset"))
    args = ap.parse_args()

    root = args.out
    for split in ("train", "val"):
        for kind in ("images", "labels"):
            os.makedirs(os.path.join(root, kind, split), exist_ok=True)

    rclpy.init()
    node = Capture()

    print(f"Lot : {args.samples} echantillons a partir de "
          f"l'index {args.start}\n")
    t0 = time.time()
    ok = 0
    for k in range(args.samples):
        i = args.start + k
        # repartition deterministe : 1 sur 5 en validation
        split = "val" if i % args.val_every == 0 else "train"
        n_obj = random.randint(args.min_objects, args.max_objects)
        if make_sample(node, i,
                       os.path.join(root, "images", split),
                       os.path.join(root, "labels", split),
                       n_obj):
            ok += 1
        if (k + 1) % 10 == 0:
            el = time.time() - t0
            print(f"  {k+1}/{args.samples}  ({el/60:.1f} min, "
                  f"{ok} valides)")

    # data.yaml regenere a chaque lot
    with open(os.path.join(root, "data.yaml"), "w") as f:
        f.write(f"path: {root}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n\n")
        f.write(f"nc: {len(CLASS_NAMES)}\n")
        f.write("names:\n")
        for i, name in enumerate(CLASS_NAMES):
            f.write(f"  {i}: {name}\n")

    for p in _tmp:
        try:
            os.unlink(p)
        except OSError:
            pass

    print(f"\n{ok}/{args.samples} echantillons valides en "
          f"{(time.time()-t0)/60:.1f} min")


if __name__ == "__main__":
    main()
