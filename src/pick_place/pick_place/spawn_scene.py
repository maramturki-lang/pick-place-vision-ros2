#!/usr/bin/env python3
"""Genere la scene de tri : 4 bacs fixes et des objets places aleatoirement.

Passe par le service /world/pick_place/create d'Ignition, ce qui permet de
re-randomiser la scene sans redemarrer Gazebo -- utile pour constituer le
jeu de donnees d'entrainement.
"""

import argparse
import math
import os
import random
import subprocess
import tempfile

WORLD = "pick_place"

TABLE_Z = 0.70          # hauteur du plateau
X_MIN, X_MAX = 0.38, 0.72
Y_MIN, Y_MAX = -0.32, 0.32
MIN_DIST = 0.11         # ecart minimal entre deux objets (m)

# nom de classe -> (couleur RGBA, geometrie, demi-hauteur)
CLASSES = {
    "cube_rouge":     ((0.8, 0.1, 0.1, 1),  "box_030",      0.015),
    "cube_bleu":      ((0.1, 0.2, 0.8, 1),  "box_030",      0.015),
    "cylindre_vert":  ((0.1, 0.6, 0.15, 1), "cylinder_025", 0.020),
    "sphere_jaune":   ((0.9, 0.75, 0.1, 1), "sphere_022",   0.022),
}

# (nom, couleur, x, y) -- tous au-dela de la table (|y| > 0.50)
BINS = [
    ("bac_rouge",  (0.8, 0.1, 0.1, 1),  0.30, -0.72),
    ("bac_bleu",   (0.1, 0.2, 0.8, 1),  0.30,  0.72),
    ("bac_vert",   (0.1, 0.6, 0.15, 1), 0.62, -0.72),
    ("bac_jaune",  (0.9, 0.75, 0.1, 1), 0.62,  0.72),
]

_tmp_files = []


def geometry_sdf(kind):
    if kind == "box_030":
        return "<box><size>0.03 0.03 0.03</size></box>"
    if kind == "cylinder_025":
        return "<cylinder><radius>0.0125</radius><length>0.04</length></cylinder>"
    if kind == "sphere_022":
        return "<sphere><radius>0.022</radius></sphere>"
    raise ValueError(kind)


def object_sdf(name, rgba, kind):
    geom = geometry_sdf(kind)
    r, g, b, a = rgba
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>{geom}</geometry>
        <surface>
          <friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction>
          <contact><ode><kp>1e6</kp><kd>100</kd></ode></contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>{geom}</geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def bin_sdf(name, rgba):
    """Bac creux : un fond et quatre parois."""
    r, g, b, a = rgba
    w, d, h, t = 0.20, 0.20, 0.10, 0.008
    mat = (f"<material><ambient>{r} {g} {b} {a}</ambient>"
           f"<diffuse>{r} {g} {b} {a}</diffuse></material>")

    def panel(pname, size, pose):
        return f"""
      <collision name="{pname}_c">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
      </collision>
      <visual name="{pname}_v">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
        {mat}
      </visual>"""

    parts = (
        panel("fond", f"{w} {d} {t}", f"0 0 {t/2} 0 0 0")
        + panel("mur_x1", f"{t} {d} {h}", f"{w/2} 0 {h/2} 0 0 0")
        + panel("mur_x2", f"{t} {d} {h}", f"{-w/2} 0 {h/2} 0 0 0")
        + panel("mur_y1", f"{w} {t} {h}", f"0 {d/2} {h/2} 0 0 0")
        + panel("mur_y2", f"{w} {t} {h}", f"0 {-d/2} {h/2} 0 0 0")
    )

    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <static>true</static>
    <link name="link">{parts}
    </link>
  </model>
</sdf>"""


def spawn(sdf, x, y, z, yaw=0.0, name="model"):
    """Passe par un fichier temporaire.

    Deux raisons : le SDF inline casse le parsing de `ign service`, et le
    fichier doit survivre au retour du service -- Gazebo le lit apres.
    """
    fd, path = tempfile.mkstemp(suffix=".sdf")
    with os.fdopen(fd, "w") as f:
        f.write(sdf)
    _tmp_files.append(path)

    req = (
        f'sdf_filename: "{path}", name: "{name}", '
        f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{z: {math.sin(yaw/2):.6f}, w: {math.cos(yaw/2):.6f}}}}}'
    )
    result = subprocess.run(
        ["ign", "service", "-s", f"/world/{WORLD}/create",
         "--reqtype", "ignition.msgs.EntityFactory",
         "--reptype", "ignition.msgs.Boolean",
         "--timeout", "5000", "--req", req],
        capture_output=True, text=True,
    )
    ok = "true" in result.stdout.lower()
    if not ok:
        print(f"    -> {result.stdout.strip()} {result.stderr.strip()}")
    return ok


def remove(name):
    req = f'name: "{name}", type: MODEL'
    subprocess.run(
        ["ign", "service", "-s", f"/world/{WORLD}/remove",
         "--reqtype", "ignition.msgs.Entity",
         "--reptype", "ignition.msgs.Boolean",
         "--timeout", "2000", "--req", req],
        capture_output=True, text=True,
    )


def scatter(n):
    """Tire n positions espacees d'au moins MIN_DIST."""
    pts = []
    for _ in range(n * 200):
        if len(pts) == n:
            break
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        if all((x - px) ** 2 + (y - py) ** 2 >= MIN_DIST ** 2 for px, py in pts):
            pts.append((x, y))
    return pts


def class_pool(n, names, balanced):
    """Sequence de classes pour les n objets."""
    if not balanced:
        return [random.choice(names) for _ in range(n)]
    pool = []
    while len(pool) < n:
        batch = names[:]
        random.shuffle(batch)
        pool.extend(batch)
    return pool[:n]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--objects", type=int, default=4,
                    help="nombre d'objets a poser sur la table")
    ap.add_argument("--no-bins", action="store_true",
                    help="ne pas creer les bacs (deja presents)")
    ap.add_argument("--clear", action="store_true",
                    help="supprimer les objets existants avant de spawner")
    ap.add_argument("--random-classes", action="store_true",
                    help="tirage independant des classes (defaut : equilibre)")
    args = ap.parse_args()

    if args.clear:
        for i in range(20):
            for cls in CLASSES:
                remove(f"{cls}_{i}")
        print("Objets precedents supprimes.\n")

    if not args.no_bins:
        for name, rgba, x, y in BINS:
            ok = spawn(bin_sdf(name, rgba), x, y, 0.0, name=name)
            print(f"  {name:12s} {'ok' if ok else 'ECHEC'}")

    names = list(CLASSES)
    positions = scatter(args.objects)
    pool = class_pool(len(positions), names, not args.random_classes)

    print(f"\n{len(positions)} objets :")
    for i, (x, y) in enumerate(positions):
        cls = pool[i]
        rgba, kind, half_h = CLASSES[cls]
        z = TABLE_Z + half_h + 0.002
        yaw = random.uniform(0, 3.1416)
        ok = spawn(object_sdf(f"{cls}_{i}", rgba, kind), x, y, z, yaw,
                   name=f"{cls}_{i}")
        print(f"  {cls:15s} ({x:.3f}, {y:.3f})  {'ok' if ok else 'ECHEC'}")

    print(f"\n{len(_tmp_files)} fichiers temporaires dans /tmp "
          f"(nettoyes au redemarrage du systeme).")


if __name__ == "__main__":
    main()
