# Pick-and-place guidé par vision (ROS2 + MoveIt2)

Tri automatique d'objets par un bras robotique UR5e, guidé par détection
visuelle. Le robot identifie les objets posés sur une table, estime leur
position 3D, et les dépose dans le bac correspondant à leur classe —
sans aucune position programmée à l'avance.

## Statut

En cours de développement.

- [x] Environnement ROS2 + Gazebo + MoveIt2
- [x] UR5e simulé, contrôleurs actifs, mouvement articulaire validé
- [ ] Contrôle cartésien via MoveIt2
- [ ] Caméra RGB-D et détection YOLOv8
- [ ] Estimation de pose 3D et calibration hand-eye
- [ ] Boucle complète de pick-and-place
- [ ] Tri par classe et mesure des performances

## Stack

ROS2 Humble · Gazebo Fortress · MoveIt2 · YOLOv8 · OpenCV · Python

## Installation

Voir [SETUP.md](SETUP.md).

## Auteur

Maram Turki — élève ingénieure en génie électrique (ENIT)
