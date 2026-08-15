# Résultats

## Semaine 1

- Environnement validé : ROS2 Humble + Gazebo Fortress + MoveIt2
- Contrôleurs actifs : joint_state_broadcaster, joint_trajectory_controller
- Mouvement articulaire commandé validé dans Gazebo

## Mesures à collecter

- [ ] Taux de réussite de saisie (sur 50 essais)
- [ ] Taux de bonne classification
- [ ] Temps de cycle moyen
- [ ] Erreur de positionnement après calibration hand-eye

## Détection YOLOv8n

Entraînement sur 99 images synthétiques annotées automatiquement
(80 train / 19 val), 69 époques, early stopping à l'époque 49.

| Métrique   | Valeur |
|------------|--------|
| Precision  | 0.990  |
| Recall     | 1.000  |
| mAP50      | 0.995  |
| mAP50-95   | 0.807  |

Inférence : 4,5 ms par image (T4).

## Estimation de pose 3D

Déprojection des détections 2D avec l'image de profondeur, puis
transformation vers le repère monde. Correction de demi-hauteur par
classe (la profondeur mesure le sommet de l'objet, pas son centre).

Erreur mesurée contre la vérité terrain Gazebo sur 4 objets :

| Objet          | Erreur 3D |
|----------------|-----------|
| cube_rouge     | 2.0 mm    |
| cube_bleu      | 6.3 mm    |
| sphere_jaune   | 8.1 mm    |
| cylindre_vert  | 8.8 mm    |

Erreur moyenne : 6.3 mm — largement dans la tolérance de la pince
Robotiq 2F-85 (ouverture 85 mm pour des objets de 30 mm).
