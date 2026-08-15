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
