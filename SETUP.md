# Installation de l'environnement

Testé sur Ubuntu 22.04 / ROS2 Humble.

## 1. Paquets ROS2

    sudo apt install -y ros-humble-moveit ros-humble-moveit-visual-tools
    sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers

Note : `ros-humble-moveit-py` n'existe pas en binaire sur Humble
(disponible à partir de Iron). On passe par l'interface d'actions ROS2.

## 2. Choix de la version de Gazebo — point critique

Le système avait initialement **Gazebo Harmonic (8.11)**. Cela ne
fonctionne pas : le paquet `ros-humble-gz-ros2-control` distribué sur
Humble est compilé contre `libignition-gazebo6` (Fortress). Le plugin
ne se charge pas sous Harmonic, le `controller_manager` ne démarre
jamais, et les spawners attendent indéfiniment.

Symptôme :

    Library [libgz_ros2_control-system.so] does not export any plugins.
    Failed to load system plugin: (Reason: No plugins detected in library)

Diagnostic :

    ldd /opt/ros/humble/lib/libign_ros2_control-system.so | grep gazebo
    # libignition-gazebo6.so.6  -> compilé pour Fortress, pas Harmonic

Solution retenue : basculer sur **Fortress**, la version officiellement
supportée par ROS2 Humble.

    sudo apt install -y ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces
    sudo apt install -y ignition-fortress

Attention : cela désinstalle les paquets `ros-humble-ros-gzharmonic`.

## 3. Espace de travail

    mkdir -p ~/pick_place_ws/src && cd ~/pick_place_ws/src
    git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
    git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git
    git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git

    cd ~/pick_place_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    echo "source ~/pick_place_ws/install/setup.bash" >> ~/.bashrc

Si rosdep n'a jamais été initialisé :

    sudo rosdep init
    rosdep update

## 4. Vérification

Terminal 1 — lancer la simulation :

    ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e

Terminal 2 — vérifier les contrôleurs :

    ros2 control list_controllers

Sortie attendue :

    joint_trajectory_controller  ... active
    joint_state_broadcaster      ... active

Test de mouvement (une seule ligne) :

    ros2 topic pub -1 /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"], points: [{positions: [1.0,-1.2,1.0,-1.5,-1.5,0.0], time_from_start: {sec: 3}}]}'

Le bras doit se déplacer en 3 secondes dans Gazebo.
## 5. Contrôle cartésien via MoveIt2

Deux corrections sont nécessaires pour que `Plan & Execute` agisse
réellement sur le robot simulé.

### a) Contrôleur par défaut

Dans `src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/controllers.yaml` :
MoveIt2 cible par défaut `scaled_joint_trajectory_controller`, qui
n'existe que sur le robot physique. En simulation, seul
`joint_trajectory_controller` est lancé.

Inverser les deux champs `default` :

    scaled_joint_trajectory_controller:  default: false
    joint_trajectory_controller:         default: true

### b) Synchronisation d'horloge

Gazebo publie les états articulaires en temps simulé, MoveIt2 raisonne
en temps système. Sans correction, MoveIt2 considère l'état du robot
comme périmé et abandonne l'exécution :

    Didn't receive robot state (joint angles) with recent timestamp
    Failed to validate trajectory: couldn't receive full current joint state within 1s
    Execution completed: ABORTED

Lancer MoveIt2 avec le temps simulé :

    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_sim_time:=true

## 6. Lancement complet

Terminal 1 :

    ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e

Terminal 2 :

    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_sim_time:=true
    ## 7. Ajout de la pince Robotiq 2F-85

### Dépôt

    cd ~/pick_place_ws/src
    git clone -b humble https://github.com/PickNikRobotics/ros2_robotiq_gripper.git
    cd ~/pick_place_ws
    colcon build --packages-select robotiq_description

Seul `robotiq_description` est nécessaire. Les autres paquets du dépôt
pilotent une vraie pince par port série et ne servent pas en simulation.

La macro `robotiq_gripper` accepte un paramètre `sim_ignition:=true` qui
génère automatiquement le bloc `ros2_control` adapté à la simulation.
Une seule articulation est commandable, `robotiq_85_left_knuckle_joint`
(0 = ouvert, 0.7929 = fermé) ; les cinq autres sont des `mimic`.

### Piège 1 — doublon de link dans l'assemblage

La macro `ur_to_robotiq` crée DEUX links : `ur_to_robotiq_link` et un
link portant le nom passé en paramètre `child`. Donner
`child="robotiq_85_base_link"` provoque un doublon, car la macro de la
pince crée déjà ce link.

    Error: link 'robotiq_85_base_link' is not unique.

Solution : utiliser un nom neutre et le passer en parent à la pince.

    <xacro:ur_to_robotiq prefix="" parent="tool0" child="gripper_mount_link"/>
    <xacro:robotiq_gripper ... parent="gripper_mount_link" .../>

Chaîne obtenue : tool0 -> ur_to_robotiq_link -> gripper_mount_link
-> robotiq_85_base_link.

Note : `xacro` ne détecte pas ce type d'erreur, seul le parseur URDF le
fait. Valider avec :

    xacro fichier.urdf.xacro > /tmp/test.urdf && check_urdf /tmp/test.urdf

(nécessite `sudo apt install liburdfdom-tools`)

### Piège 2 — version installée vs sources

Sans `--symlink-install`, un paquet CMake copie ses fichiers dans
`install/`. Modifier le xacro dans `src/` n'a alors aucun effet : le
launch lit la copie figée. Symptôme typique — une erreur déjà corrigée
qui persiste.

Vérifier ce qui est réellement utilisé :

    grep ... install/pick_place_description/share/pick_place_description/urdf/*.xacro

Toujours compiler ce paquet avec `--symlink-install`.

### Piège 3 — meshes model:// introuvables

Gazebo cherche les meshes en `model://robotiq_description/meshes/...`
et échoue : le bras s'affiche sans la pince.

    [Err] Unable to find file with URI [model://robotiq_description/meshes/...]

La racine doit pointer sur le dossier `share`, pas sur `install` :

    export IGN_GAZEBO_RESOURCE_PATH=$HOME/pick_place_ws/install/robotiq_description/share:$HOME/pick_place_ws/install/ur_description/share

Ces chemins sont désormais définis directement dans `sim.launch.py`, il
n'y a donc plus rien à exporter manuellement.

## 8. Lancement du robot complet

    ros2 launch pick_place_description sim.launch.py

Vérification :

    ros2 control list_controllers
    # joint_state_broadcaster      ... active
    # joint_trajectory_controller  ... active
    # gripper_controller           ... active

Test de la pince (fermer puis ouvrir) :

    ros2 topic pub -1 /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: ["robotiq_85_left_knuckle_joint"], points: [{positions: [0.7929], time_from_start: {sec: 2}}]}'

    ros2 topic pub -1 /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: ["robotiq_85_left_knuckle_joint"], points: [{positions: [0.0], time_from_start: {sec: 2}}]}'

## 9. Note sur setuptools

`--symlink-install` échoue sur les paquets Python avec les setuptools
récents :

    error: option --editable not recognized

Solution : `pip3 install "setuptools==58.2.0"` (version attendue par Humble).
## 10. Configuration MoveIt2 pour le robot complet

Générée avec le Setup Assistant à partir d'un URDF figé :

    xacro src/pick_place_description/urdf/ur5e_with_gripper.urdf.xacro \
      simulation_controllers:=/tmp/dummy.yaml \
      > src/pick_place_description/urdf/ur5e_gripper.urdf

    ros2 launch moveit_setup_assistant setup_assistant.launch.py

Réglages retenus :

- Self-Collisions : matrice générée automatiquement (83 paires désactivées)
- Virtual Joint : `fixed_base`, parent `world`, type `fixed`
- Groupe `ur_manipulator` : chaîne `base_link -> tool0`, solveur KDL, RRTConnect
- Groupe `gripper` : le seul joint `robotiq_85_left_knuckle_joint`, sans solveur
- Poses : `home` (bras), `open` et `closed` (pince, 0 et 0.7929 rad)
- End Effector : `gripper_eef`, parent link `tool0`, parent group `ur_manipulator`
- Passive Joints : les cinq articulations `mimic` de la pince
- Contrôleurs renommés en `joint_trajectory_controller` et `gripper_controller`

Ne pas toucher à l'onglet `ros2_control URDF Modifications` : l'URDF
possède déjà son bloc `ros2_control`, généré par `sim_ignition:=true`.

### Piège 4 — moveit_controllers.yaml tronqué

Le fichier généré s'arrête après la liste des joints de la pince, sans
`action_ns` ni `default`. MoveIt2 ne charge alors qu'un contrôleur :

    Returned 1 controllers in list
    Unable to identify any set of controllers that can actuate the
    specified joints: [ robotiq_85_left_knuckle_joint ]

Compléter le bloc `gripper_controller` :

      gripper_controller:
        type: FollowJointTrajectory
        joints:
          - robotiq_85_left_knuckle_joint
        action_ns: follow_joint_trajectory
        default: true

### Piège 5 — use_sim_time ignoré par le launch généré

`generate_move_group_launch` n'expose pas `use_sim_time` comme argument.
Le passer en ligne de commande n'a aucun effet — vérifiable avec :

    ros2 param get /move_group use_sim_time
    # Boolean value is: False

Sans lui, MoveIt2 compare le temps système au temps simulé, juge l'état
du robot périmé et abandonne toute exécution :

    Failed to validate trajectory: couldn't receive full current joint state within 1s
    Execution completed: ABORTED

Solution : instancier le nœud `move_group` directement dans
`launch/move_group.launch.py`.

    from launch import LaunchDescription
    from launch_ros.actions import Node
    from moveit_configs_utils import MoveItConfigsBuilder

    def generate_launch_description():
        moveit_config = MoveItConfigsBuilder(
            "ur5e_with_gripper", package_name="ur5e_gripper_moveit_config"
        ).to_moveit_configs()

        move_group = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": True},
                {"publish_robot_description_semantic": True},
            ],
        )
        return LaunchDescription([move_group])

## 11. Lancement complet du système

Trois terminaux :

    ros2 launch pick_place_description sim.launch.py
    ros2 launch ur5e_gripper_moveit_config move_group.launch.py
    ros2 launch ur5e_gripper_moveit_config moveit_rviz.launch.py

Dans RViz, panneau MotionPlanning : choisir le groupe `ur_manipulator`
ou `gripper`, sélectionner une pose dans Goal State, puis Plan & Execute.
