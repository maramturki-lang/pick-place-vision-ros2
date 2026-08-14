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
