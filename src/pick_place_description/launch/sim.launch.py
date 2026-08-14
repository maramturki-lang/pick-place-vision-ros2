"""Lance le UR5e equipe de la pince Robotiq 2F-85 dans Gazebo Fortress."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Chemins de ressources pour que Gazebo resolve les meshes model://
    ws_install = os.path.join(os.path.expanduser("~"), "pick_place_ws", "install")
    mesh_paths = ":".join([
        os.path.join(ws_install, "robotiq_description", "share"),
        os.path.join(ws_install, "ur_description", "share"),
    ])
    for var in ("IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH"):
        os.environ[var] = mesh_paths + ":" + os.environ.get(var, "")

    pkg = FindPackageShare("pick_place_description")

    xacro_file = PathJoinSubstitution([pkg, "urdf", "ur5e_with_gripper.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", "controllers.yaml"])
    world_file = PathJoinSubstitution([pkg, "worlds", "pick_place_world.sdf"])

    robot_description = {
        "robot_description": Command([
            "xacro ", xacro_file,
            " simulation_controllers:=", controllers,
        ])
    }

    # Gazebo Fortress, monde de la cellule de tri, demarre en lecture
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": ["-r -v 3 ", world_file],
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "ur5e", "-z", "0.0"],
    )

    # Horloge + flux de la camera RGB-D vers ROS2
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/rgbd/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/rgbd/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/rgbd/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/rgbd/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    jsb = spawner("joint_state_broadcaster")
    arm = spawner("joint_trajectory_controller")
    gripper = spawner("gripper_controller")

    # Chainage : robot depose -> broadcaster -> bras -> pince
    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[gripper])),
    ])
