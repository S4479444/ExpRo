import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    test_robot_description_share = FindPackageShare(package = "robot_urdf").find("robot_urdf")
    def_model_path = os.path.join(test_robot_description_share, "urdf/robot4.xacro")
    def_world_path = os.path.join(test_robot_description_share, "worlds/aruco_assignment.world")
    rviz_config_path = os.path.join(test_robot_description_share, "config/rviz.rviz")

    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{
            "robot_description": Command(["xacro", LaunchConfiguration("model")])
        }]
    )

    joint_state_publisher_node = Node(
        package = "joint_state_publisher",
        executable = "joint_state_publisher",
        name = "joint_state_publisher"
    )

    joint_camera_rot = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["joint_cam_controller"]
    )

    broad = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["joint_broad"]
    )

    aruco_gen_marker_node = Node(
        package = "ros2_aruco",
        executable = "aruco_generate_marker",
        name = "aruco_generate_marker"
    )

    aruco_node = Node(
        package = "ros2_aruco",
        executable = "aruco_node",
        name = "aruco_node"
    )

    spawn_entity = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments = [
            "-entity",
            "my_test_robot",
            "-topic",
            "/robot_description"
        ],
        output = "screen"
    )

    return LaunchDescription([
        DeclareLaunchArguments(
            name = "model",
            default_value = def_model_path,
            description = "Path to robot URDF file."),
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        joint_camera_rot,
        broad,
        aruco_gen_marker_node,
        aruco_node,
        ExecuteProcess(
            cmd = [
                "gazebo",
                "--verbose",
                def_world_path,
                "-s",
                "libgazebo_ros_factory.so"],
            output = "screen"),
        ExecuteProcess(
            cmd = [
                "rviz2",
                "-d",
                rviz_config_path],
            output = "screen")
    ])