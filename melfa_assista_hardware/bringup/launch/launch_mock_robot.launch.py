import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')

    package_name = 'melfa_assista_hardware'  # <--- CHANGE ME

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'sim_mode_time': 'false', 'use_mockup': 'true'}.items()
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    robot_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("melfa_assista_hardware"),
            "config",
            "melfa_controller.yaml",
        ]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    robot_controllers_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "--controller-manager", "/controller_manager", '--ros-args', '--log-level',
                   'INFO'],
        output="screen",
    )

    gripper_action_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gpio2gripper_action_spawner = Node(
        package="melfa_assista_hardware",
        executable="GPIOGripperActionClient",
        output="screen",
    )

    nodes = [
        rsp,
        delayed_controller_manager,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        gripper_action_controller_spawner,
        gpio_controller_spawner,
        gpio2gripper_action_spawner,
    ]

    return LaunchDescription(nodes)
