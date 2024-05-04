# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

    #Declare Args
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.122.200',
            description='Robot IP.',
        )
    )

    # Args to Vars
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')

    urdf_command = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("melfa_assita_ros2_hw"),
                    "urdf",
                    "RV-5AS.urdf.xacro",
                ]
            ),
            " use_sim:=", use_sim,
            " use_fake_hardware:=", use_fake_hardware,
            " robot_ip:=", robot_ip,
        ]
    )

    # Hinweis: Hier wird der Befehl direkt als Teil des Parameters verwendet.
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": urdf_command}],
    )

    robot_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("melfa_assita_ros2_hw"),
            "config",
            "melfa_controller.yaml",
        ]
    )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": urdf_command},robot_controllers_file],
        output="both",
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[control_node])


    joint_state_broadcaster_spawner = Node(
        # The joint_state_broadcaster is necessary for the controller_manager to work
        # It publishes the joint states of the robot to the controller_manager
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        # The robot_controller_spawner is used to start the joint_trajectory_controller
    
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller", "--controller-manager", "/controller_manager"],
        )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[robot_controller_spawner],
        )
    )


    nodes = [
        
        robot_state_pub_node,
        delayed_controller_manager,
        joint_state_broadcaster_spawner,
        robot_controller_spawner
    ]

    

    return LaunchDescription(nodes)