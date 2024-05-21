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

    robot_ip = LaunchConfiguration('robot_ip')


    package_name='melfa_assista_hardware' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'sim_mode_time': 'false',"robot_ip" : robot_ip}.items()
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
    
    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "-c", "/controller_manager"],
    )

    gripper_action_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller"],
    )


    nodes = [
        #Declare Args
        DeclareLaunchArgument(
            'robot_ip',
            default_value='127.0.0.1',
            description='Robot IP.',
        ),
        rsp,
        delayed_controller_manager,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        gpio_controller_spawner,
        gripper_action_controller_spawner,
    ]

    return LaunchDescription(nodes)
