import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    sim_mode_time = LaunchConfiguration('sim_mode_time')
    robot_ip = LaunchConfiguration('robot_ip')
    use_mockup = LaunchConfiguration('use_mockup')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('melfa_assista_hardware'))
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' sim_mode:=', sim_mode_time, ' robot_ip:=', robot_ip,' use_mockup:=',use_mockup ,])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'sim_mode_time': sim_mode_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode_time',
            default_value='true',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'robot_ip',
            default_value='127.0.0.1',
            description='Robot IP address'),
        DeclareLaunchArgument(
            'use_mockup',
            default_value='false',
            description='Use mockup if true'),

        node_robot_state_publisher
    ])