#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_go2 = get_package_share_directory('go2gazebo')
    urdf_path = os.path.join(pkg_go2, 'urdf', 'go2_description.urdf')
    world_path = os.path.join(pkg_go2, 'worlds', 'empty.world')
    controllers_path = os.path.join(pkg_go2, 'config', 'controllers.yaml')

    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'go2', '-topic', 'robot_description'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers_path, {'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['FL_hip_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
    ])
