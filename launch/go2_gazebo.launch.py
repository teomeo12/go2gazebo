#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package paths
    pkg_go2 = get_package_share_directory('go2_description')
    urdf_path = os.path.join(pkg_go2, 'urdf', 'go2_description.urdf')
    world_path = os.path.join(pkg_go2, 'worlds', 'empty.world')

    # Read URDF content for robot_state_publisher
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Run robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Optional: Run joint_state_publisher_gui if you want GUI sliders
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_gui': True}]
        ),
        

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                os.path.join(pkg_go2, 'config', 'controllers.yaml'),
                {'use_sim_time': True}
            ],
            output='screen'
        ),


        # Spawn the robot in Gazebo
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'go2', '-file', urdf_path],
            output='screen'
        ),

        # Load joint state controller
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner',
                 '--controller-manager', '/controller_manager',
                 'joint_state_controller'],
            output='screen'
        ),

        # Load other controllers if needed
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner',
                 'FL_hip_controller'],
            output='screen'
        ),
    ])
