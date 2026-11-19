#!/usr/bin/env python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    diff_drive_description_pkg_path = get_package_share_directory('diff_drive_description')

    xacro_file = os.path.join(
        diff_drive_description_pkg_path,
        'description',
        'models',
        'robot.urdf.xacro'
    )

    # Process Xacro with the same mesh path as Gazebo
    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'mesh_path': diff_drive_description_pkg_path
        }
    ).toxml()

    # Publish robot description and TF
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Publish joint states for visualization
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        joint_state_publisher,
        rsp_node,
        rviz_node
    ])