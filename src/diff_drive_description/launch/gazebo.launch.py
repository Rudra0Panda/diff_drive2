#!/usr/bin/env python3


import os
from os.path import join
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction ,IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # warehouse = get_package_share_directory("rs_warehouse")
    diff_drive_description_pkg_path = get_package_share_directory('diff_drive_description')

    spawn_x, spawn_y, spawn_z, spawn_yaw = '0.0', '0.0', '0.3862', '0'

    # floor_top_z = '0.3312'

    xacro_file = os.path.join(diff_drive_description_pkg_path, "description", "models","robot.urdf.xacro")

    robot_description = xacro.process_file(
            xacro_file,
            mappings={'mesh_path': diff_drive_description_pkg_path}
        ).toxml()

    # world_file = LaunchConfiguration(
    # "world_file",
    # default=join(warehouse,"worlds", "warehouse_world.sdf")
    # )

    gz_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    ),
    launch_arguments={
        'gz_args':['-r -v 4 ', 'empty.sdf'] 
    }.items()
)
    

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }]
    )

    create_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-name', 'diff_drive',
            '-topic', '/robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw
        ],
    )
    spawn_robot = TimerAction(period=2.0, actions=[create_robot])


    bridge_config= os.path.join(diff_drive_description_pkg_path,
                                'config',
                                'bridge.yaml'
    )

    bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='diff_drive_bridge',
    output='screen',
    parameters=[
        {
            'config_file': bridge_config,
            'use_sim_time': True,
        }
    ],
)


    bridge_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=create_robot, on_exit=[bridge_node])
    )

    return LaunchDescription([
        gz_sim,
        rsp_node,
        # world_to_odom,
        spawn_robot,
        # bridge_after_spawn,
        bridge_node
    ])
