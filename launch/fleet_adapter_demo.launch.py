#!/usr/bin/python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    nav2_tb3_launch = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'tb3_simulation_launch.py'
    ])
    
    rmf_common_launch = PathJoinSubstitution([
        FindPackageShare('fleet_adapter_demo'),
        'launch',
        'include',
        'rmf_common.launch.py'
    ])

    fleet_adapter_launch = PathJoinSubstitution([
        FindPackageShare('fleet_adapter'),
        'launch',
        'include',
        'adapter.launch.py'
    ])

    fleet_config_path = os.path.join(
        get_package_share_directory('fleet_adapter_demo'),
        'config',
        'fleet_config.yaml'
    )

    building_graph_path = os.path.join(
        get_package_share_directory('fleet_adapter_demo'),
        'maps',
        'turtlebot3_world.building.yaml'
    )

    nav_graph_path = os.path.join(
        get_package_share_directory('fleet_adapter_demo'),
        'maps',
        'nav_graphs',
        '0.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            nav2_tb3_launch,
            launch_arguments={
                'use_sim_time': 'true',
                'headless': 'True',
                'slam': 'True'
            }.items()
        )
        ,
        IncludeLaunchDescription(
            rmf_common_launch,
            launch_arguments={
                'using_sim_time': 'false',
                'headless': 'false',
                'initial_map': 'L1',
                'config_file': building_graph_path
            }.items()
        ),
        IncludeLaunchDescription(
            fleet_adapter_launch,
            launch_arguments={
                'fleet_config_path': fleet_config_path,
                'nav_graph_path': nav_graph_path
            }.items()
        )
    ])
