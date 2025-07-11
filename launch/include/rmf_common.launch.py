#!/usr/bin/python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('server_uri', default_value=''),
        DeclareLaunchArgument('initial_map', default_value='L1'),
        DeclareLaunchArgument('bidding_time_window', default_value='2.0'),
        DeclareLaunchArgument('use_unique_hex_string_with_task_id', default_value='true'),
        DeclareLaunchArgument('use_reservation_node', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('viz_config_file', default_value=''),
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_schedule',
            name='rmf_traffic_schedule_primary',
            output='both',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_blockade',
            name='rmf_traffic_blockade',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        Node(
            package='rmf_building_map_tools',
            executable='building_map_server',
            arguments=[
                LaunchConfiguration('config_file')
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        Node(
            package='rmf_fleet_adapter',
            executable='door_supervisor',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        Node(
            package='rmf_fleet_adapter',
            executable='lift_supervisor',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        Node(
            package='rmf_task_ros2',
            executable='rmf_task_dispatcher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'bidding_time_window': LaunchConfiguration('bidding_time_window'),
                'use_unique_hex_string_with_task_id': LaunchConfiguration('use_unique_hex_string_with_task_id'),
                'server_uri': LaunchConfiguration('server_uri')
            }]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('fleet_adapter_demo'),
                             'launch',
                             'include',
                             'visualization.launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_name': LaunchConfiguration('initial_map'),
                'viz_config_file': LaunchConfiguration('viz_config_file'),
                'headless': LaunchConfiguration('headless')
            }.items()
        )
    ])