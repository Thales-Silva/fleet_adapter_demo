from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rate', default_value='10.0'),
        DeclareLaunchArgument('map_name', default_value='B1'),
        DeclareLaunchArgument(
            'viz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rmf_visualization_schedule'),
                'config',
                'rmf.rviz'
            ])
        ),
        DeclareLaunchArgument('display_names', default_value='true'),
        DeclareLaunchArgument('websocket_port', default_value='8006'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('path_width', default_value='0.2'),
        DeclareLaunchArgument('lane_width', default_value='0.5'),
        DeclareLaunchArgument('waypoint_scale', default_value='1.3'),
        DeclareLaunchArgument('text_scale', default_value='0.7'),
        DeclareLaunchArgument('lane_transparency', default_value='0.6'),
        DeclareLaunchArgument('wait_secs', default_value='10'),
        DeclareLaunchArgument('retained_history_count', default_value='50'),
        DeclareLaunchArgument('fleet_state_nose_scale', default_value='0.5'),
        DeclareLaunchArgument('rmf_frame_id', default_value='map'),
    ]

    # Nós do grupo principal
    visualizer_nodes = GroupAction([
        Node(
            package='rmf_visualization_schedule',
            executable='schedule_visualizer_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rate': LaunchConfiguration('rate'),
                'path_width': LaunchConfiguration('path_width'),
                'initial_map_name': LaunchConfiguration('map_name'),
                'wait_secs': LaunchConfiguration('wait_secs'),
                'port': LaunchConfiguration('websocket_port'),
                'retained_history_count': LaunchConfiguration('retained_history_count'),
            }]
        ),
        Node(
            package='rmf_visualization_fleet_states',
            executable='fleetstates_visualizer_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'fleet_state_nose_scale': LaunchConfiguration('fleet_state_nose_scale'),
                'tinyRobot_radius': 0.3,
                'deliveryRobot_radius': 0.6,
                'cleanerBotA_radius': 1.0,
                'cleanerBotE_radius': 1.0,
                'caddy_radius': 1.5,
            }]
        ),
        Node(
            package='rmf_visualization_building_systems',
            executable='rmf_visualization_building_systems',
            arguments=['-m', LaunchConfiguration('map_name')],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),
        Node(
            package='rmf_visualization_navgraphs',
            executable='navgraph_visualizer_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'initial_map_name': LaunchConfiguration('map_name'),
                'lane_width': LaunchConfiguration('lane_width'),
                'waypoint_scale': LaunchConfiguration('waypoint_scale'),
                'text_scale': LaunchConfiguration('text_scale'),
                'lane_transparency': LaunchConfiguration('lane_transparency'),
            }]
        ),
        Node(
            package='rmf_visualization_floorplans',
            executable='floorplan_visualizer_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'initial_map_name': LaunchConfiguration('map_name'),
            }]
        ),
        Node(
            package='rmf_visualization_obstacles',
            executable='obstacle_visualizer_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'initial_map_name': LaunchConfiguration('map_name'),
                'global_fixed_frame': LaunchConfiguration('rmf_frame_id'),
            }]
        ),
    ])

    # Lançamento do RViz (se headless == false)
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', LaunchConfiguration('viz_config_file')],
        output='both',
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

    return LaunchDescription(launch_args + [visualizer_nodes, rviz_node])
