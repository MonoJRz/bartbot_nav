import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bartbot_nav')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'map', 'map.yaml'),
            description='Full path to map yaml file'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Full path to Nav2 params file'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_lidar',
            output='screen',
            arguments=[
                '--x', '0.15',
                '--y', '-0.08',
                '--z', '0.15',
                '--qx', '0',
                '--qy', '0.21644',
                '--qz', '0',
                '--qw', '0.976296',
                '--frame-id', 'base_link',
                '--child-frame-id', 'lidar_base_link',
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_lidar_2',
            output='screen',
            arguments=[
                '--x', '0.15',
                '--y', '-0.08',
                '--z', '0.25',
                '--qx', '0',
                '--qy', '0',
                '--qz', '0',
                '--qw', '1',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser_projection_frame',
            ]
        ),

        # Nav2 raw command filter:
        # /cmd_vel_nav -> /cmd_vel_nav_smooth
        Node(
            package='bartbot_nav',
            executable='nav2_cmd_vel_filter',
            name='nav2_cmd_vel_filter',
            output='screen',
            parameters=[{
                'in_topic': '/cmd_vel_nav',
                'out_topic': '/cmd_vel_nav_smooth',

                'publish_rate_hz': 30.0,

                'linear_deadband': 0.005,
                'angular_deadband': 0.03,

                'max_linear_x': 0.10,
                'max_reverse_x': 0.05,
                'max_angular_z': 0.80,

                'min_forward_x': 0.04,
                'min_reverse_x': 0.03,

                'forward_w_limit': 0.18,
                'turn_in_place_w': 1.00,
                'turn_forward_scale': 0.45,

                'max_v_rate': 0.15,
                'max_w_rate': 1.00,

                'cmd_timeout': 0.50,
            }]
        ),

        # Custom Twist mux:
        # joystick /cmd_vel has higher priority than Nav2 /cmd_vel_nav_smooth
        Node(
            package='bartbot_nav',
            executable='cmd_vel_mux_simple',
            name='cmd_vel_mux_simple',
            output='screen',
            parameters=[{
                'joy_topic': '/cmd_vel',
                'nav_topic': '/cmd_vel_nav_smooth',
                'out_topic': '/cmd_vel_muxed',

                'publish_rate_hz': 30.0,

                'joy_timeout': 0.50,
                'nav_timeout': 0.50,

                'joy_linear_deadband': 0.005,
                'joy_angular_deadband': 0.005,
            }]
        ),

        # Simple platform bridge:
        # /cmd_vel_muxed -> /platform/twist
        Node(
            package='bartbot_nav',
            executable='cmd_vel_to_platform_twist',
            name='cmd_vel_to_platform_twist',
            output='screen',
            parameters=[{
                'in_topic': '/cmd_vel_muxed',
                'out_topic': '/platform/twist',

                'linear_scale': 6.0,
                'angular_scale': 5.0,
                'angular_sign': 1.0,

                'max_linear': 0.6,
                'max_angular': 4.0,
                'max_reverse': 0.4,

                'linear_deadband': 0.01,
                'angular_deadband': 0.03,

                'min_forward_cmd': 0.20,
                'min_reverse_cmd': 0.30,
                'min_turn_cmd': 3.60,

                'publish_rate_hz': 30.0,
                'cmd_timeout': 0.50,
            }]
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/dlio/odom_node/pointcloud/deskewed'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'laser_projection_frame',
                'transform_tolerance': 0.1,
                'min_height': -0.1,
                'max_height': 0.1,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.25,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file
            }]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('cmd_vel', '/cmd_vel_nav'),
            ]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('cmd_vel', '/cmd_vel_nav'),
            ]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        ),
    ])