import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('bartbot_nav')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'map', 'map.yaml'),
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params_3d.yaml'),
            description='Full path to Nav2 3D params file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'use_joint_state_publisher',
            default_value='true',
            description='Set false when the arm driver already publishes /joint_states'
        ),

        # LiDAR frame: MID-360 mount pose (same as original bringup)
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

        # robot_state_publisher + joint_state_publisher for the arm chain
        # (publishes base_link → ... → camera_link_d435i dynamically)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('bartbot_description'),
                '/launch/bartbot_description.launch.py'
            ]),
            launch_arguments={
                'gui_rviz': 'false',
                'use_joint_state_publisher': use_joint_state_publisher,
            }.items()
        ),

        Node(
            package='bartbot_nav',
            executable='cmd_vel_to_platform_twist',
            name='cmd_vel_to_platform_twist',
            output='screen',
            parameters=[{
                'in_topic': '/cmd_vel',
                'out_topic': '/platform/twist',
                'linear_scale': 1.0,
                'angular_scale': 1.0,
                'max_linear': 1.0,
                'max_angular': 4.0,
                'max_reverse': 0.4,
                'enable_gate': True,
                'w_gate': 0.10,
                'v_gate': 0.10,
            }]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
            }]
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
            parameters=[params_file]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]
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
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                ]
            }]
        ),
    ])
