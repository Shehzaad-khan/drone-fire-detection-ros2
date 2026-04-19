import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drone_fire_detection')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_file   = os.path.join(pkg_share, 'worlds', 'fire_world.sdf')
    model_sdf    = os.path.join(pkg_share, 'models', 'drone', 'model.sdf')
    params_file  = os.path.join(pkg_share, 'config', 'flight_params.yaml')
    rviz_config  = os.path.join(pkg_share, 'config', 'fire_detection.rviz')

    return LaunchDescription([

        # ── Gazebo 11 (gzserver + gzclient) ──────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world_file,
                'verbose': 'false',
            }.items(),
        ),

        # ── Spawn the drone model into Gazebo ────────────────────────────
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_drone',
            arguments=[
                '-file', model_sdf,
                '-entity', 'drone',
                '-x', '0', '-y', '0', '-z', '2.5',
            ],
            output='screen',
        ),

        # ── Static TF: map → odom (identity) ────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),

        # ── Mode mux — bridges auto/teleop → /cmd_vel ─────────────────────
        # Switch at runtime:
        #   ros2 topic pub --once /set_mode std_msgs/msg/String "{data: 'manual'}"
        #   ros2 topic pub --once /set_mode std_msgs/msg/String "{data: 'autonomous'}"
        Node(
            package='drone_fire_detection',
            executable='mode_mux_node.py',
            name='mode_mux',
            output='screen',
        ),

        # ── Autonomous flight node (publishes to /cmd_vel_auto) ───────────
        Node(
            package='drone_fire_detection',
            executable='autonomous_flight_node.py',
            name='autonomous_flight_node',
            output='screen',
            parameters=[params_file],
            remappings=[('/cmd_vel', '/cmd_vel_auto')],
        ),

        # ── Fire detection node ───────────────────────────────────────────
        Node(
            package='drone_fire_detection',
            executable='fire_detection_node.py',
            name='fire_detection_node',
            output='screen',
            parameters=[{'display_window': False}],
        ),

        # ── Alert + RViz marker node ──────────────────────────────────────
        Node(
            package='drone_fire_detection',
            executable='ros_alert_node.py',
            name='fire_alert_node',
            output='screen',
        ),

        # ── RViz2 ─────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
