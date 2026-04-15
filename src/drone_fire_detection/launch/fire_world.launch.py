from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('drone_fire_detection')
    world_file = os.path.join(
        pkg_share,
        'worlds/fire_world.sdf'
    )
    model_path = os.path.join(
        pkg_share,
        'models'
    )
    params_file = os.path.join(
        pkg_share,
        'config/flight_params.yaml'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            additional_env={
                'GZ_SIM_RESOURCE_PATH': model_path,
                'LIBGL_ALWAYS_SOFTWARE': '1',
                'MESA_GL_VERSION_OVERRIDE': '3.3'
            },
            output='screen'
        ),
        Node(
            package='drone_fire_detection',
            executable='autonomous_flight_node.py',
            name='autonomous_flight_node',
            output='screen',
            parameters=[params_file]
        )
    ])
