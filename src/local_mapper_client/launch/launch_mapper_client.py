import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory('local_mapper_client'), 'rviz', 'local_map.rviz')

    return LaunchDescription([
        # Launch your node
        Node(
            package='local_mapper_client',
            executable='viz_polytopes',
            name='viz_polytopes',
            output='screen'
        ),

        # Launch RViz with the predefined configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])