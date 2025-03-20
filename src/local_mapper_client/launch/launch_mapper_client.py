import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory('local_mapper_client'), 'rviz', 'local_map.rviz')

    return LaunchDescription([
        # Launch Polytope viz node
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
        ),

        # Launch Prompt Node
        Node(
            package='local_mapper_client',
            executable='seg_prompt_client_node',
            name='seg_prompt_client_node',
            output='screen'
        )
    ])