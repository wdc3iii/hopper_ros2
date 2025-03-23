import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="realsense_ros2",
            executable="rs_t265_node",
            name="t265_node",
            output="screen"
        ),
        # t265 -> Hopper transform
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "-0.1604", "--y", "-0.0954", "--z", "-0.0747",
                "--qx",  "0.7765426942365389", "--qy",  "-0.13685039898430323", "--qz",  "-0.1807761986582881", "--qw",  "0.5878548956369704",
                "--frame-id", "t265_frame", "--child-frame-id", "hopper"
            ],
            name="static_tf_t265_to_hopper"
        ),
        # Hopper -> d435 transform
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "0.101805322541873", "--y", "-0.03790236372609371", "--z", "-0.193623093443654",
                "--qx",  "0.8364829108286959", "--qy",  "-0.14529703161196594", "--qz",  "0.5205893644679247", "--qw",  "-0.0903981531845955",
                "--frame-id", "hopper", "--child-frame-id", "d435"
            ],
            name="static_tf_hopper_to_d435"
        ),
    ])