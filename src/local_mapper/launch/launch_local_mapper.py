from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Another Node (Example: Some Processing Node)
        Node(
            package="realsense_ros2",
            executable="rs_t265_node",
            name="t265_node",
            output="screen"
        ),

        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     arguments=[
        #         "0", "0", "0",
        #         "0.5", "-0.5", "-0.5", "0.5",
        #         "t265_frame", "t265corrected_frame"
        #     ],
        #     name="static_tf_t265_to_t265corrected"
        # ),

        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     arguments=[
        #         "0.0954", "-0.0747", "0.1604",
        #         "0.7765427", "0.5878549", "0.1368504", "0.1807762",
        #         "t265corrected_frame", "hopper_solid"
        #     ],
        #     name="static_tf_t265corrected_to_hopper_solid"
        # ),
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     arguments=[
        #         "0", "0", "0",
        #         "0.5", "0.5", "0.5", "-0.5",
        #         "hopper_solid", "hopper"
        #     ],
        #     name="static_tf_hopper_solid_to_hopper"
        # ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "-0.1604", "-0.0954", "-0.0747",
                "0.7765426942365389", "-0.13685039898430323", "-0.1807761986582881", "0.5878548956369704",
                "t265_frame", "hopper"
            ],
            name="static_tf_t265_to_hopper"
        ),

        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     arguments=[
        #         "0", "0", "0",
        #         "-0.5", "-0.5", "-0.5", "-0.5",
        #         "hopper", "hopper_solid2"
        #     ],
        #     name="static_tf_hopper_to_hopper_solid2"
        # ),

        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     arguments=[
        #         "-0.0379023637260937", "-0.193623093443654", "0.101805322541873",
        #         "-0.5607", "-0.7964", "-0.1854", "0.1305",
        #         "hopper_solid2", "d435"
        #     ],
        #     name="static_tf_hopper_solid2_to_d435"
        # ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.101805322541873", "-0.03790236372609371", "-0.193623093443654",
                "0.0903981531845955", "-0.5205893644679247", "-0.14529703161196594", "0.8364829108286959",
                "hopper", "d435"
            ],
            name="static_tf_hopper_to_d435"
        ),

        # Segmentation Node
        Node(
            package="local_mapper",
            executable="local_mapper_node",
            name="loc_map_node",
            output="screen"
        ),
    ])