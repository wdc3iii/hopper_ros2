from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_disc = DeclareLaunchArgument('map_disc', default_value='0.05', description='Discretization of the map.')
    map_dim = DeclareLaunchArgument('map_dim', default_value='200', description='Number of cells per dimension on the map.')
    n_free_spaces = DeclareLaunchArgument('n_free_spaces', default_value='5', description='Number of free spaces to fit to the map.')
    init_free_radius = DeclareLaunchArgument('init_free_radius', default_value='0.75', description='Radius around robot assumed obstacle free.')
    recenter_thresh = DeclareLaunchArgument('recenter_thresh', default_value='0.5', description='Distance at which to recenter the map.')
    publish_pc = DeclareLaunchArgument('publish_pc', default_value='false', description='Whether to publish the point cloud.')
    publish_occ = DeclareLaunchArgument('publish_occ', default_value='true', description='Whether to publish the occupancy grid.')
    publish_frame = DeclareLaunchArgument('publish_frame', default_value='true', description='Whether to publish the rbg frame.')
    viz_poly = DeclareLaunchArgument('viz_poly', default_value='false', description='Whether to visualize the polytopes.')
    local_prompt = DeclareLaunchArgument('local_prompt', default_value='false', description='Whether to visualize the polytopes.')
    buffer_free = DeclareLaunchArgument('buffer_free', default_value='2', description='Grid spaces to buffer free space by.')

    return LaunchDescription([
        # Launch Parameters
        map_disc, map_dim, n_free_spaces, init_free_radius, recenter_thresh,
        publish_pc, publish_occ, publish_frame, viz_poly, local_prompt, buffer_free,
        # t265 Node
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
        # Local Mapper Node
        Node(
            package="local_mapper",
            executable="local_mapper_node",
            name="loc_map_node",
            parameters=[{
                'map_disc': LaunchConfiguration('map_disc'),
                'map_dim': LaunchConfiguration('map_dim'),
                'n_free_spaces': LaunchConfiguration('n_free_spaces'),
                'init_free_radius': LaunchConfiguration('init_free_radius'),
                'recenter_thresh': LaunchConfiguration('recenter_thresh'),
                'publish_pc': LaunchConfiguration('publish_pc'),
                'publish_occ': LaunchConfiguration('publish_occ'),
                'publish_frame': LaunchConfiguration('publish_frame'),
                'viz_poly': LaunchConfiguration('viz_poly'),
                'local_prompt': LaunchConfiguration('local_prompt'),
                'buffer_free': LaunchConfiguration('buffer_free')
            }],
            output="screen"
        ),
    ])
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
#         "hopper_solid2", "d435ish"
#     ],
#     name="static_tf_hopper_solid2_to_d435"
# ),

# Node(
#     package="tf2_ros",
#     executable="static_transform_publisher",
#     arguments=[
#         "0", "0", "0",
#         "1", "0", "0", "0",
#         "d435ish", "d435"
#     ],
#     name="static_tf_d4345ish_to_d435"
# ),
