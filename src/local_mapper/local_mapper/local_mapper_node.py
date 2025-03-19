import threading
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs_py import point_cloud2 
from sensor_msgs.msg import PointCloud2, Image
from local_mapper_interfaces.msg import PromptClickData
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from local_mapper_interfaces.msg import PolytopeArray, Polytope

from rclpy.action import ActionServer
from local_mapper_interfaces.action import SegPrompt
from scipy.spatial import Delaunay

from trav_seg.local_mapper import LocalMapper
from scipy.spatial.transform import Rotation


class LocalMapperNode(Node):
    def __init__(self):
        super().__init__("local_mapper_node")

        # TF2 Transform buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for capturing camera frames and transforms
        self.declare_parameter('capture_period', 0.2)
        self.timer = self.create_timer(self.get_parameter('capture_period').value, self.capture_frame)
        self.pub_polytopes = self.create_publisher(PolytopeArray, 'free_polytopes', 10)

        # Shared Data and Locks
        self.pos = None
        self.lock = threading.Lock()
        self.bridge = CvBridge()

        # Declare mapping parameters with default values
        self.declare_parameter('map_disc', 0.05)
        self.declare_parameter('map_dim', 200)
        self.declare_parameter('n_free_spaces', 5)
        self.declare_parameter('init_free_radius', 0.25)
        self.declare_parameter('recenter_thresh', 0.5)
        self.declare_parameter('publish_pc', False)
        self.declare_parameter('publish_occ', False)
        self.declare_parameter('viz_poly', False)
        self.declare_parameter('local_prompt', False)

        self.seg_prompt_server = ActionServer(
            self,
            SegPrompt,
            'seg_prompt',
            self.seg_prompt_server
        )
        self.pub_frame = self.create_publisher(Image, 'd435_image', 1)
        self.latest_clicks = []
        self.sub_click = self.create_subscription(
            PromptClickData,
            'seg_prompt_clicks',
            self.seg_prompt_click_callback,
            10
        )
        self.prompt_completed = False

        self.local_mapper = LocalMapper(
            self.get_parameter('map_disc').value,
            self.get_parameter('map_dim').value,
            self.get_parameter('n_free_spaces').value,
            self.get_parameter('init_free_radius').value,
            self.get_parameter('recenter_thresh').value,
            local_prompt=self.get_parameter('local_prompt').value
        )
        self.pc_pub = self.create_publisher(PointCloud2, 'd435_pointcloud', 1)
        self.occ_pub = self.create_publisher(OccupancyGrid, 'occ_grid', 1)
        self.poly_viz_pub = self.create_publisher(MarkerArray, 'viz_poly', 1)

        # Flags to trigger tasks asynchronously
        self.segmentation_ready = threading.Event()
        self.occupancy_ready = threading.Event()
        self.polytope_ready = threading.Event()

        # Start Threads for Async Execution
        self.segmentation_thread = threading.Thread(target=self.run_segmentation, daemon=True)
        self.occupancy_thread = threading.Thread(target=self.update_occupancy_grid, daemon=True)
        self.polytope_thread = threading.Thread(target=self.compute_free_space_polytopes, daemon=True)

        self.segmentation_thread.start()
        self.occupancy_thread.start()
        self.polytope_thread.start()

    def capture_frame(self):
        self.get_logger().info("Capture Frame callback triggered.")
        """Captures a frame and retrieves transform."""
        with self.lock:
            try:
                # Get transform (Replace with real frame IDs)
                hopper_to_odom = self.tf_buffer.lookup_transform("hopper", "odom", rclpy.time.Time())
                self.pos = np.array([
                    hopper_to_odom.transform.translation.x,
                    hopper_to_odom.transform.translation.y,
                    hopper_to_odom.transform.translation.z
                ])
                d435_to_odom = self.tf_buffer.lookup_transform("odom", "d435", rclpy.time.Time())

                # Extract translation
                p = np.array([
                    d435_to_odom.transform.translation.x,
                    d435_to_odom.transform.translation.y,
                    d435_to_odom.transform.translation.z
                ])

                # Extract rotation matrix from quaternion
                q = [
                    d435_to_odom.transform.rotation.x,
                    d435_to_odom.transform.rotation.y,
                    d435_to_odom.transform.rotation.z,
                    d435_to_odom.transform.rotation.w
                ]
                R = Rotation.from_quat(q).as_matrix()
                
                # Capture Camera Frame
                self.local_mapper.capture_frame(p, R)

            except Exception as e:
                self.get_logger().warn(f"Could not get transform: {e}")
                return

        # Trigger segmentation
        if (self.local_mapper.trav_seg.local_prompt and self.local_mapper.trav_seg.prompt_completed) or self.prompt_completed:
            self.get_logger().info("Image Captured: Ready to segment.")
            self.segmentation_ready.set()
        else:
            self.get_logger().info("Image Captured: Waiting for segmentation Prompt.")
        if self.get_parameter('publish_pc').value:
            self.publish_pc_()

    def run_segmentation(self):
        """Runs segmentation asynchronously when triggered."""
        while rclpy.ok():
            self.segmentation_ready.wait()
            self.segmentation_ready.clear()

            with self.lock:
                self.local_mapper.segment_frame()

            # Notify occupancy grid update
            self.get_logger().info("Frame Segmented: Ready to update grid.")
            self.occupancy_ready.set()

            if self.get_parameter('publish_occ').value:
                self.publish_occ_grid_()

    def update_occupancy_grid(self):
        """Updates occupancy grid asynchronously."""
        while rclpy.ok():
            self.occupancy_ready.wait()
            self.occupancy_ready.clear()

            with self.lock:
                self.local_mapper.update_occ_grid(self.pos)

            # Notify free-space polytope computation
            self.get_logger().info("Grid Updated: Ready to fit polytopes.")
            self.polytope_ready.set()

    def compute_free_space_polytopes(self):
        """Computes free-space polytopes asynchronously."""
        while rclpy.ok():
            self.polytope_ready.wait()
            self.polytope_ready.clear()

            # Simulated polytope computation (Replace with real convex decomposition)
            with self.lock:
                self.local_mapper.fit_free_space()
            self.get_logger().info("Computed free-space polytopes.")

            # TODO: Publish polytopes
            polytopes = []
            for polytope in self.local_mapper.polytopes:
                poly = Polytope()
                poly.vertices = polytope['vertices'].flatten().tolist()
                poly.normals = polytope['A'].flatten().tolist()
                poly.b = polytope['b'].flatten().tolist()
                polytopes.append(poly)
            poly_arr = PolytopeArray()
            poly_arr.polytopes = polytopes
            self.pub_polytopes.publish(poly_arr)

            if self.get_parameter('viz_poly').value:
                self.viz_polytopes_()

    def shutdown(self):
        """Graceful shutdown."""
        self.local_mapper.shutdown()
        self.segmentation_thread.join()
        self.occupancy_thread.join()
        self.polytope_thread.join()

    def publish_pc_(self):
        # Create header with current timestamp
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "odom"

        # Convert NumPy array to PointCloud2 message
        cloud_msg = point_cloud2.create_cloud_xyz32(header, self.local_mapper.trav_seg.xyz)
        self.pc_pub.publish(cloud_msg)

    def publish_occ_grid_(self):
        grid = OccupancyGrid()

        # Header
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "odom"  # Set the reference frame

        # Grid metadata
        grid.info = MapMetaData()
        grid.info.resolution = self.local_mapper.disc
        grid.info.width = self.local_mapper.occ_grid.shape[0]
        grid.info.height = self.local_mapper.occ_grid.shape[1]
        grid.info.origin.position.x = self.local_mapper.map_origin[0]
        grid.info.origin.position.y = self.local_mapper.map_origin[1]
        grid.info.origin.position.z = 0.0

        # Flatten to 1D array for ROS message
        grid.data = self.local_mapper.occ_grid.flatten().tolist()

        # Publish
        self.get_logger().info("Publishing occupancy grid.")
        self.occ_pub.publish(grid)
    
    def viz_polytopes_(self):
        """Publishes multiple filled polygon markers as a MarkerArray"""
        marker_array = MarkerArray()

        for i, poly in enumerate(self.local_mapper.polytopes):
            marker = self.create_filled_polygon_marker_(poly['vertices'], marker_id=i)
            marker_array.markers.append(marker)

        self.poly_viz_pub.publish(marker_array)
        self.get_logger().info("Published filled polygons to RViz")

    def create_filled_polygon_marker_(self, vertices, marker_id, color=(0.0, 1.0, 0.0, 0.25)):  
        """Creates an RViz marker for a filled polygon using TRIANGLE_LIST"""
        marker = Marker()
        marker.header.frame_id = "odom"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "polygons"
        marker.id = marker_id
        marker.type = Marker.TRIANGLE_LIST  # Filled polygon
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0  

        # Set color (RGBA)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        # Triangulate the polygon using Delaunay
        if len(vertices) >= 3:
            tri = Delaunay(vertices)
            for simplex in tri.simplices:
                for i in simplex:
                    p = Point(x=float(vertices[i][0]), y=float(vertices[i][1]), z=0.0)
                    marker.points.append(p)

        return marker
    
    # Server
    async def seg_prompt_server(self, goal_handle):
        """Handles segmentation process interactively."""
        self.current_goal_handle = goal_handle

        # Capture and publish image
        while self.local_mapper.trav_seg.seg_frame is None:
            self.get_logger().info("Spinning in action server.")
            rclpy.spin_once(self, timeout_sec=0.1) 
        
        image_msg = self.bridge.cv2_to_imgmsg(self.local_mapper.trav_seg.seg_frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "d435"

        self.pub_frame.publish(image_msg)
        self.prompt_idx = self.local_mapper.trav_seg.frame_idx
        self.get_logger().info("Published image for segmentation.")

        feedback_msg = SegPrompt.Feedback()

        exit_prompt = False
        while not exit_prompt:
            self.get_logger().info("Waiting for exit prompt command.")
            # Check for cancellation
            if not self.current_goal_handle or self.current_goal_handle.is_cancel_requested:
                self.get_logger().info("Segmentation canceled.")
                goal_handle.canceled()
                return SegPrompt.Result()
            
            # Wait for click data
            while not self.latest_clicks:
                self.get_logger().info("Spinning in action server, waiting for clicks.")
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            # Apply prompts
            for group, label, x, y, ex_prompt_ in self.latest_clicks:
                if ex_prompt_:
                    exit_prompt = ex_prompt_
                    break
                self.local_mapper.trav_seg.add_prompt(group, label, x, y, self.prompt_idx)
            self.latest_clicks = []

            # Send feedback (updated segmentation mask)
            feedback_msg.mask.height, feedback_msg.mask.width, _ = self.local_mapper.trav_seg.all_mask.shape
            feedback_msg.mask.data = self.local_mapper.trav_seg.all_mask.flatten().tolist()
            goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info("Segmentation confirmed, sending final mask.")
        goal_handle.succeed()
        result_msg = SegPrompt.Result()
        result_msg.final_mask.height, result_msg.final_mask.width, _ = self.local_mapper.trav_seg.all_mask.shape
        result_msg.final_mask.data = self.local_mapper.trav_seg.all_mask.flatten().tolist()
        self.get_logger().info("Action returning")
        self.prompt_completed = True
        return result_msg

    def seg_prompt_click_callback(self, msg):
        self.latest_clicks.append((msg.group, msg.label, msg.x, msg.y, msg.exit_prompt))
        if msg.exit_prompt:
            self.get_logger().info(f"Received exit prompt command.")
        else:
            self.get_logger().info(f"Received click at ({msg.x}, {msg.y}) in group {msg.group} with label {msg.label}.")

def main():
    rclpy.init()
    node = LocalMapperNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()  # Multi-threaded event loop
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()