import rclpy
import threading
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from trav_seg.local_mapper import LocalMapper


class LocalMapperNode(Node):
    def __init__(self):
        super().__init__("local_mapper_node")

        # TF2 Transform buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for capturing camera frames and transforms
        self.timer = self.create_timer(0.1, self.capture_frame)  # 10Hz

        # Shared Data and Locks
        self.lock = threading.Lock()

        self.local_mapper = LocalMapper()

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
        """Captures a frame and retrieves transform."""
        with self.lock:
            try:
                # Get transform (Replace with real frame IDs)
                d435_to_hopper = self.tf_buffer.lookup_transform("d435", "odom", self.get_clock().now())
                
                # Capture Camera Frame
                self.local_mapper.capture_frame(self.transform_to_matrix_(d435_to_hopper))
            except Exception as e:
                self.get_logger().warn(f"Could not get transform: {e}")
                return

        # Trigger segmentation
        self.segmentation_ready.set()

    def run_segmentation(self):
        """Runs segmentation asynchronously when triggered."""
        while rclpy.ok():
            self.segmentation_ready.wait()
            self.segmentation_ready.clear()

            with self.lock:
                self.local_mapper.segment_frame()

            # Notify occupancy grid update
            self.occupancy_ready.set()

    def update_occupancy_grid(self):
        """Updates occupancy grid asynchronously."""
        while rclpy.ok():
            self.occupancy_ready.wait()
            self.occupancy_ready.clear()

            with self.lock:
                self.local_mapper.update_occupancy_grid()

            # Notify free-space polytope computation
            self.polytope_ready.set()

    def compute_free_space_polytopes(self):
        """Computes free-space polytopes asynchronously."""
        while rclpy.ok():
            self.polytope_ready.wait()
            self.polytope_ready.clear()

            # Simulated polytope computation (Replace with real convex decomposition)
            with self.lock:
                self.local_mapper.compute_free_space_polytopes()
            self.get_logger().info("Computed free-space polytopes.")

    def shutdown(self):
        """Graceful shutdown."""
        self.segmentation_thread.join()
        self.occupancy_thread.join()
        self.polytope_thread.join()


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