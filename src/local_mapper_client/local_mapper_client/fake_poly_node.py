import rclpy
import numpy as np
from rclpy.node import Node
from local_mapper_interfaces.msg import PolytopeArray, Polytope


class FakePolyNode(Node):
    def __init__(self):
        super().__init__('fake_poly_node')
        self.publisher_ = self.create_publisher(PolytopeArray, 'free_polytopes', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0

        self.msg = PolytopeArray()

        # Polytope 1  -> -0.5 <= x <= 0.5, -0.5 <= y <= 2
        p1 = Polytope()
        p1.vertices = np.array([
            [-0.5, -0.5],
            [0.5, -0.5],
            [0.5, 2],
            [-0.5, 2]
        ]).flatten().tolist()
        p1.normals = np.array([
            [1., 0.],
            [-1., 0.],
            [0., 1.],
            [0., -1.],
        ]).flatten().tolist()
        p1.b = np.array([
            [0.5],
            [0.5],
            [2],
            [0.5],
        ]).flatten().tolist()

        # Polytope 2  -> -2 <= x <= 0.5, 1.5 <= y <= 2
        p2 = Polytope()
        p2.vertices = np.array([
            [-2.5, 1.5],
            [0.5, 1.5],
            [0.5, 2],
            [-2.5, 2]
        ]).flatten().tolist()
        p2.normals = np.array([
            [1., 0.],
            [-1., 0.],
            [0., 1.],
            [0., -1.],
        ]).flatten().tolist()
        p2.b = np.array([
            [0.5],
            [2],
            [2],
            [-1.5],
        ]).flatten().tolist()
        self.msg.polytopes = [p1, p2]
        self.get_logger().info("Initialized Fake Polytope Publisher node.")



    def timer_callback(self):
        self.publisher_.publish(self.msg)
        self.get_logger().info(f'Publishing Polytopes')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = FakePolyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
