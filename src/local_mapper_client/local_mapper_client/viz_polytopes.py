import rclpy
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from local_mapper_interfaces.msg import PolytopeArray
from geometry_msgs.msg import Point

from scipy.spatial import Delaunay


class VizPolytopes(Node):
    def __init__(self):
        super().__init__('fake_poly_node')
        self.poly_viz_pub = self.create_publisher(MarkerArray, 'viz_poly', 1)

        self.image_subscriber = self.create_subscription(
            PolytopeArray,
            'free_polytope',
            self.polytope_callback,
            1
        )

        self.get_logger().info("Initialized Viz Polytope node.")

    def polytope_callback(self, msg):
        marker_array = MarkerArray()
        for i, polytope in enumerate(msg.polytopes):
            vertices = polytope.vertices.reshape((-1, 2))
            marker = self.create_filled_polygon_marker_(vertices, marker_id=i)
            marker_array.markers.append(marker)
        self.poly_viz_pub.publish(marker_array)

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


def main(args=None):
    rclpy.init(args=args)
    node = VizPolytopes()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
