import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class ExampleNode(Node):
    def __init__(self):
        super().__init__('marker_creator')
        self.vis_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        timer_period = 10
        self.timer = self.create_timer(timer_period, self.publish_marker)

    def publish_marker(self):
            # create marker
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "my_namespace"
            marker.id = 0

            # set marker attributes
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = 1.0
            marker.pose.position.y = 2.0
            marker.pose.position.z = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.
            marker.scale.z = 1.
            marker.color.a = 1.0
            marker.color.r = 0.5
            marker.color.g = 0.0
            marker.color.b = 0.5

            # publish marker
            self.vis_pub.publish( marker )


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()