import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
import numpy

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        self.markerlist = MarkerArray()
        self.id = 0

        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.timer = self.create_timer(1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg_pub = self.create_publisher(MarkerArray, 'wall', 10)

        

        # range_max = 5
        # range_min = 0.1

    def process_scan(self, scan):
        self.dist1 = scan.ranges[85]
        dist2 = scan.ranges[95]

        self.angle_error = self.dist1 - dist2
        # print(f"angle error: {self.angle_error}")
    
    def run_loop(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = self.angle_error * 2

        self.vel_pub.publish(msg)

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.get_clock().now()}"
        marker.id = self.id
        self.id += 1

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = -self.dist1 * numpy.cos(1.48)
        marker.pose.position.y = self.dist1 * numpy.sin(1.48)
        marker.pose.position.z = 0.01

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.g = 1.0

        self.markerlist.markers.append(marker)
        self.msg_pub.publish(self.markerlist)
        


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()