import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
import numpy

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # used a marker array to display the wall
        self.markerlist = MarkerArray()
        self.id = 0

        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.timer = self.create_timer(1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg_pub = self.create_publisher(MarkerArray, 'wall', 10)
        self.last_timestamp = None

    def process_scan(self, scan):
        # we're using two angles: 85 and 95 degrees
        self.dist1 = scan.ranges[85]
        dist2 = scan.ranges[95]
        self.last_timestamp = scan.header.stamp
        self.angle_error = self.dist1 - dist2
    
    def run_loop(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = self.angle_error * 2 # turning to correct is proportional to the error
        self.vel_pub.publish(msg)

        # this section is to publish markers continuously for where the wall is (visualize section)
        marker = Marker()
        marker.header.frame_id = "base_link"
        if self.last_timestamp is not None:
            marker.header.stamp = self.last_timestamp
        else:
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