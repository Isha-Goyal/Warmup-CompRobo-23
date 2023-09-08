import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # self.angle_error = 0.0

        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.timer = self.create_timer(1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # range_max = 5
        # range_min = 0.1

    def run_loop(self):

        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = self.angle_error * 2

        self.vel_pub.publish(msg)


    def process_scan(self, scan):
        dist1 = scan.ranges[85]
        dist2 = scan.ranges[95]

        self.angle_error = dist1 - dist2
        print(f"angle error: {self.angle_error}")

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()