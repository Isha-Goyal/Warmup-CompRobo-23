import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')

        self.create_subscription(LaserScan, 'scan' self.process_scan, 10)
        self.timer = self.create_timer(1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def process_scan(self, scan):

    def run_loop(self):
        


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()