import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import math

class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')

        self.create_subscription(LaserScan, 'scan' self.process_scan, 10)
        self.timer = self.create_timer(1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg_pub = self.create_publisher(Marker, 'wall', 10)


    def process_scan(self, scan):
        left_angle = -30
        right_angle = 30
        self.dist_above30 = scan.ranges[:right_angle]
        self.dist_below30 = scan.ranges[left_angle:]
        self.dist = self.dist_below30 + self.dist_above30

        self.x_cord = 0
        self.y_cord = 0
        degrees = range(left_angle,right_angle)
        for i in range(len(self.dist)):
            self.x_cord += self.dist[i]*math.sin(math.radians(degrees[i]))
            self.y_cord += self.dist[i]*math.cos(math.radians(degrees[i]))
        self.x_cord = self.x_cord / len(self.dist)
        self.y_cord = self.y_cord / len(self.dist)


    def run_loop(self, scan):
        marker = Marker()
        #you are currently setting up a marker of the laser
        


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()