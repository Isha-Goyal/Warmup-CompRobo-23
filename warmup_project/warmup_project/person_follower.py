import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
import math
from std_msgs.msg import String


class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')

        self.x_cord = 0.0
        self.y_cord = 0.0
        self.dist_way = .4
        self.run = True

        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.timer = self.create_timer(.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg_pub = self.create_publisher(Marker, 'wall', 10)
        self.xcord_pub = self.create_publisher(Float32, 'xcord', 10)
        self.subscriber = self.create_subscription(String, 'command', self.receive_msg, 10)


    def process_scan(self, scan):
        left_angle = -30
        right_angle = 30
        self.dist_above30 = scan.ranges[:right_angle]
        self.dist_below30 = scan.ranges[left_angle:]
        self.dist = self.dist_below30 + self.dist_above30

        
        degrees = range(left_angle,right_angle)
        counter = 0

        for i in range(len(self.dist)):
            if not math.isinf(self.dist[i]): 
                self.y_cord += self.dist[i]*math.sin(math.radians(degrees[i]))
                self.x_cord += self.dist[i]*math.cos(math.radians(degrees[i]))
                counter += 1
        if counter != 0:
            self.x_cord = self.x_cord / counter
            self.y_cord = self.y_cord / counter

        marker = Marker()
        marker.header.frame_id = "base_link"

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.x_cord
        marker.pose.position.y = self.y_cord
        marker.pose.position.z = 0.01

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.g = 1.0

        self.msg_pub.publish(marker)
        msg = Float32()
        msg.data = self.x_cord
        self.xcord_pub.publish(msg)


    def receive_msg(self, cmd):
        self.run = cmd.data == "Follow"
        

    def run_loop(self):
        if self.run:
            msg = Twist()
            if self.x_cord > self.dist_way:
                msg.linear.x = 0.2
            else:
                msg.linear.x = 0.0
            print(self.x_cord, self.y_cord)

            msg.angular.z = self.y_cord
            self.vel_pub.publish(msg)
        

        


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()