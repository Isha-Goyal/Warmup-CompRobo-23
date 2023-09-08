import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('drive_square_node')

        self.t = 0
        self.create_timer(0.1, self.update_var)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def update_var(self):
        self.t += 1
        print(self.t)
        msg = Twist()

        if 15<self.t<=21:
            print("turning")
            self.turn_right()
        elif self.t>21:
            self.t = 0
        else:
            msg.linear.x = 0.2
            self.vel_pub.publish(msg)

    def turn_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 3.14
        self.vel_pub.publish(msg)


    

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()