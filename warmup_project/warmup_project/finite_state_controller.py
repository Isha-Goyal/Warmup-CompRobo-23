import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class FiniteStateNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.create_timer(0.1, self.run_loop)

        self.subscriber = self.create_subscription(String, 'greetings', self.receive_msg, 10)

    def run_loop(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()