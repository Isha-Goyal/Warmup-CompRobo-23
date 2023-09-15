import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32


class FiniteStateNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        # self.create_timer(0.1, self.run_loop)
        self.subscriber = self.create_subscription(Float32, 'xcord', self.receive_msg, 10)
        self.publisher = self.create_publisher(String, 'command', 10)


    def receive_msg(self, msg):
        cmd = String()
        if msg.data < 1:
            cmd.data = "Follow"
        else:
            cmd.data = "Square"
        self.publisher.publish(cmd)

    
    


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()