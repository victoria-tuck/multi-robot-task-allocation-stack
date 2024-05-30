# import sys
# sys.path.append("./SMrTa/src/")
# from SMrTa.src.MRTASolver import MRTASolver

# from MRTASolver import MRTASolver

# s = MRTASolver()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        print("Initializing publisher node...")  # Debug print
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        print("Timer callback called...")  # Debug print
        msg = String()
        msg.data = f'Hello, world: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    print("ROS 2 initialized...")  # Debug print
    node = PublisherNode()
    rclpy.spin(node)
    print("Shutting down node...")  # Debug print
    node.destroy_node()
    rclpy.shutdown()
    print("ROS 2 shutdown complete...")  # Debug print

if __name__ == '__main__':
    main()

