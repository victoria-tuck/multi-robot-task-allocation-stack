import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class Init_Planner(Node):
    def __init__(self):
        super().__init__(f'init_planner')

        self.clock = self.get_clock()

        self.timer_period = 1
        self.init_planner_publisher = self.create_publisher(Bool, f'/planner_init', 1)
        self.publish_timer = self.create_timer(self.timer_period, self.publish_init)

    def publish_init(self):
        msg = Bool()
        msg.data = True
        self.init_planner_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    init_planner = Init_Planner()
    rclpy.spin(init_planner)
    init_planner.destroy_node()
    rclpy.shutdown() 


if __name__ == '__main__':
    main()
