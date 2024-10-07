from queue import Queue
import rclpy
from rclpy.node import Node

from social_navigation_msgs.msg import QueueMsg, QueueRequest
from std_msgs.msg import String

class Room_Queue(Node):
    def __init__(self):
        super().__init__(f'room_queue')

        self.room_id = 1
        self.robot_queue = Queue(maxsize=6)
        self.robot_set = set()

        self.queue_request_subscriber = self.create_subscription(QueueRequest, '/queue_request', self.request_callback, 1)
        self.remove_request_subscriber = self.create_subscription(String, '/room1/remove_from_queue', self.queue_remove_callback, 1)
        self.queue_publisher = self.create_publisher(QueueMsg, f'/room1/queue', 10)

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.publish_queue)
        self.allowed_robot = ""

    def request_callback(self, msg):
        if msg.room_id == self.room_id:
            robot_id = msg.robot_name  # Assuming the robot ID is in msg.data
        
            # Check if the robot is already in the set
            if len(self.robot_set) == 0:
                self.allowed_robot = msg.robot_name
            elif robot_id not in self.robot_set:
                if not self.robot_queue.full():
                    # Add to both queue and set
                    self.robot_queue.put(robot_id)
                    self.robot_set.add(robot_id)
                    self.get_logger().info(f"Robot {robot_id} added to queue.")
                else:
                    self.get_logger().info("Queue is full. Dropping new robot.")
            # else:
            #     self.get_logger().info(f"Robot {robot_id} is already in the queue.")

    def queue_remove_callback(self, msg):
        self.get_logger().info(f"Removing {self.allowed_robot} from queue.")
        self.allowed_robot = self.robot_queue.get()
        self.robot_set.remove(msg.data)

    def publish_queue(self):
        msg = QueueMsg()
        msg.allowed_robot = self.allowed_robot
        msg.robot_queue = list(self.robot_set)
        
        self.queue_publisher.publish(msg)
        self.get_logger().info(f"Robot allowed in room: {self.allowed_robot} and robots in queue: {list(self.robot_set)}")


def main(args=None):
    rclpy.init(args=args)
    room_queue = Room_Queue()
    rclpy.spin(room_queue)
    room_queue.destroy_node()
    rclpy.shutdown() 


if __name__ == '__main__':
    main()