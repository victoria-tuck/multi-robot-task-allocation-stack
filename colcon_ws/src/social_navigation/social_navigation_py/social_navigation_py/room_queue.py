from queue import Queue
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from social_navigation_msgs.msg import QueueMsg, QueueRequest
from std_msgs.msg import String

class Room_Queue(Node):
    def __init__(self, room_id=None):
        super().__init__(f'room_queue_{room_id}')

        self.room_id = room_id
        self.robot_queue = Queue(maxsize=6)
        self.robot_set = set()

        self.queue_request_subscriber = self.create_subscription(QueueRequest, '/queue_request', self.request_callback, 1)
        self.remove_request_subscriber = self.create_subscription(String, f'/room{self.room_id}/remove_from_queue', self.queue_remove_callback, 1)
        self.queue_publisher = self.create_publisher(QueueMsg, f'/room{self.room_id}/queue', 10)

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.publish_queue)
        self.allowed_robot = ""

    def request_callback(self, msg):
        if msg.room_id == self.room_id:
            robot_id = msg.robot_name  # Assuming the robot ID is in msg.data
        
            # Check if the robot is already in the set
            if len(self.robot_set) == 0:
                self.allowed_robot = msg.robot_name
                self.robot_set.add(robot_id)
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
        if msg.data in self.robot_set:
            self.get_logger().info(f"Removing {msg.data} from queue.")
            try:
                next_allowed = self.robot_queue.get(block=False)
                self.allowed_robot = next_allowed
            except:
                self.allowed_robot = ""
                print("The queue is empty.")
            self.robot_set.remove(msg.data)
            self.get_logger().info(f"Finished removing from queue. New allowed robot: {self.allowed_robot} and queue: {list(self.robot_set)}")

    def publish_queue(self):
        self.get_logger().info(f"Publishing queue")
        msg = QueueMsg()
        msg.allowed_robot = self.allowed_robot
        msg.robot_queue = list(self.robot_queue.queue)
        
        self.queue_publisher.publish(msg)
        self.get_logger().info(f"Robot allowed in room: {self.allowed_robot} and robots in queue: {list(self.robot_queue.queue)}")


def main(args=None):
    # rclpy.init(args=args)
    # room_queue = Room_Queue()
    # rclpy.spin(room_queue)
    # room_queue.destroy_node()
    # rclpy.shutdown() 

    num_rooms = 6

    # Initialize Nodes
    rclpy.init()
    executor = MultiThreadedExecutor()
    room_queues = []
    for id in range(0, num_rooms):
        room_queue = Room_Queue(room_id=id)
        room_queues.append(room_queue)
        executor.add_node(room_queue)
    executor.spin()

    # Clean up
    for node in room_queues:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()