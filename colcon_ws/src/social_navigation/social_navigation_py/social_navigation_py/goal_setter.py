import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped

import math

DIST_THRES = 0.1

def dist(c1, c2):
    return math.sqrt((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2)

class GoalPublisher(Node):
    def __init__(self, name=''):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, name + '/goal_location', 1)
        self.timer_period = 1
        self.timer = self.create_timer(self.timer_period, self.publish_goal)
        self.name = name
        self.location_listener = self.create_subscription(PoseStamped, name + '/robot_location', self.listen_location_callback, 1)

        self.navigator = BasicNavigator()
        self.locs = [(6.5, 11.5), (3, 8), (2, 4)]
        self.loc_idx = 0

        self.goal_reached = True

    def publish_goal(self):
        goal = self.locs[self.loc_idx]
        if self.goal_reached:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.navigator.get_clock().now().to_msg()
            # msg.header.stamp = self.navigator.get_clock().now().to_msg()
            msg.pose.position.x = float(goal[0])
            msg.pose.position.y = float(goal[1])
            msg.pose.position.z = 0.01
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            self.publisher_.publish(msg)
            self.goal_reached = False
            print(f"New goal sent: {msg}")
        else:
            print(f"Waiting for goal {goal} to be reached...")

    def listen_location_callback(self, msg):
        cur_loc = (msg.pose.position.x, msg.pose.position.y)
        # print(f"Received {self.name}'s current location: {cur_loc}")
        if dist(self.locs[self.loc_idx], cur_loc) < DIST_THRES:
            self.loc_idx = min(self.loc_idx + 1, len(self.locs) - 1)
            self.goal_reached = True

def main(args=None):
    rclpy.init(args=args)
    goal_publisher1 = GoalPublisher(name='/tb3')
    goal_publisher2 = GoalPublisher(name='/tb3_adv')
    executor = MultiThreadedExecutor()
    executor.add_node(goal_publisher1)
    executor.add_node(goal_publisher2)
    executor.spin()
    # rclpy.spin(goal_publisher)
    rclpy.shutdown() 

if __name__ == '__main__':
    main()