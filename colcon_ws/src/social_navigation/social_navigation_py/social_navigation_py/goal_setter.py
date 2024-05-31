import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped, PoseArray

import math

DIST_THRES = 0.5

def dist(c1, c2):
    return math.sqrt((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2)

class GoalSetter(Node):
    def __init__(self, name=''):
        super().__init__(f'goal_setter_{name}')
        self.get_logger().info(f"goal_setter_{name} started!")
        if name != "":
            prefix = '/' + name
        else:
            prefix = ""

        self.subscriber = self.create_subscription( PoseArray, prefix + '/goal_sequence', self.goal_sequence_callback, 10 )
        # self.subscriber = self.create_subscription( PoseArray, '/goal_sequence', self.goal_sequence_callback, 10 )

        self.publisher_ = self.create_publisher(PoseStamped, prefix + '/goal_location', 1)
        # self.publisher_ = self.create_publisher(PoseStamped, '/goal_location', 1)
        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.publish_goal)
        self.name = name
        self.location_listener = self.create_subscription(PoseStamped, prefix + '/robot_location', self.listen_location_callback, 1)
        # self.location_listener = self.create_subscription(PoseStamped, '/robot_location', self.listen_location_callback, 1)

        # self.navigator = BasicNavigator()
        # self.delivery_locations = [(0, 2.2), (4.25, -27.5), (-7.75, -21.7), (7.85, -21.8), (7.9, -7.5), (-7.75, -7.5)]
        # self.locs = [(0, 2.2), (4.25, -27.5), (-7.75, -21.7), (7.85, -21.8), (7.9, -7.5), (-7.75, -7.5)]
        self.locs = []
        # self.travel_time = []
        # self.pull_travel_time = False
        # self.start_time = None
        self.loc_idx = 0

        self.goal_reached = True

    def goal_sequence_callback(self, msg):
        self.locs = [(pose.position.x, pose.position.y) for pose in msg.poses]
        self.loc_idx = 0
        print(f"{self.name} updated its goals: {self.locs}")

    def publish_goal(self):
        if self.loc_idx < len(self.locs):
            goal = self.locs[self.loc_idx]
            if self.goal_reached:
                msg = PoseStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = self.get_clock().now().to_msg()
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
                self.get_logger().info(f"Using non-build goal_setter. New goal sent: {msg}")
            else:
                self.get_logger().info(f"Waiting for goal {goal} to be reached...")

    def listen_location_callback(self, msg):
        cur_loc = (msg.pose.position.x, msg.pose.position.y)
        print(f"Received {self.name}'s current location: {cur_loc}")
        if self.loc_idx < len(self.locs):
            if dist(self.locs[self.loc_idx], cur_loc) < DIST_THRES:
                # if self.start_time is not None:
                #     current_time = Clock().now()
                #     self.travel_time.append(current_time - self.start_time)
                #     self.start_time = current_time
                self.loc_idx += 1
                # if self.loc_idx + 1 > len(self.locs):
                #     self.start_time = None
                # self.loc_idx = min(self.loc_idx + 1, len(self.locs) - 1)
                self.goal_reached = True

def main(args=None):
    rclpy.init(args=args)
    goal_publisher1 = GoalSetter(name="robot1")
    goal_publisher2 = GoalSetter(name='robot2')
    executor = MultiThreadedExecutor()
    executor.add_node(goal_publisher1)
    executor.add_node(goal_publisher2)
    executor.spin()
    goal_publisher1.destroy_node()
    goal_publisher2.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
