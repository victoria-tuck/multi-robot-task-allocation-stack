import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator
import csv

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
        self.locs = [(0, 2.2), (4.25, -27.5), (-7.75, -21.7), (7.85, -21.8), (7.9, -7.5), (-7.75, -7.5)]
        # self.locs = [(0, 2.2), (4.25, -27.5), (-7.75, -21.7)]
        self.starting_idx = 0
        self.ending_idx = 1
        self.goal_idx = 1
        self.iteration = 1
        self.max_iterations = 40
        self.traveling_to_start = False
        self.finished = False
        # self.locs = [(0, 2.2), (4.25, -27.5)]
        # self.locs = []
        location_indices = list(range(len(self.locs)))
        self.travel_times = {key1: {key2: [] for key2 in location_indices if key1 != key2} for key1 in location_indices}
        self.arrived_at_start = False
        self.pull_travel_time = False
        self.start_time = None
        self.loc_idx = 0
        self.data_saved = True

        self.goal_reached = True

    def goal_sequence_callback(self, msg):
        self.locs = [(pose.position.x, pose.position.y) for pose in msg.poses]
        self.loc_idx = 0
        print(f"{self.name} updated its goals: {self.locs}")

    def publish_goal(self):
        if self.goal_idx < len(self.locs) and not self.finished:
            goal = self.locs[self.goal_idx]
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
                self.start_time = self.get_clock().now().nanoseconds * 1.0e-9
                self.goal_reached = False
                self.get_logger().info(f"Using non-build goal_setter. New goal sent: {msg}")
            else:
                self.get_logger().info(f"Waiting for goal {goal} to be reached...")
        elif not self.data_saved:
            self.get_logger().info("All goals reached. Saving...")
            with open("travel_time2.csv", "w") as f:
                write = csv.writer(f)
                for key1, sub_dict in self.travel_times.items():
                    for key2, travel_time_list in sub_dict.items():
                        write.writerow([key1] + [key2] + travel_time_list)
            self.data_saved = True

    def listen_location_callback(self, msg):
        cur_loc = (msg.pose.position.x, msg.pose.position.y)
        # print(f"Received {self.name}'s current location: {cur_loc}")
        if self.goal_idx < len(self.locs):
            if dist(self.locs[self.goal_idx], cur_loc) < DIST_THRES:
                if self.start_time is not None:
                    current_time = self.get_clock().now().nanoseconds * 1.0e-9
                    travel_time = current_time - self.start_time
                    print(f"Travel time: {travel_time}")
                    if not self.traveling_to_start:
                        if self.goal_idx == self.ending_idx:
                            self.travel_times[self.starting_idx][self.ending_idx].append(travel_time)
                        elif self.goal_idx == self.starting_idx:
                            self.travel_times[self.ending_idx][self.starting_idx].append(travel_time)
                        else:
                            self.get_logger().info("Goal index value invalid")
                    self.start_time = None  
                if self.goal_idx == self.starting_idx:
                    if self.iteration >= self.max_iterations:
                        if self.ending_idx + 1 >= len(self.locs):
                            if self.starting_idx + 2 >= len(self.locs):
                                self.finished = True
                            else:
                                self.starting_idx += 1
                                self.ending_idx = self.starting_idx + 1
                                self.goal_idx = self.starting_idx
                                self.traveling_to_start = True
                                self.iteration = 0
                        else:
                            self.ending_idx += 1
                            self.goal_idx = self.ending_idx
                            self.iteration = 1
                    else:
                        if self.iteration == 0:
                            self.traveling_to_start = False
                        self.iteration += 1
                        self.goal_idx = self.ending_idx
                else:
                    self.goal_idx = self.starting_idx
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
