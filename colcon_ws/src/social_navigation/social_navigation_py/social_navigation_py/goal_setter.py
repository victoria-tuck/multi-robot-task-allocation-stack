import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped, PoseArray
from social_navigation_msgs.msg import Feedback, TimedPoseList
from builtin_interfaces.msg import Time

import json
import math
import numpy as np
import sys
import argparse

DIST_THRES = 1
GOAL_REGION_RADIUS = 0.25 #0.25 #0.5

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

        self.clock = self.get_clock()
        self.actual_arrival_times = []

        self.subscriber = self.create_subscription( TimedPoseList, prefix + '/goal_sequence', self.goal_sequence_callback, 10 )
        self.start_time_subscriber = self.create_subscription(Time, '/start_time', self.start_time_callback, 10)

        self.publisher_ = self.create_publisher(PoseStamped, prefix + '/goal_location', 1)
        self.timer_period = 1.0
        self.goal_timer = self.create_timer(self.timer_period, self.publish_goal)
        self.feedback_timer = self.create_timer(self.timer_period, self.publish_feedback)
        self.name = name
        self.location_listener = self.create_subscription(PoseStamped, prefix + '/robot_location', self.listen_location_callback, 1)
        self.feedback_publisher = self.create_publisher(Feedback, prefix + '/feedback', 1)

        self.timed_locs = []
        self.loc_idx = 0
        self.start_time_s = None

        self.goal_reached = True

    def goal_sequence_callback(self, msg):
        self.timed_locs = [(timed_pose.time, (timed_pose.pose.position.x, timed_pose.pose.position.y)) for timed_pose in msg.poses_with_time]
        print(f"{self.name} updated its goals: {self.timed_locs}")

    def start_time_callback(self, msg):
        self.start_time_s = msg.sec + msg.nanosec * 1e-9
        if self.start_time_s is not None:
            self.get_logger().info(f"Start time received: {self.start_time_s} s")

    def publish_goal(self):
        if self.loc_idx < len(self.timed_locs):
            time_to_reach, goal = self.timed_locs[self.loc_idx]
            current_time = self.clock.now().nanoseconds * 1e-9
            time_reached = time_to_reach < current_time
            if self.goal_reached and time_reached:
                msg = PoseStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = self.get_clock().now().to_msg()
                rng = np.random.default_rng()
                msg.pose.position.x = float(goal[0]) + 2* GOAL_REGION_RADIUS * rng.random() - GOAL_REGION_RADIUS
                msg.pose.position.y = float(goal[1]) + 2* GOAL_REGION_RADIUS * rng.random() - GOAL_REGION_RADIUS
                msg.pose.position.z = 0.01
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = 0.0
                msg.pose.orientation.w = 1.0
                self.publisher_.publish(msg)
                self.goal_reached = False
                self.get_logger().info(f"Using non-build goal_setter. New goal sent: {msg}")
            elif not time_reached:
                self.get_logger().info(f"Agent waiting until {time_to_reach}s. Current time: {current_time}s.")
            # else:
                # self.get_logger().info(f"Waiting for goal {goal} to be reached...")

    def listen_location_callback(self, msg):
        cur_loc = (msg.pose.position.x, msg.pose.position.y)
        if self.loc_idx < len(self.timed_locs):
            if dist(self.timed_locs[self.loc_idx][1], cur_loc) < DIST_THRES:
                current_clock = self.clock.now()
                self.actual_arrival_times.append(current_clock.nanoseconds * 1e-9)
                print(self.actual_arrival_times)
                self.loc_idx += 1
                self.goal_reached = True

    def publish_feedback(self):
        msg = Feedback()
        msg.timing_feedback = self.actual_arrival_times
        self.feedback_publisher.publish(msg)


def main(argv=None):
    # Get input file
    argv = rclpy.utilities.remove_ros_args()[1:]
    print(f"Arguments: {argv}")
    parser = argparse.ArgumentParser(
        description='Start robot goal setters'
    )
    parser.add_argument('-input_file', type=str, help='Scenario file')
    args = parser.parse_args(argv)

    # Pull number of robots
    with open(args.input_file, 'r') as f:
        scenario_setup = json.load(f)
    num_robots = len(list(scenario_setup["agents"].keys()))

    # Initialize Nodes
    rclpy.init()
    executor = MultiThreadedExecutor()
    goal_publishers = []
    for i in range(1, num_robots + 1):
        name = f"robot{i}"
        goal_publisher = GoalSetter(name=name)
        goal_publishers.append(goal_publisher)
        executor.add_node(goal_publisher)
    executor.spin()

    # Clean up
    for node in goal_publishers:
        node.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
