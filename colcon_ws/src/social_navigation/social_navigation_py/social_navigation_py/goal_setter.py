import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped, PoseArray
from social_navigation_msgs.msg import Feedback, PoseStampedPair
from std_msgs.msg import Bool

import json
import math
import numpy as np
import sys
import argparse

DIST_THRES = 1
GOAL_REGION_RADIUS = 0.4 #0.25 #0.25 #0.5

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

        self.subscriber = self.create_subscription( PoseArray, prefix + '/goal_sequence', self.goal_sequence_callback, 10 )

        self.activity_publisher = self.create_publisher(Bool, f'{prefix}/active', 10)
        self.publisher_ = self.create_publisher(PoseStampedPair, f'{prefix}/goal_location', 1)
        self.timer_period = 0.1
        self.goal_timer = self.create_timer(self.timer_period, self.publish_goal)
        self.feedback_timer = self.create_timer(self.timer_period, self.publish_feedback)
        self.status_timer = self.create_timer(self.timer_period, self.publish_status)
        self.name = name
        self.location_listener = self.create_subscription(PoseStamped, prefix + '/robot_location', self.listen_location_callback, 1)
        self.feedback_publisher = self.create_publisher(Feedback, prefix + '/feedback', 1)

        self.locs = []
        self.loc_idx = 0
        self.current_message_count = 10
        self.current_message = None

        self.active = False
        self.goal_reached = True

    def goal_sequence_callback(self, msg):
        self.locs = [(pose.position.x, pose.position.y) for pose in msg.poses]
        # print(f"{self.name} updated its goals: {self.locs}")

    def publish_goal(self):
        if self.loc_idx < len(self.locs):
            goal = self.locs[self.loc_idx]
            if self.loc_idx + 1 < len(self.locs):
                next_goal = self.locs[self.loc_idx + 1]
            else:
                next_goal = goal
            if self.goal_reached:
                msg = PoseStampedPair()
                current_waypoint = PoseStamped()
                current_waypoint.header.frame_id = "map"
                current_waypoint.header.stamp = self.get_clock().now().to_msg()
                rng = np.random.default_rng()
                current_waypoint.pose.position.x = float(goal[0]) + 2* GOAL_REGION_RADIUS * rng.random() - GOAL_REGION_RADIUS
                current_waypoint.pose.position.y = float(goal[1]) + 2* GOAL_REGION_RADIUS * rng.random() - GOAL_REGION_RADIUS
                current_waypoint.pose.position.z = 0.01
                current_waypoint.pose.orientation.x = 0.0
                current_waypoint.pose.orientation.y = 0.0
                current_waypoint.pose.orientation.z = 0.0
                current_waypoint.pose.orientation.w = 1.0
                
                next_waypoint = PoseStamped()
                next_waypoint.header.frame_id = "map"
                next_waypoint.header.stamp = self.get_clock().now().to_msg()
                rng = np.random.default_rng()
                if goal == next_goal:
                    next_waypoint.pose.position.x = current_waypoint.pose.position.x
                    next_waypoint.pose.position.y = current_waypoint.pose.position.y
                else:
                    next_waypoint.pose.position.x = float(next_goal[0]) + 2* GOAL_REGION_RADIUS * rng.random() - GOAL_REGION_RADIUS
                    next_waypoint.pose.position.y = float(next_goal[1]) + 2* GOAL_REGION_RADIUS * rng.random() - GOAL_REGION_RADIUS
                next_waypoint.pose.position.z = 0.01
                next_waypoint.pose.orientation.x = 0.0
                next_waypoint.pose.orientation.y = 0.0
                next_waypoint.pose.orientation.z = 0.0
                next_waypoint.pose.orientation.w = 1.0 

                msg.current_waypoint = current_waypoint
                msg.next_waypoint = next_waypoint
                self.current_message = msg
                self.current_message_count = 0
                self.publisher_.publish(msg)
                self.goal_reached = False
                self.active = True
                self.get_logger().info(f"Using non-build goal_setter. New goal sent: ({msg.current_waypoint.pose.position.x}, {msg.current_waypoint.pose.position.y}) then ({msg.next_waypoint.pose.position.x}, {msg.next_waypoint.pose.position.y})")
        elif self.current_message_count < 10 and self.current_message is not None:
            self.publisher_.publish(self.current_message)
            self.current_message_count += 1
        else:
            self.active = False
            # else:
                # self.get_logger().info(f"Waiting for goal {goal} to be reached...")

    def publish_status(self):
        msg = Bool()
        msg.data = self.active
        self.activity_publisher.publish(msg)

    def listen_location_callback(self, msg):
        cur_loc = (msg.pose.position.x, msg.pose.position.y)
        if self.loc_idx < len(self.locs):
            if dist(self.locs[self.loc_idx], cur_loc) < DIST_THRES:
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
