import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from social_navigation_msgs.msg import Feedback, PoseStampedPair, Plan, QueueRequest, QueueMsg
from std_msgs.msg import Bool, String

import json
import math
import numpy as np
import pickle
import sys
import argparse

DIST_THRES = 1
GOAL_REGION_RADIUS = 0.2 #0.25 #0.25 #0.5

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
        self.name = name

        self.clock = self.get_clock()
        self.actual_arrival_times = []

        # with open('roadmap.pkl', 'rb') as file:
        with open('roadmap_and_queues.pkl', 'rb') as file:
            self.roadmap, self.queues, self.queue_to_room = pickle.load(file)

        self.queue_request_timer = self.create_timer(0.5, self.run_queue_check)

        self.subscriber = self.create_subscription( Plan, prefix + '/plan', self.goal_sequence_callback, 10 )
        self.queue_subscribers = {room: self.create_subscription(QueueMsg, f'room{room}/queue', self.make_queue_callback(room), 10 ) for room in range(6)}

        self.activity_publisher = self.create_publisher(Bool, f'{prefix}/active', 10)
        self.publisher_ = self.create_publisher(PoseStampedPair, f'{prefix}/goal_location', 1)
        self.timer_period = 0.1
        self.goal_timer = self.create_timer(self.timer_period, self.publish_goal)
        self.feedback_timer = self.create_timer(self.timer_period, self.publish_feedback)
        self.status_timer = self.create_timer(self.timer_period, self.publish_status)
        self.release_queue_timer = self.create_timer(self.timer_period, self.release_queue)
        self.reminaing_plan_timer = self.create_timer(self.timer_period, self.publish_remaining_plan)
        # self.waiting_timer = self.create_timer(self.timer_period, self.publish_waiting)
        self.name = name
        self.location_listener = self.create_subscription(PoseStamped, prefix + '/robot_location', self.listen_location_callback, 1)
        self.feedback_publisher = self.create_publisher(Feedback, prefix + '/feedback', 1)
        self.queue_request_pub = self.create_publisher(QueueRequest, "/queue_request", 1)
        self.queue_remove_pub = {room_id: self.create_publisher(String, f"/room{room_id}/remove_from_queue", 1) for room_id in range(6)}
        self.remaining_plan_pub = self.create_publisher(Plan, f'{prefix}/remaining_plan', 1)
        # self.waiting_pub = self.create_publisher(Bool, f'{prefix}/waiting', 10)

        self.room_positions = {0: (0, 2.2),
                               1: (7.9, -7.5),
                               2: (7.85, -21.8),
                               3: (4.25, -27.2),
                               4: (-7.75, -21.7),
                               5: (-7.75, -7.5)}

        self.locs = []
        # self.loc_idx = 0
        self.loc_idx = 0
        self.goal_idx = 0
        self.waypoints = []
        self.goal_room = None
        self.current_message_count = 10
        self.current_message = None
        self.queue_position = None
        self.in_queue = False
        self.finished_with_queue = False
        self.added_position = 10
        self.prev_room = None
        self.prev_point = None
        self.waiting = False
        self.reinitialize = False

        self.active = False
        self.goal_reached = True
        self.goal_sequence = []
    
    # def create_pose_from_point(self, point) -> Pose:
    #     msg = Pose()
    #     # print(point)
    #     msg.position.x = float(point[0])
    #     msg.position.y = float(point[1])
    #     return msg

    def publish_remaining_plan(self):
        msg = Plan()
        msg.plan = self.goal_sequence[self.goal_idx:]
        self.remaining_plan_pub.publish(msg)

    def goal_sequence_callback(self, msg):
        def road_coord(prev_rid, next_rid):
            prev_rid_map = self.roadmap.get(prev_rid)
            return prev_rid_map.get(next_rid)

        def plan_to_positions(plan, start):
            positions_list = []
            for prev_id, next_id in zip(plan[start:-1], plan[start+1:]):
                positions = road_coord(prev_id, next_id)
                positions_list.append(positions[1:])
            return positions_list
        
        ind = 0
        for goal in self.goal_sequence:
            if goal != msg.plan[ind+1]:
                break
            ind += 1
        self.goal_sequence = self.goal_sequence[:ind] + list(msg.plan[ind+1:])
        # self.goal_sequence = msg.plan[1:]
        self.locs = self.locs[:ind] + plan_to_positions(msg.plan, ind)
        # self.locs = [(pose.position.x, pose.position.y) for pose in msg.poses]
        # self.get_logger().info(f"{self.name} updated its goals: {self.locs} starting at index {ind}")

    def publish_goal(self):
        if self.goal_idx < len(self.locs):
            self.goal_room = self.goal_sequence[self.goal_idx]
            # print(f"Current goal room: {self.goal_room}")
            # print(f"Current sequence: {self.waypoints}")
            self.waypoints = self.locs[self.goal_idx]
            # print(f"{self.name}'s current sequence: {self.waypoints}")
            # self.get_logger().info(f"Checking goals with index {self.loc_idx} and goal index {self.goal_idx}")
            if self.loc_idx < len(self.waypoints):
                goal = self.waypoints[self.loc_idx]
                if self.loc_idx + 1 < len(self.waypoints):
                    next_goal = self.waypoints[self.loc_idx + 1]
                else:
                    next_goal = goal
                if not self.waiting and self.goal_reached:
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
                    msg.initialize = self.loc_idx == 0 or self.reinitialize #or self.queue_position == 0
                    self.reinitialize = False
                    self.current_message = msg
                    self.current_message_count = 0
                    self.publisher_.publish(msg)
                    self.goal_reached = False
                    self.active = True
                    self.get_logger().info(f"Using non-build goal_setter. New goal sent: ({msg.current_waypoint.pose.position.x}, {msg.current_waypoint.pose.position.y}) then ({msg.next_waypoint.pose.position.x}, {msg.next_waypoint.pose.position.y})")
                # else:
                #     self.get_logger().info(f"{self.name} waiting to send goals.")
            elif self.current_message_count < 10 and self.current_message is not None:
                self.publisher_.publish(self.current_message)
                self.current_message_count += 1
            elif self.in_queue:
                self.active = self.queue_position == 0
                self.get_logger().info("In queue. Not active unless at the head of the queue.")
            # else:
            #     self.active = False
                # else:
                    # self.get_logger().info(f"Waiting for goal {goal} to be reached...")
        else:
            self.active = False

    def publish_status(self):
        msg = Bool()
        msg.data = self.active and not self.waiting
        self.activity_publisher.publish(msg)

    # def publish_waiting(self):
    #     msg = Bool()
    #     msg.data = self.waiting
    #     self.waiting_pub.publish(msg)

    def make_queue_callback(self, room):
        def queue_callback(msg):
            # print(f"Processing queue callback with {msg.allowed_robot} allowed and {msg.robot_queue} as the queue.")
            # print(f"Checking room {room} against desired room {self.goal_room}")
            updated_queue_position = False
            if room == self.goal_room:
                queue_position = [index for index, value in enumerate(msg.robot_queue) if value == self.name]
                updated_queue_position = False
                if msg.allowed_robot == self.name and self.queue_position != 0:
                    print(f"Current queue position: {self.queue_position}. Setting queue position to 0")
                    self.queue_position = 0
                    self.in_queue = True
                    updated_queue_position = True
                elif len(queue_position) > 0 and queue_position[0] + 1 != self.queue_position:
                    print(f"Setting queue position to {queue_position[0] + 1}")
                    self.queue_position = queue_position[0] + 1
                    self.in_queue = True
                    updated_queue_position = True
                elif self.queue_position is None:
                    self.in_queue = False
            if self.queue_position is not None and self.queue_position < self.added_position and updated_queue_position:
                if self.queue_position == 0:
                    self.added_position = 0
                    # self.loc_idx = len(self.locs[self.goal_idx]) + 1
                    next_idx = len(self.locs[self.goal_idx])
                    self.locs[self.goal_idx] += self.queue_to_room[self.goal_room]
                    self.waypoints = self.locs[self.goal_idx]
                    self.loc_idx = next_idx
                    print(f"New path to room for robot {self.name}: {self.locs[self.goal_idx]} with next planned position {self.waypoints[self.loc_idx]}")
                else:
                    self.added_position = self.queue_position
                    self.locs[self.goal_idx].append(self.queues[self.goal_room][self.queue_position-1])
                    self.waypoints = self.locs[self.goal_idx]
                # self.loc_idx += 2
                self.goal_reached = True
                self.active = True
                if self.waiting:
                    self.reinitialize = True
                    self.get_logger().info(f"Setting {self.name} to reinitialize after waiting")
                self.waiting = False
                self.get_logger().info(f"Updating queue. {self.name} active")
        return queue_callback

    def listen_location_callback(self, msg):
        self.position = (msg.pose.position.x, msg.pose.position.y)
        if self.goal_idx < len(self.locs) and self.loc_idx < len(self.waypoints):
            # self.get_logger().info(f"Checking current position {self.position} against goal {self.waypoints[self.loc_idx]}")
            if dist(self.waypoints[self.loc_idx], self.position) < DIST_THRES:
                current_clock = self.clock.now()
                # self.get_logger().info(f"Reached next waypoint")
                # print(self.actual_arrival_times) # ToDo: Make this not count the queue multiple times
                arrival_point = self.room_positions[self.goal_room]
                if self.loc_idx + 1 < len(self.waypoints):
                    print("Increased waypoint index")
                    self.loc_idx += 1
                    self.goal_reached = True
                    self.waiting = False
                elif abs(self.waypoints[self.loc_idx][0] - arrival_point[0]) < 0.3 and abs(self.waypoints[self.loc_idx][1] - arrival_point[1]) < 0.3:
                    self.actual_arrival_times.append(current_clock.nanoseconds * 1e-9)
                    print("Increased goal index")
                    self.goal_idx += 1
                    self.loc_idx = 0
                    self.prev_room = self.goal_room
                    if self.goal_idx < len(self.locs):
                        self.waypoints = self.locs[self.goal_idx]
                        self.goal_room = self.goal_sequence[self.goal_idx]
                    else:
                        self.waypoints = []
                        self.goal_room = None
                    self.goal_reached = True
                    self.finished_with_queue = True
                    self.waiting = False
                else:
                    # print(f"{self.name} switched into waiting with waypoints: {self.waypoints}, goal index: {self.goal_idx}, and loc index: {self.loc_idx}")
                    self.goal_reached = False
                    self.waiting = True

    def run_queue_check(self):
        if self.goal_room is not None and not self.in_queue:
            arrival_point = self.room_positions[self.goal_room]
            # arrival_point = self.waypoints[-1]
            if abs(self.position[0] - arrival_point[0]) < 5.0 and abs(self.position[1] - arrival_point[1]) < 5.0:
                queue_request_msg = QueueRequest()
                queue_request_msg.room_id = self.goal_room
                queue_request_msg.robot_name = self.name
                self.in_queue = False
                self.queue_position = None
                self.added_position = 10
                self.queue_request_pub.publish(queue_request_msg)

    def release_queue(self):
        if self.finished_with_queue:
            prev_point = self.room_positions[self.prev_room]
            # Moved away from room or room is last task position
            if abs(self.position[0] - prev_point[0]) > 5.0 or abs(self.position[1] - prev_point[1]) > 5.0 or self.goal_idx >= len(self.locs):
                queue_remove_msg = String()
                queue_remove_msg.data = self.name
                self.queue_remove_pub[self.prev_room].publish(queue_remove_msg)
                self.finished_with_queue = False
                self.queue_position = None
                self.added_position = 10
                self.in_queue = False


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
