import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import csv
import json

from geometry_msgs.msg import PoseStamped, PoseArray

import math

DIST_THRES = 0.5

def dist(c1, c2):
    return math.sqrt((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2)

class TravelTimeCollector(Node):
    def __init__(self, name=''):
        super().__init__(f'goal_setter_{name}')
        self.get_logger().info(f"goal_setter_{name} started!")
        if name != "":
            prefix = '/' + name
        else:
            prefix = ""

        self.declare_parameter('time_collection_params', rclpy.Parameter.Type.STRING)
        self.declare_parameter('save_file', rclpy.Parameter.Type.STRING)
        time_collection_params = self.get_parameter('time_collection_params').value

        with open(time_collection_params, 'r') as f:
            params = json.load(f)
        self.max_iterations, self.save_mode = params["iterations"], params["save_mode"]
        self.locs = [(loc[0], loc[1]) for loc in params["map_locations"]]
        self.save_file = self.get_parameter('save_file').value

        self.publisher_ = self.create_publisher(PoseStamped, prefix + '/goal_location', 1)

        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.publish_goal)
        self.location_listener = self.create_subscription(PoseStamped, prefix + '/robot_location', self.listen_location_callback, 1)

        location_indices = list(range(len(self.locs)))
        assert len(location_indices) > 1
        self.travel_times = {key1: {key2: [] for key2 in location_indices if key1 != key2} for key1 in location_indices}

        self.starting_idx = 0
        self.ending_idx = 1
        self.goal_idx = 1
        self.iteration = 1
        self.loc_idx = 0

        self.traveling_to_start = False
        self.finished = False
        self.start_time = None
        self.data_saved = False
        self.goal_reached = True

    def publish_goal(self):
        if self.goal_idx < len(self.locs) and not self.finished:
            goal = self.locs[self.goal_idx]
            if self.goal_reached:
                msg = PoseStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = self.get_clock().now().to_msg()
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
            with open(self.save_file, "w") as f:
                write = csv.writer(f)
                for key1, sub_dict in self.travel_times.items():
                    for key2, travel_time_list in sub_dict.items():
                        header = [key1, key2]
                        if self.save_mode == "MAX_CEIL":
                            write.writerow(header + [math.ceil(max(travel_time_list))])
                        elif self.save_mode == "MAX":
                            write.writerow(header + [max(travel_time_list)])
                        elif self.save_mode == "AVG":
                            write.writerow(header + [sum(travel_time_list)/len(travel_time_list)])
                        elif self.save_mode == "ALL":
                            write.writerow([key1, key2] + travel_time_list)
            self.data_saved = True

    def listen_location_callback(self, msg):
        cur_loc = (msg.pose.position.x, msg.pose.position.y)
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
                self.goal_reached = True

def main(args=None):
    rclpy.init(args=args)
    goal_publisher1 = TravelTimeCollector(name="robot1")
    executor = MultiThreadedExecutor()
    executor.add_node(goal_publisher1)
    executor.spin()
    goal_publisher1.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
