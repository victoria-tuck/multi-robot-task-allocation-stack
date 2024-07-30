import csv
import json
import math
import numpy as np
import pickle
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped

DIST_THRES = 0.5

def dist(c1, c2):
    return math.sqrt((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2)

class TravelTimeCollector(Node):
    def __init__(self, name=''):
        super().__init__(f'travel_time_collector')
        self.get_logger().info(f"travel_time_collector started!")
        if name != "":
            prefix = '/' + name
        else:
            prefix = ""

        self.declare_parameter('time_collection_params', rclpy.Parameter.Type.STRING)
        self.declare_parameter('save_file', rclpy.Parameter.Type.STRING)
        time_collection_params = self.get_parameter('time_collection_params').value

        with open(time_collection_params, 'r') as f:
            params = json.load(f)
        self.max_iterations, self.save_mode, self.format = params["iterations"], params["save_mode"], params["format"]
        self.locs = [(loc[0], loc[1]) for loc in params["map_locations"]]
        self.location_ids = params["map_location_ids"]
        with open('roadmap.pkl', 'rb') as file:
            self.roadmap = pickle.load(file)
        self.save_file = self.get_parameter('save_file').value

        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.publish_waypoint)
        self.location_subscriber = self.create_subscription(PoseStamped, prefix + '/robot_location', self.location_callback, 1)

        self.publisher_ = self.create_publisher(PoseStamped, prefix + '/goal_location', 1)

        # location_indices = list(range(len(self.locs)))
        assert len(self.location_ids) > 1
        self.travel_times = {key1: {key2: [] for key2 in self.location_ids if key1 != key2} for key1 in self.location_ids}

        self.starting_idx = 0
        self.ending_idx = 1
        self.goal_idx = 1
        self.iteration = 1
        self.loc_idx = 0
        self.path_idx = 0
        self.waypoint_iter = iter(self.roadmap.get(self.starting_idx).get(self.ending_idx))
        self.waypoint = next(self.waypoint_iter)

        self.traveling_to_start = False
        self.finished = False
        self.start_time = None
        self.data_saved = False
        self.waypoint_reached = True
        self.finishing_path = False

    def location_callback(self, msg):
        """
        Save travel time and update the goal parameters if the current goal has been reached.
        Update waypoint if the current one has been reached.

        Args:
        - msg (PoseStamped): Position message received from controller
        """
        cur_loc = (msg.pose.position.x, msg.pose.position.y)
        assert self.goal_idx < len(self.locs)
        if dist(self.locs[self.goal_idx], cur_loc) < DIST_THRES:

            # Save travel time
            if self.start_time is not None and self.finishing_path:
                current_time = self.get_clock().now().nanoseconds * 1.0e-9
                travel_time = current_time - self.start_time
                self.get_logger().info(f"Travel time: {travel_time}")
                if not self.traveling_to_start:
                    if self.goal_idx == self.ending_idx:
                        self.travel_times[self.starting_idx][self.ending_idx].append(travel_time)
                    elif self.goal_idx == self.starting_idx:
                        self.travel_times[self.ending_idx][self.starting_idx].append(travel_time)
                    else:
                        self.get_logger().info("Goal index value invalid")
                self.start_time = None

            # Update goal idx
            if self.goal_idx == self.starting_idx:

                # Update to the next data collection pair if the max iterations have been reached
                if self.iteration >= self.max_iterations:
                    if self.ending_idx + 1 >= len(self.locs):
                        if self.starting_idx + 2 >= len(self.locs):
                            self.finished = True
                        else:
                            self.starting_idx += 1
                            curr_idx = self.ending_idx
                            self.ending_idx = self.starting_idx + 1
                            self.goal_idx = self.starting_idx
                            self.traveling_to_start = True
                            self.iteration = 0
                            self.waypoint_iter = iter(self.roadmap.get(curr_idx).get(self.starting_idx))
                    else:
                        self.ending_idx += 1
                        self.goal_idx = self.ending_idx
                        self.iteration = 1
                        self.waypoint_iter = iter(self.roadmap.get(self.starting_idx).get(self.ending_idx))
                else:
                    # Check if the starting point has just been reached and set goal idx to end of pair
                    if self.iteration == 0:
                        self.traveling_to_start = False
                    self.iteration += 1
                    self.goal_idx = self.ending_idx
                    self.waypoint_iter = iter(self.roadmap.get(self.starting_idx).get(self.ending_idx))
            else:
                # Set goal idx to start of pair (return trip)
                self.goal_idx = self.starting_idx
                self.waypoint_iter = iter(self.roadmap.get(self.ending_idx).get(self.starting_idx))
            self.waypoint_reached = True
        elif dist(self.waypoint, cur_loc) < DIST_THRES:
            self.waypoint = next(self.waypoint_iter)
            self.waypoint_reached = True

    def publish_waypoint(self):
        """
        Publish next location to visit if current goal has been reached and there is still data to collect.
        Otherwise, save the data.
        """
        if self.goal_idx < len(self.location_ids) and not self.finished:
            if self.waypoint_reached:
                msg = PoseStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.position.x = float(self.waypoint[0])
                msg.pose.position.y = float(self.waypoint[1])
                msg.pose.position.z = 0.01
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = 0.0
                msg.pose.orientation.w = 1.0
                self.publisher_.publish(msg)
                self.start_time = self.get_clock().now().nanoseconds * 1.0e-9
                self.waypoint_reached = False
                self.get_logger().info(f"Using non-build goal_setter. New goal sent: {msg}")
            else:
                self.get_logger().info(f"Waiting for goal {self.waypoint} to be reached...")
        elif not self.data_saved:
            self.data_saved = self.save_data()
    
    def save_data(self):
        """
        Save data if collection procedure has finished.
        """
        self.get_logger().info("All goals reached. Saving...")
        processed_data = self.process_data()
        if self.format == "CSV":
            with open(self.save_file, "w") as f:
                write = csv.writer(f)
                for key1, subdict in processed_data.items():
                    for key2, data in subdict.items():
                        header = [key1, key2]
                        write.writerow(header + data)
        elif self.format == "GRAPH":
            assert self.save_mode != "ALL"
            room_count = len(self.locs)
            travel_time_graph = np.zeros((room_count, room_count))
            for key1, subdict in processed_data.items():
                for key2, data in subdict.items():
                    travel_time_graph[key1][key2] = data[0]
            with open(self.save_file, 'wb') as handle:
                pickle.dump((room_count, travel_time_graph), handle)
        self.get_logger().info(f"Data saved: {processed_data}")
        return True

    def process_data(self):
        all_processed_data = {}
        for key1, subdict in self.travel_times.items():
            processed_subdict = {}
            for key2, travel_time_list in subdict.items():
                if self.save_mode == "MAX_CEIL":
                    processed_data = [math.ceil(max(travel_time_list))]
                elif self.save_mode == "MAX":
                    processed_data = [max(travel_time_list)]
                elif self.save_mode == "AVG":
                    processed_data = [sum(travel_time_list)/len(travel_time_list)]
                elif self.save_mode == "ALL":
                    processed_data = travel_time_list
                processed_subdict[key2] = processed_data
            all_processed_data[key1] = processed_subdict   
        return all_processed_data
                

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
