import json
import math
import pickle
import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from social_navigation_msgs.msg import Feedback

from MRTASolver import MRTASolver
# ToDo: Put load_config in run_realistic_setting instead of create_randomized_inputs
from MRTASolver.create_randomized_inputs import load_config


class Dispatcher(Node):
    def __init__(self):
        super().__init__(f'dispatcher')

        # Pull agent information from file
        self.declare_parameter('input_file', rclpy.Parameter.Type.STRING)
        input_file = self.get_parameter('input_file').value

        with open(input_file, 'r') as f:
            scenario_setup = json.load(f)
        robot_list = list(scenario_setup["agents"].keys())

        # Wait until start time is valid
        self.clock = self.get_clock()
        def wait_for_non_zero_clock_time():
            self.get_logger().info("Waiting for non-zero clock time...")
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                current_clock = self.clock.now()
                current_time_s = current_clock.nanoseconds * 1e-9
                if current_time_s > 0:
                    self.run_start_time_s = current_time_s
                    self.get_logger().info(f"Non-zero clock time received: {self.run_start_time_s} s")
                    break
        wait_for_non_zero_clock_time()

        self.get_logger().info("Dispatcher starts!")

        # Initialize variables
        self.timer_period = 1.0
        self.task_set_index = 0
        self.positions_lists = []
        self.feedback = [[] for i in range(len(robot_list))]
        self.has_new_sequences = False
        with open('roadmap.pkl', 'rb') as file:
            self.roadmap = pickle.load(file)

        # Initialize subscribers, publishers, and callbacks
        self.feedback_subscribers = { robot: self.create_subscription( Feedback,  f'/{robot}/feedback', self.make_feedback_callback(i), 1) for i, robot in enumerate(robot_list) }
        self.publishers_ = { robot: self.create_publisher(PoseArray, f'/{robot}/goal_sequence', 1) for robot in robot_list }
        self.update_plan_timer = self.create_timer(self.timer_period, self.update_plan_callback)
        self.publish_timer = self.create_timer(self.timer_period, self.publish_goal_sequence_callback)

        self.initialize_solver()

    def initialize_solver(self):
        # MRTASolver arguments
        # file = 'simulation/testcase_4agents_duplicate_tasks.json'
        file = 'simulation/testcase_6agents.json'
        solver = 'bitwuzla'
        theory = 'QF_UFBV'
        capacity = 2
        fidelity = 1
        free_action_points = False
        timeout = 3600
        basename = None
        default_deadline = 100
        incremental = True
        verbose = False
        
        self.agents, self.tasks_stream = load_config(file)
        num_agents = len(self.agents)
        tot_tasks = sum([len(tasks) for tasks, _ in self.tasks_stream])
        num_aps = math.ceil(tot_tasks / num_agents) * 2 + 1
        aps_list = list(range(3, num_aps+1, 2))

        # room_dictionary = load_weighted_graph()
        # room_count, room_graph = dictionary_to_matrix(room_dictionary)
        with open('travel_time_07_22_24_full.pkl', 'rb') as file:
            data = pickle.load(file)
            room_count, room_graph = data
        
        self.solver = MRTASolver(solver, theory, self.agents, self.tasks_stream, room_graph, capacity, num_aps, fidelity, free_action_points, timeout, basename, default_deadline, aps_list, incremental, verbose)
        self.plans = []
        self.has_new_sequences = True

    def coord(self, rid):
        coordinate_map = {0: (0, 2.2),
                          1: (7.9, -7.5),
                          2: (7.85, -21.8),
                          3: (4.25, -27.2),
                          4: (-7.75, -21.7),
                          5: (-7.75, -7.5)}
        old_coordinate_map = {0: (0, 2.2), 
                          1: (4.25, -27.2), # updated to (4.5, -27.2), # was (4.25, -27.5)
                          2: (-7.75, -21.7), 
                          3: (7.85, -21.8), 
                          4: (7.9, -7.5), 
                          5: (-7.75, -7.5)}
        return coordinate_map.get(rid, None)
    
    def road_coord(self, prev_rid, next_rid):
        prev_rid_map = self.roadmap.get(prev_rid)
        return prev_rid_map.get(next_rid)

    def room_id(self, task_id, agents, tasks_stream):
        task_counts = [len(tasks) for tasks, _ in tasks_stream]
        num_agents = len(agents)
        prev_ids = num_agents
        ind = 0
        if task_id >= 0 and task_id < num_agents:
            return agents[task_id].start
        while ind < len(task_counts):
            curr_count = task_counts[ind]
            if task_id >= prev_ids and task_id < 2*curr_count + prev_ids:
                in_set_id = task_id - prev_ids
                task_num = math.floor(in_set_id / 2)
                if in_set_id % 2 == 0:
                    return tasks_stream[ind][0][task_num].start
                else:
                    return tasks_stream[ind][0][task_num].end
            ind += 1
            prev_ids += curr_count * 2
        assert False, f"Task id {task_id} does not exist"

    def is_agent_id(self, id, num_agents):
        return id >= 0 and id < num_agents

    def get_plan(self, id_sequence, agents, tasks_stream):
        num_agents = len(agents)
        plan = []
        started_plan = False
        for i, task_id in enumerate(id_sequence):
            # print(task_id)
            if self.is_agent_id(task_id, num_agents) and started_plan:
                return plan
            else:
                started_plan = True
            plan.append(self.room_id(task_id, agents, tasks_stream))
        return plan

    def create_pose_from_point(self, point) -> Pose:
        msg = Pose()
        # print(point)
        msg.position.x = float(point[0])
        msg.position.y = float(point[1])
        return msg

    def update_plan_callback(self):
        def plan_to_positions(plan):
            positions_list = []
            for prev_id, next_id in zip(plan[:-1], plan[1:]):
                positions = self.road_coord(prev_id, next_id)
                positions_list += positions
            return positions_list

        current_time_s = self.clock.now().nanoseconds * 1e-9
        if self.task_set_index < len(self.tasks_stream):
            next_tasks, next_batch_arrives = self.tasks_stream[self.task_set_index]
            # next_batch_arrives, next_plan = self.plans[self.task_set_index]
            # self.get_logger().info(f"Current time: {current_time_s - self.run_start_time_s}")
            if current_time_s - self.run_start_time_s > next_batch_arrives:
                self.get_logger().info(f"New tasks arrived at {next_batch_arrives}s")
                next_plan = self.solver.allocate_next_task_set(self.feedback)
                self.plans.append(next_plan)
                positions_lists = []
                for agt in next_plan['agt']:
                    # print(f"Agent's ids: {agt['id']}")
                    room_ids = self.get_plan(agt['id'], self.agents, self.tasks_stream)
                    positions_lists.append(plan_to_positions(room_ids))
                    # pose_lists.append([self.coord(rid) for rid in room_ids])
                    # Test case that some times caused issues:
                    # if self.task_set_index == 0:
                        # pose_lists = [[(0, 2.2), (-7.75, -21.7), (-7.75, -7.5)], [(4.25, -27.5), (4.25, -27.5), (7.9, -7.5)]]
                        # pose_lists = [[(0, 2.2), (-7.75, -21.7), (7.9, -7.5), (-7.75, -7.5), (7.9, -7.5)], [(4.25, -27.5), (4.25, -27.5),(-7.75, -7.5)]]
                        # pose_lists = [[(0, 2.2), (7.9, -7.5), (-7.75, -21.7), (-7.75, -7.5), (7.9, -7.5)], [(4.25, -27.5), (-7.75, -7.5)]]
                        # pose_lists = [[(0, 2.2), (-7.75, -7.5)], [(4.25, -27.5), (-7.75, -7.5)]]
                    # else:
                        # pose_lists = [[(0, 2.2), (-7.75, -21.7), (-7.75, -7.5), (7.85, -21.8), (0, 2.2)], [(4.25, -27.5), (4.25, -27.5), (7.9, -7.5), (7.9, -7.5), (-7.75, -7.5)]]
                        # pose_lists = [[(0, 2.2), (7.9, -7.5), (-7.75, -21.7), (-7.75, -7.5), (7.9, -7.5)], [(4.25, -27.5), (-7.75, -7.5)]]
                        # pose_lists = [[(0, 2.2), (-7.75, -21.7), (7.9, -7.5), (-7.75, -7.5), (7.9, -7.5)], [(4.25, -27.5), (4.25, -27.5), (-7.75, -7.5)]]
                self.positions_lists = positions_lists
                self.task_set_index += 1
                self.has_new_sequences = True

    def publish_goal_sequence_callback(self):
        # ToDo: Add intermediate nodes and times in here
        for (name, publisher), position_list in zip(self.publishers_.items(), self.positions_lists):
            msg = PoseArray()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.poses = [self.create_pose_from_point(pos) for pos in position_list][1:]
            publisher.publish(msg)
            if self.has_new_sequences:
                self.get_logger().info(f"New goal sequence sent to {name}: {position_list[1:]}")
        self.has_new_sequences = False

    def make_feedback_callback(self, index):
        def feedback_callback(msg):
            timing_feedback = [sim_time - self.run_start_time_s for sim_time in msg.timing_feedback]
            self.feedback[index] = timing_feedback
            # print(f"Feedback received so far: {self.feedback}")
        return feedback_callback


def main(args=None):
    rclpy.init(args=args)
    dispatcher = Dispatcher()
    rclpy.spin(dispatcher)
    dispatcher.destroy_node()
    rclpy.shutdown() 


if __name__ == '__main__':
    main()
