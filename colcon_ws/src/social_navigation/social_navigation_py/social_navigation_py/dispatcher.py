import json
import math
import pickle
import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from social_navigation_msgs.msg import Feedback, TimedPose, TimedPoseList
from builtin_interfaces.msg import Time

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
                    self.run_start_time = current_clock
                    self.run_start_time_s = current_time_s
                    self.get_logger().info(f"Non-zero clock time received: {self.run_start_time_s} s")
                    break
        wait_for_non_zero_clock_time()

        self.get_logger().info("Dispatcher starts!")

        # Initialize variables
        self.timer_period = 1.0
        self.task_set_index = 0
        self.timed_position_lists = []
        self.feedback = [[] for i in range(len(robot_list))]
        self.has_new_sequences = False
        self.lock_hold_time = 45

        # Initialize subscribers, publishers, and callbacks
        self.feedback_subscribers = { robot: self.create_subscription( Feedback,  f'/{robot}/feedback', self.make_feedback_callback(i), 1) for i, robot in enumerate(robot_list) }
        self.publishers_ = { robot: self.create_publisher(TimedPoseList, f'/{robot}/goal_sequence', 1) for robot in robot_list }
        self.start_time_publisher = self.create_publisher(Time, '/start_time', 10)
        self.update_plan_timer = self.create_timer(self.timer_period, self.update_plan_callback)
        self.publish_timer = self.create_timer(self.timer_period, self.publish_goal_sequence_callback)
        self.start_time_timer = self.create_timer(self.timer_period, self.start_time_callback)

        self.initialize_solver()

    def initialize_solver(self):
        # MRTASolver arguments
        file = 'simulation/testcase_4agents_duplicate_tasks_6_6_V3.json'
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
        with open('weighted_graph_hospital.pkl', 'rb') as file:
            data = pickle.load(file)
            room_count, room_graph = data
        
        self.room_graph = room_graph
        self.solver = MRTASolver(solver, theory, self.agents, self.tasks_stream, room_graph, capacity, num_aps, fidelity, free_action_points, timeout, basename, default_deadline, aps_list, incremental, verbose)
        self.plans = []
        self.has_new_sequences = True

    def coord(self, rid):
        room_coordinate_map = {0: (0, 2.2),
                          1: (7.9, -7.5),
                          2: (7.85, -21.8),
                          3: (4.25, -27.5),
                          4: (-7.75, -21.7),
                          5: (-7.75, -7.5)}
        old_coordinate_map = {0: (0, 2.2), 
                          1: (4.25, -27.5), # updated to (4.5, -27.75), # was (4.25, -27.5)
                          2: (-7.75, -21.7), 
                          3: (7.85, -21.8), 
                          4: (7.9, -7.5), 
                          5: (-7.75, -7.5)}
        return room_coordinate_map.get(rid, None)
    
    def get_hold_coord(self, prev_rid, rid):
        """
        Gets the coordinate that an agent should hold at.

        Args:
        - prev_rid: room id agent is coming from
        -      rid: room id agent is going to
        """
        # ToDo: CHANGE TO CORRECT VALUES
        hold_coord_map = {0: {0: (0, 2.2), # (1.7, 2.6),
                              1: (5.6, -6.5),
                              2: (5.6, -20.8),
                              3: (3.6, -25.2),
                              4: (-5.3, -20.4),
                              5: (-5.4, -6.8)},
                          1: {0: (1.7, 2.6),
                              1: (7.9, -7.5), # (5.6, -6.5),
                              2: (5.6, -20.8),
                              3: (3.6, -25.2),
                              4: (-5.3, -20.4),
                              5: (-5.4, -6.8)},
                          2: {0: (1.7, 2.6),
                              1: (5.4, -11.4),
                              2: (7.85, -21.8), # (5.6, -20.8),
                              3: (3.6, -25.2),
                              4: (-5.7, -25.9),
                              5: (-5.5, -11.4)},
                          3: {0: (1.7, 2.6),
                              1: (5.4, -11.4),
                              2: (5.6, -20.8),
                              3: (4.25, -27.5), # (3.6, -25.2),
                              4: (-5.7, -25.9),
                              5: (-5.5, -11.4)},
                          4: {0: (-1.8, 2.3),
                              1: (5.4, -11.4),
                              2: (3.6, -25.2),
                              3: (3.6, -25.2),
                              4: (-7.75, -21.7), # (-5.3, -20.4),
                              5: (-5.5, -11.4)},
                          5: {0: (-1.8, 2.3),
                              1: (5.6, -6.5),
                              2: (5.6, -20.8),
                              3: (3.6, -25.2),
                              4: (-5.3, -20.4),
                              5: (-7.75, -7.5)}} #(-5.4, -6.8)}}
        prev_map = hold_coord_map.get(prev_rid, None)
        return prev_map.get(rid, None)
    
    def get_post_act_coord(self, rid, next_rid):
        """
        Gets the coordinate that an agent should hold at after completing an action.

        Args:
        - prev_rid: room id agent is coming from
        -      rid: room id agent is going to
        """
        # ToDo: CHANGE TO CORRECT VALUES
        post_coord_map = {0: {0: (0, 2.2),
                              1: (1.7, 2.6),
                              2: (1.7, 2.6),
                              3: (1.7, 2.6),
                              4: (-1.8, 2.3),
                              5: (-1.8, 2.3),
                              None: (1.7, 2.6)},
                          1: {0: (5.6, -6.5),
                              1: (5.6, -6.5),
                              2: (5.4, -11.4),
                              3: (5.4, -11.4),
                              4: (5.4, -11.4),
                              5: (5.6, -6.5),
                              None: (5.6, -6.5)},
                          2: {0: (5.6, -20.8),
                              1: (5.6, -20.8),
                              2: (5.6, -20.8),
                              3: (5.6, -20.8),
                              4: (3.6, -25.2),
                              5: (3.6, -25.2),
                              None: (3.6, -25.2)},
                          3: {0: (5.6, -20.8),
                              1: (5.6, -20.8),
                              2: (5.6, -20.8),
                              3: (3.6, -25.2),
                              4: (3.6, -25.2),
                              5: (3.6, -25.2),
                              None: (3.6, -25.2)},
                          4: {0: (-5.3, -20.4),
                              1: (-5.3, -20.4),
                              2: (-5.7, -25.9),
                              3: (-5.7, -25.9),
                              4: (-5.7, -25.9), 
                              5: (-5.3, -20.4),
                              None: (-5.3, -20.4)},
                          5: {0: (-5.4, -6.8),
                              1: (-5.4, -6.8),
                              2: (-5.5, -11.4),
                              3: (-5.5, -11.4),
                              4: (-5.5, -11.4),
                              5: (-5.4, -6.8),
                              None: (-5.4, -6.8)}} 
        curr_map = post_coord_map.get(rid, None)
        return curr_map.get(next_rid, None)

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
    
    def get_timed_plan(self, id_sequence, time_sequence, agents, tasks_stream):
        num_agents = len(agents)
        plan = []
        started_plan = False
        for task_id, arr_time in zip(id_sequence, time_sequence):
            # print(task_id)
            if self.is_agent_id(task_id, num_agents) and started_plan:
                return plan
            else:
                started_plan = True
            plan.append((arr_time, self.room_id(task_id, agents, tasks_stream)))
        return plan

    def update_plan_callback(self):
        def timed_positions_with_locks(action1, action2, action3):
            arrival_time1, rid1 = action1
            arrival_time2, rid2 = action2
            _, rid3 = action3
            nominal_timed_position = (arrival_time2, self.coord(rid2))

            # Always leave the room after dropping off
            post_act_timed_position = (arrival_time2 + 60 - self.lock_hold_time, self.get_post_act_coord(rid2, rid3))

            # Check if a hold needs to be invoked
            if self.room_graph[rid1][rid2] < arrival_time2 - arrival_time1:
                hold_timed_position = (arrival_time2 - self.lock_hold_time, self.get_hold_coord(rid1, rid2))
                return [hold_timed_position, nominal_timed_position, post_act_timed_position]
            
            return [nominal_timed_position, post_act_timed_position]
            
        def plan_to_positions(plan):
            timed_positions = []
            index = 2
            for previous_action, current_action in zip(plan[:-1], plan[1:]):
                if len(plan) > index:
                    next_action = plan[index]
                else:
                    next_action = None
                timed_positions += timed_positions_with_locks(previous_action, current_action, next_action)
                index += 1
            return timed_positions

        current_time_s = self.clock.now().nanoseconds * 1e-9
        if self.task_set_index < len(self.tasks_stream):
            next_tasks, next_batch_arrives = self.tasks_stream[self.task_set_index]
            # next_batch_arrives, next_plan = self.plans[self.task_set_index]
            # self.get_logger().info(f"Current time: {current_time_s - self.run_start_time_s}")
            if current_time_s - self.run_start_time_s > next_batch_arrives:
                self.get_logger().info(f"New tasks arrived at {next_batch_arrives}s")
                next_plan = self.solver.allocate_next_task_set(self.feedback)
                self.plans.append(next_plan)
                timed_position_lists = []
                for agt in next_plan['agt']:
                    # print(f"Agent's ids: {agt['id']}")
                    plan = self.get_timed_plan(agt['id'], agt['t'], self.agents, self.tasks_stream)
                    timed_position_lists.append(plan_to_positions(plan))

                self.timed_position_lists = timed_position_lists
                self.task_set_index += 1
                self.has_new_sequences = True

    def publish_goal_sequence_callback(self):
        def position_to_timed_pose(arr_time, point) -> TimedPose:
            msg = TimedPose()
            msg.time = arr_time
            msg.pose.position.x = float(point[0])
            msg.pose.position.y = float(point[1])
            return msg
    
        for (name, publisher), timed_position_list in zip(self.publishers_.items(), self.timed_position_lists):
            msg = TimedPoseList()
            msg.poses_with_time = [position_to_timed_pose(arr_time, position) for arr_time, position in timed_position_list]
            publisher.publish(msg)
            if self.has_new_sequences:
                self.get_logger().info(f"New goal sequence sent to {name}: {[pose for _, pose in timed_position_list]}")
        self.has_new_sequences = False

    def start_time_callback(self):
        msg = Time()
        msg.sec = math.floor(self.run_start_time.nanoseconds * 1e-9)
        msg.nanosec = round(self.run_start_time.nanoseconds % 1e9)
        self.start_time_publisher.publish(msg)

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
