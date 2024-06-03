import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator
import pickle

from geometry_msgs.msg import Pose, PoseArray

import math

from MRTASolver import MRTASolver
from MRTASolver.run_realistic_setting import load_weighted_graph, dictionary_to_matrix
from MRTASolver.create_randomized_inputs import load_config


class Dispatcher(Node):
    def __init__(self, name='', robot_list=[]):
        super().__init__(f'dispatcher_{name}')
        self.get_logger().info("Dispatcher starts!")

        self.publishers_ = { robot: self.create_publisher(PoseArray, f'/{robot}/goal_sequence', 1) for robot in robot_list }
        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.publish_goal_sequence_callback)
        self.name = name

        # MRTASolver arguments
        # file = 'benchmark/testcase.json'
        file = 'simulation/testcase.json'
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
        
        agents, tasks_stream = load_config(file)
        num_agents = len(agents)
        tot_tasks = sum([len(tasks) for tasks, _ in tasks_stream])
        num_aps = math.ceil(tot_tasks / num_agents) * 2 + 1
        aps_list = list(range(3, num_aps+1, 2))

        # room_dictionary = load_weighted_graph()
        # room_count, room_graph = dictionary_to_matrix(room_dictionary)
        with open('weighted_graph_hospital.pkl', 'rb') as file:
            data = pickle.load(file)
            room_count, room_graph = data
        
        self.solver = MRTASolver(solver, theory, agents, tasks_stream, room_graph, capacity, num_aps, fidelity, free_action_points, timeout, basename, default_deadline, aps_list, incremental, verbose)

        sol = self.solver.extract_model(self.solver.s)
        print(sol)
        # Get coordinates from solution

        coordinate_map = {0: (0, 2.2), 
                          1: (4.25, -27.5), 
                          2: (-7.75, -21.7), 
                          3: (7.85, -21.8), 
                          4: (7.9, -7.5), 
                          5: (-7.75, -7.5)}
        def coord(rid):
            # TODO: get x, y coordinates from room id
            # return rid, rid + 1
            return coordinate_map.get(rid, None)

        self.pose_lists = []
        for agt in sol['agt']:
            room_ids = [self.room_id(rid, agents, tasks_stream) for rid in agt['id']]
            self.pose_lists.append([coord(rid) for rid in room_ids])
        

        # self.pose_lists = [
        #         [(6.5, 11.5), (0.0, 2.0), (5.0, -9.0)],
        #         [(0.0, 2.0), (5.0, -9.0), (6.5, 11.5)]
        #         ]

        self.has_new_sequences = True

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

    def publish_goal_sequence_callback(self):
        if self.has_new_sequences:
            for (name, publisher), pose_list in zip(self.publishers_.items(), self.pose_lists):
                msg = PoseArray()
                msg.header.frame_id = "map"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.poses = [self.create_pose_from_point(pose) for pose in pose_list]
                publisher.publish(msg)
                self.has_new_sequences = False
                self.get_logger().info(f"New goal sequence sent to {name}: {msg}")

    def create_pose_from_point(self, point) -> Pose:
        msg = Pose()
        print(point)
        msg.position.x = float(point[0])
        msg.position.y = float(point[1])
        return msg

def main(args=None):
    rclpy.init(args=args)
    robot_list = ["robot1", "robot2"]
    dispatcher = Dispatcher(name='default', robot_list=robot_list)
    # executor = MultiThreadedExecutor()
    # executor.add_node(dispatcher)
    # executor.spin()
    rclpy.spin(dispatcher)
    dispatcher.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
