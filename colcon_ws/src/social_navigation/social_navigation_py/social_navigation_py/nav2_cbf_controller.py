import time
import networkx as nx
import rclpy
from rclpy.node import Node

from social_navigation_msgs.msg import HumanStates, RobotClosestObstacle, RobotCluster, PoseStampedPair, PathRequest, QueueRequest
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Bool
# from geometry_msgs.msg import Point
from tf2_ros.transform_broadcaster import TransformBroadcaster

# from cbf_controller import cbf_controller
# from .utils.cbf_obstacle_controller import cbf_controller
from .utils.cbf_multiagent_obstacle_controller import multi_cbf_controller
from .utils.cbf_obstacle_controller import cbf_controller
import numpy as np
# import jax.numpy as jnp

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class RobotController(Node):

    def __init__(self):
        super().__init__(f'robot_controller')
        # Get inputs
        self.declare_parameter('robot_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('robot_list', rclpy.Parameter.Type.STRING_ARRAY)
        robot_name_param, robot_list_param = self.get_parameter('robot_name'), self.get_parameter('robot_list')
        self.name, self.other_robots = robot_name_param.value, robot_list_param.value
        self.get_logger().info(f"Other robots: {self.other_robots}")

        # Define topic prefix
        if self.name != "":
            self.prefix = "/" + self.name
            self.priority = int(self.name[-1])
        else:
            self.prefix = ""
            self.priority = 1
        all_robots = [self.name] + self.other_robots
        self.robot_priorities = dict(zip(all_robots, [int(name[-1]) for name in all_robots])) # THIS WILL BREAK WITH MORE THAN 9 ROBOTS
        self.get_logger().info(f'Robot prefix: {self.prefix}')

        self.new_odom_name = f"{self.name}_new_odom"

        self.robot_state = np.array([10,10,0.0, 0.1]).reshape(-1,1)
        self.robot_pos = np.array([self.robot_state[0], self.robot_state[1]])

        # Obstacles
        self.num_obstacles = 12 # exact
        self.obstacle_states = np.zeros((2,self.num_obstacles))

        # Dynamic Obstacles
        self.num_other_robots = len(self.other_robots) # exact
        self.num_humans = 2 # upper bound
        # self.num_dynamic_obstacles = self.num_other_robots + self.num_humans
        self.num_dynamic_obstacles = self.num_humans
        self.other_robot_states = 100*np.ones((4,self.num_other_robots))
        self.other_robot_states_prev = np.zeros((4,self.num_other_robots))
        # self.other_robot_states_dot = np.zeros((2,self.num_other_robots))
        self.distance_to_other_robots = 100*np.ones((self.num_other_robots,1))
        self.connected_robots = []
        self.human_states = 100*np.ones((2,self.num_humans))
        self.human_states_prev = np.zeros((2,self.num_humans))
        self.human_states_dot = np.zeros((2,self.num_humans))

        # Parameters
        self.robot_radius = 0.15 # 0.12 # Previous values have been 0.2 and 0.18
        self.min_robot_to_robot = 4
        self.replan_count = 0
        self.print_count = 0
        self.timer_period_s = 0.05
        self.goal = np.array([0,0]).reshape(-1,1)
        self.leaving_room = int(self.name[-1])
        
        #Controller
        self.control_prev  = np.array([0.0,0.0])
        self.controller = cbf_controller( self.robot_state, self.num_dynamic_obstacles, self.num_obstacles, 1.0, 2.0)
        self.cluster_controller = None
        self.cluster_controller_active = False

        # Call once to initiate JAX JIT
        self.controller_id = 0
        dummy_time_step = self.timer_period_s
        if self.controller_id == 0:
            self.dynamic_obstacle_states_valid, self.human_states_valid, self.all_other_robot_states_valid = True, True, True
            self.update_dynamic_obstacles()
            self.get_logger().info("Calling initial cbf")
            self.controller.policy_cbf(self.robot_state, self.goal, self.robot_radius, self.dynamic_obstacle_states, self.dynamic_obstacle_states_dot, self.obstacle_states, dummy_time_step)
        elif self.controller_id == 1:
            self.controller.policy_nominal(self.robot_state, self.goal, dummy_time_step)

        # Subscribers
        self.humans_state_sub = self.create_subscription( HumanStates, '/human_states', self.human_state_callback, 10 )
        self.other_robot_state_sub = { robot: self.create_subscription( Odometry,  f'/{robot}/odom', self.make_other_robot_state_callback(i), 10) for i, robot in enumerate(self.other_robots) } 
        self.robot_state_subscriber = self.create_subscription( Odometry, self.prefix + '/odom', self.robot_state_callback, 10 )
        self.obstacle_subscriber = self.create_subscription( RobotClosestObstacle, self.prefix + '/robot_closest_obstacles', self.obstacle_callback, 10 )
        self.other_obstacle_subscriber = { robot: self.create_subscription( RobotClosestObstacle, f'{robot}/robot_closest_obstacles', self.make_other_robot_obstacle_callback(i, robot), 10 ) for i, robot in enumerate(self.other_robots)}
        self.plan_init_sub = self.create_subscription( Bool, '/planner_init', self.controller_plan_init_callback, 10 )
        self.goal_subscriber = self.create_subscription(PoseStampedPair, f'{self.prefix}/goal_location', self.new_goal_callback, 1)
        self.cluster_sub = { robot : self.create_subscription(RobotCluster, f'{robot}/cluster', self.make_cluster_callback(i, robot), 10) for i, robot in enumerate(self.other_robots)}
        self.activity_sub = self.create_subscription(Bool, f'{self.prefix}/active', self.status_callback, 10)
        self.remote_control = self.create_subscription(Twist, f'{self.prefix}/remote_control', self.remote_control_callback, 1)
        self.path_return_sub = self.create_subscription(Path, f'{self.prefix}/path_return', self.path_callback, 1)

        self.new_goal_poses = None
        self.human_states_valid = False
        self.all_other_robot_states_valid = False
        self.other_robot_states_valid = [False] * self.num_other_robots
        self.robot_state_valid = False
        self.path_active = False
        self.obstacles_valid = False
        self.pose_init = False
        self.goal_init = False
        self.initial_goal = True
        self.planner_init = False
        self.active = False
        self.path_end = None
        self.remote_controlled = False
        # self.replan = False
        self.error_count = 0
        self.h_min_dyn_obs_count = 0
        self.h_min_obs_count = 0
        self.finalized_each_cluster = [False] * self.num_other_robots
        self.finalized_cluster = False
        self.robot_cluster = (self.name, [self.name])
        self.other_robots_clusters = dict(zip(self.other_robots, [(robot, [robot], []) for robot in self.other_robots]))
        self.robots_active = dict(zip(all_robots, [False] * (self.num_other_robots + 1)))
        self.other_robot_obstacle_states = dict(zip(self.other_robots, np.zeros((2,self.num_obstacles))))
        self.other_robot_obstacles_valid = [False] * self.num_other_robots

        # Publishers
        self.robot_command_pub = self.create_publisher( Twist, self.prefix + '/cmd_vel', 10 )
        self.path_request_pub = self.create_publisher(PathRequest, '/path_request', 1)
        self.nav2_path_publisher = self.create_publisher( Path, self.prefix + '/plan', 1)
        self.robot_local_goal_pub = self.create_publisher( PoseStamped, self.prefix + '/local_goal', 1)
        self.robot_location_pub = self.create_publisher( PoseStamped, self.prefix + '/robot_location', 1)
        self.robot_new_odom_pub = self.create_publisher(Odometry, f"{self.prefix}/new_odom", 10)
        self.robot_cluster_pub = self.create_publisher(RobotCluster, f"{self.prefix}/cluster", 10)
        self.remote_control_pub = { robot: self.create_publisher(Twist, f'/{robot}/remote_control', 10) for robot in self.other_robots }
        # self.queue_request_pub = self.create_publisher(QueueRequest, "/queue_request", 1)

        # Frame broadcaster
        self.robot_tf_broadcaster = TransformBroadcaster(self)

        # Connect to planner
        self.navigator = BasicNavigator()
        self.path = Path()
        
        # Start controller
        self.time_prev = self.get_clock().now().nanoseconds
        self.get_logger().info(f"Current time: {self.time_prev}")
        self.controller_timer = self.create_timer(self.timer_period_s, self.run_controller)
        self.planner_timer = self.create_timer(self.timer_period_s*10, self.run_planner)
        # self.queue_request_timer = self.create_timer(self.timer_period_s*10, self.run_queue_request)
        self.nearby_robots_timer = self.create_timer(self.timer_period_s, self.nearby_robots)
        self.get_logger().info("User Controller is ONLINE")

    def update_dynamic_obstacles(self):
        if self.human_states_valid and self.all_other_robot_states_valid and self.dynamic_obstacle_states_valid:
            self.dynamic_obstacle_states = self.human_states
            self.dynamic_obstacle_states_prev = self.human_states_prev
            self.dynamic_obstacle_states_dot = self.human_states_dot
            # self.dynamic_obstacle_states = np.hstack((self.other_robot_states, self.human_states))
            # self.dynamic_obstacle_states_prev = np.hstack((self.other_robot_states_prev, self.human_states_prev))
            # self.dynamic_obstacle_states_dot = np.hstack((self.other_robot_states_dot, self.human_states_dot))
            self.dynamic_obstacle_states_valid = True
        
    def controller_plan_init_callback(self, msg):
        self.planner_init = msg.data

    def human_state_callback(self, msg):
        self.human_states_prev = np.copy(self.human_states)
        for i in range( len(msg.states) ):
            self.human_states[:,i] = np.array([ msg.states[i].position.x, msg.states[i].position.y ])
            self.human_states_dot[:,i] = np.array([ msg.velocities[i].linear.x, msg.velocities[i].linear.y ])
        self.human_states_valid = True
        self.update_dynamic_obstacles()

    def make_other_robot_state_callback(self, index):
        def other_robot_state_callback(msg):
            self.other_robot_states_prev = np.copy(self.other_robot_states)
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            linear = msg.twist.twist.linear
            other_robot_pos = np.array([position.x, position.y])
            self.other_robot_states[:,index] = np.array(  [position.x, position.y, 2 * np.arctan2( orientation.z, orientation.w ), linear.x]  )
            # self.other_robot_states[:,index] = other_robot_pos
            # self.get_logger().info(f"Calculated distance between {self.name} and {self.other_robots[index]}: {np.linalg.norm(other_robot_pos - self.robot_pos)}")
            # if self.print_count > 10:
            #     self.get_logger().info(f"Current robot position: {self.robot_pos}")
            #     self.get_logger().info(f"Difference between robots: {other_robot_pos.reshape((2,1)) - self.robot_pos.reshape((2,1))}")
            self.distance_to_other_robots[index] = np.linalg.norm(other_robot_pos.reshape((2,1)) - self.robot_pos.reshape((2,1)))
            # if self.print_count > 10:
            #     self.get_logger().info(f"Distance to other robot: {np.linalg.norm(other_robot_pos.reshape((2,1)) - self.robot_pos.reshape((2,1)))}")
            # velocity = msg.twist.twist.linear
            # theta = 2 * np.arctan2( orientation.z, orientation.w )
            # self.other_robot_states_dot[:, index] = np.array([velocity.x * np.cos(theta), velocity.x * np.sin(theta)])
            # self.other_robot_states_dot[:, index] = np.array([velocity.x, velocity.y])
            self.other_robot_states_valid[index] = True
            self.all_other_robot_states_valid = all(self.other_robot_states_valid)
            self.update_dynamic_obstacles()
        return other_robot_state_callback
    
    def make_cluster_callback(self, index, robot):
        def cluster_callback(msg):
            if self.name in msg.cluster and sorted(msg.cluster) == sorted(self.robot_cluster[1]) and self.robot_cluster[0] == msg.leader:
                self.finalized_each_cluster[index] = True
            elif self.name in msg.cluster:
                # self.get_logger().info(f"{self.name} has a mis-matched cluster or is not the leader.")
                self.finalized_each_cluster[index] = False
            elif self.name not in msg.cluster and self.other_robots[index] in self.robot_cluster[1]:
                # self.get_logger().info(f"{self.name}' cluster has not finalized")
                self.finalized_each_cluster[index] = False
            else:
                self.finalized_each_cluster[index] = True
            self.robots_active[robot] = bool(msg.active)
            self.other_robots_clusters[robot] = (msg.leader, msg.cluster, msg.neighbors)
            self.finalized_cluster = all(self.finalized_each_cluster)
        return cluster_callback
    
    def make_other_robot_obstacle_callback(self, index, robot):
        def other_robot_obstacle_callback(msg):
            obstacle_states_temp = 100*np.ones((2,self.num_obstacles))
            for i in range(min(msg.num_obstacles, self.num_obstacles)):
                obstacle_states_temp[:,i] = np.array([ msg.obstacle_locations[i].x, msg.obstacle_locations[i].y ])
            self.other_robot_obstacle_states[robot] = np.copy(obstacle_states_temp)
            self.other_robot_obstacles_valid[index] = True
        return other_robot_obstacle_callback

    def robot_state_callback(self, msg):
        self.robot_state = np.array(  [msg.pose.pose.position.x, msg.pose.pose.position.y, 2 * np.arctan2( msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ), msg.twist.twist.linear.x]  ).reshape(-1,1)
        self.robot_pos = np.array([self.robot_state[0], self.robot_state[1]])
        if self.print_count > 10:
            # print(f"Current robot state: {self.robot_state}")
            self.print_count = 0
        else:
            self.print_count += 1
        self.robot_state_valid = True

        # Publish secondary odometry that will be transformed to current robot position
        new_msg = Odometry()
        new_msg = msg
        new_msg.header.frame_id = 'map'
        new_msg.child_frame_id = self.new_odom_name
        self.robot_new_odom_pub.publish(new_msg)

        # Publish pose as transform for new odometry
        tf = TransformStamped()
        tf.header.frame_id = 'map'
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.child_frame_id = self.new_odom_name
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.robot_tf_broadcaster.sendTransform(tf)
        
    def obstacle_callback(self, msg):
        self.obstacle_states_temp = 100*np.ones((2,self.num_obstacles))
        for i in range(min(msg.num_obstacles, self.num_obstacles)):
            self.obstacle_states_temp[:,i] = np.array([ msg.obstacle_locations[i].x, msg.obstacle_locations[i].y ])
        self.obstacle_states = np.copy(self.obstacle_states_temp)
        self.obstacles_valid = True

    def new_goal_callback(self, msg):
        msg.current_waypoint.header.stamp = self.navigator.get_clock().now().to_msg()
        self.new_goal_poses = msg
        if (msg.current_waypoint.pose != msg.next_waypoint.pose):
            self.goal_init = False
            self.path_active = False
            self.initial_goal = msg.initialize
        self.get_logger().info(f"{self.name} received new goals: ({msg.current_waypoint.pose.position.x}, {msg.current_waypoint.pose.position.y})")

    def status_callback(self, msg):
        self.active = bool(msg.data)
        self.robots_active[self.name] = bool(msg.data)
        # if msg.data:
        #     self.get_logger().info(f'{self.name} is active')
        # else:
        #     self.get_logger().info(f'{self.name} is not active')

    def remote_control_callback(self, msg):
        self.robot_command_pub.publish(msg)
        self.remote_controlled = True
        self.initial_goal = True
        self.goal_init = False

    def path_callback(self, path):
        # Todo: Relocate to here everything from the run_planner logic that handles the newly received plan
        # path = msg.path

        # assert path is not None
        # current_end
        # close_x = abs(path.poses[0].pose.position.x - self.path.poses[-1].pose.position.x) < 0.05
        # new_path = abs(path.poses[-1].pose.position.y - goal_pose.pose.position.y) < 0.05 and abs(path.poses[-1].pose.position.y - goal_pose.pose.position.y) < 0.05
        # second_goal = abs(path.poses[0].pose.position.x - current_pose.pose.position.x) < 0.05
        # initial_pose_close_y = abs(path.poses[0].pose.position.y - current_pose.pose.position.y) < 0.05
        # goal_pose_close_x = abs(path.poses[-1].pose.position.x - goal_pose.pose.position.x) < 0.05
        # goal_pose_close_y = abs(path.poses[-1].pose.position.y - goal_pose.pose.position.y) < 0.05
        # assert initial_pose_close_x and initial_pose_close_y and goal_pose_close_x and goal_pose_close_y
        if not self.goal_init and self.new_goal_poses is not None and not self.remote_controlled:
            new_path = False
            new_start_pos = path.poses[0].pose.position
            close_to_pos_x = abs(self.robot_state[0,0] - new_start_pos.x) < 0.05
            close_to_pos_y = abs(self.robot_state[1,0] - new_start_pos.y) < 0.05
            starting_path = close_to_pos_x and close_to_pos_y
            if len(self.path.poses) > 0:
                current_end_pos = self.path.poses[-1].pose.position
                close_x = abs(current_end_pos.x - new_start_pos.x) < 0.05
                close_y = abs(current_end_pos.y - new_start_pos.y) < 0.05
                new_path = close_x and close_y
            if self.initial_goal and starting_path:
                # self.get_logger().info(f"Setting {self.name}'s initial path to {path}.")
                self.path = path
                self.path2 = path
                self.path_end = path.poses[-1]
                # self.nav2_path_publisher.publish(self.path)
                self.initial_goal = False
            elif new_path:
            # elif self.second_goal:
                # self.get_logger().info(f"Extending {self.name}'s path to {path}.")
                self.path.poses = self.path.poses + path.poses
                self.path2 = path
                self.path_end = path.poses[-1]
                # self.nav2_path_publisher.publish(self.path)
                self.path_active = True
                self.goal_init = True
                self.initial_goal = False
                # self.new_goal_poses = None
                if len(self.path.poses) < 2:
                    self.get_logger().info(f"SHORT PATH. POSSIBLE ERROR. {self.name}'s path is {path}.")

    def run_planner(self):
    # def run_controller(self):
        if self.print_count > 100:
            print(f"Planner init: {self.planner_init}")  
        if not self.planner_init:
            return
        
        if self.goal_init and len(self.path.poses) < 2:
            self.get_logger().info(f"SHORT PATH. POSSIBLE ERROR. {self.name}'s path is {self.path.poses}.")

        # # # Get current position and publish
        current_pose = PoseStamped()
        current_pose.header.frame_id = 'map'
        current_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        current_pose.pose.position.x = self.robot_state[0,0]
        current_pose.pose.position.y = self.robot_state[1,0]
        current_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
        current_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )
        # self.robot_location_pub.publish(current_pose)

        # # if self.print_count > 10:
        # #     self.navigator.setInitialPose(current_pose)


        # # set goal for first time
        # # if self.print_count > 100:
        # # print(f"goal_init: {self.goal_init}")
        # # print(f"new_goal_pose: {self.new_goal_poses}")
        # # if (not self.goal_init or self.replan_count > 100) and self.new_goal_pose is not None:
        # if not self.path_active:
        #     control = Twist()
        #     control.linear.x = 0.0
        #     control.angular.z = 0.0
        #     # self.get_logger().info(f"{self.name}'s path is inactive")
        #     self.robot_command_pub.publish(control)
        if not self.goal_init and self.new_goal_poses is not None:
        # if not self.goal_init and self.new_goal_poses is not None and self.name == self.robot_cluster[0]:
            # print(self.robot_state_valid, self.human_states_valid, self.obstacles_valid)
            if (self.robot_state_valid and self.human_states_valid and self.obstacles_valid):
                # if self.print_count > 100:
                #     print("Start planning...")
                goal = self.new_goal_poses.current_waypoint
                self.goal_pose = goal
                self.goal = np.array([ goal.pose.position.x, goal.pose.position.y ]).reshape(-1,1)
                # Get current position and publish
                # initial_pose = PoseStamped()
                # initial_pose.header.frame_id = 'map'
                # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                # initial_pose.pose.position.x = self.robot_state[0,0]
                # initial_pose.pose.position.y = self.robot_state[1,0]
                # initial_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
                # initial_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )
                # if self.print_count > 10:
                #     print(f"Initial pose: {current_pose.pose.position}")
                initial_fail = False
                if self.initial_goal:
                    # self.navigator.waitUntilNav2Active()
                    success = False
                    tries = 0
                    # while not success and tries < 10:
                    self.path_active = False
                    # self.get_logger().info(f"{self.name} trying for the {tries} time")
                    request_msg = PathRequest()
                    request_msg.id = self.name
                    request_msg.current_pose = current_pose
                    request_msg.goal_pose = self.goal_pose
                    self.path_request_pub.publish(request_msg)
                        # try:
                        #     self.navigator.setInitialPose(current_pose)
                        #     path = self.navigator.getPath(current_pose, self.goal_pose) # replace with naman's planner
                        #     # if self.print_count > 10:
                        #     #     print(f"Should be initial pose: {path.poses[0].pose.position}")
                        #     #     print(f"Requested initial pose: {initial_pose.pose.position}")
                        #     assert path is not None
                        #     initial_pose_close_x = abs(path.poses[0].pose.position.x - current_pose.pose.position.x) < 0.05
                        #     initial_pose_close_y = abs(path.poses[0].pose.position.y - current_pose.pose.position.y) < 0.05
                        #     goal_pose_close_x = abs(path.poses[-1].pose.position.x - self.goal_pose.pose.position.x) < 0.05
                        #     goal_pose_close_y = abs(path.poses[-1].pose.position.y - self.goal_pose.pose.position.y) < 0.05
                        #     assert initial_pose_close_x and initial_pose_close_y and goal_pose_close_x and goal_pose_close_y
                        #     # print(f"Updated {self.name}'s path")
                        #     # if close_x and close_y:
                        #     self.path = path
                        #     self.path2 = path
                        #     self.path_end = path.poses[-1]
                        #     # self.nav2_path_publisher.publish(self.path)
                        #     success = True
                        #     self.initial_goal = False
                        #     print(f"Updated {self.name}'s path")
                        #     return
                        # except Exception as e:
                        #     # print(f"{self.name} trying to find path again")
                        #     self.get_logger().info(f"{self.name} failing to plan due to {e}")
                        #     success = False
                        #     self.path_active = False
                        #     self.initial_goal = True
                        # tries += 1
                    # if tries >= 10 and not success:
                    #     self.get_logger().info(f"{self.name} tried to make initial plan too many times")
                    #     initial_fail = True
                    #     self.path_active = False
                    #     self.initial_goal = True
                    #     success = False

                # success = False
                # tries = 0
                # while not self.initial_goal and not success and not initial_fail and tries < 10:
                if not self.initial_goal:
                    start_pose = self.path_end
                    start_pose.header.frame_id = "map"
                    start_pose.header.stamp = self.get_clock().now().to_msg()

                    next_goal = self.new_goal_poses.next_waypoint

                    request_msg = PathRequest()
                    request_msg.id = self.name
                    request_msg.current_pose = start_pose
                    request_msg.goal_pose = next_goal
                    self.path_request_pub.publish(request_msg)

                #     self.get_logger().info(f"{self.name} trying for the {tries} time")
                #     self.path_active = False
                #     next_goal = self.new_goal_poses.next_waypoint
                #     # self.goal = np.array([ goal.pose.position.x, goal.pose.position.y ]).reshape(-1,1)
                #     # Get current position and publish
                #     # start_pose = self.new_goal_poses.current_waypoint
                #     start_pose = self.path_end
                #     start_pose.header.frame_id = "map"
                #     start_pose.header.stamp = self.get_clock().now().to_msg()
                #     # self.get_logger().info(f"Ending pose: {self.path_end} vs current waypoint: {self.new_goal_poses.current_waypoint}")
                #     # start_pose = PoseStamped()
                #     # start_pose.header.frame_id = 'map'
                #     # start_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                #     # start_pose.pose.position.x = self.robot_state[0,0]
                #     # start_pose.pose.position.y = self.robot_state[1,0]
                #     # start_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
                #     # start_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )
                #     # if self.print_count > 10:
                #     #     print(f"Initial pose: {current_pose.pose.position}")
                #     # self.navigator.clearGlobalCostmap()
                #     self.navigator.setInitialPose(start_pose)
                #     # self.navigator.waitUntilNav2Active()
                #     try:
                #         path = self.navigator.getPath(start_pose, next_goal) # replace with naman's planner
                #         # if self.print_count > 10:
                #         #     print(f"Should be initial pose: {path.poses[0].pose.position}")
                #         #     print(f"Requested initial pose: {initial_pose.pose.position}")
                #         assert path is not None
                #         initial_pose_close_x = abs(path.poses[0].pose.position.x - start_pose.pose.position.x) < 0.05
                #         initial_pose_close_y = abs(path.poses[0].pose.position.y - start_pose.pose.position.y) < 0.05
                #         goal_pose_close_x = abs(path.poses[-1].pose.position.x - next_goal.pose.position.x) < 0.05
                #         goal_pose_close_y = abs(path.poses[-1].pose.position.y - next_goal.pose.position.y) < 0.05
                #         assert initial_pose_close_x and initial_pose_close_y and goal_pose_close_x and goal_pose_close_y
                #         # if close_x and close_y:
                #         # self.path.poses = self.path2.poses + path.poses
                #         self.get_logger().info(f"{self.name} found valid path")
                #         self.path.poses = self.path.poses + path.poses
                #         self.path2 = path
                #         self.path_end = path.poses[-1]
                #         # self.nav2_path_publisher.publish(self.path)
                #         self.path_active = True
                #         self.goal_init = True
                #         success = True
                #         self.initial_goal = False
                #         print(f"Updated {self.name}'s path")
                #         return
                #     except Exception as e:
                #         self.get_logger().info(f"{self.name} failing to plan due to {e}")
                #         # print(f"Trying to find path again")
                #         success = False
                #         self.path_active = False
                #         self.goal_init = False
                #         # self.initial_goal = True
                #     tries += 1 
                # if tries >= 10 and not success:
                #     self.get_logger().info(f"{self.name} tried to get secondary plan too many times.")
                #     self.goal_init = False
                # elif success:
                #     self.new_goal_poses = None
            else:
                self.get_logger().info("Invalid world information")
    
    # def run_queue_request(self):
    #     # If agent is near room and next target is the room, publish queue request
    #     if abs(self.robot_state[0,0] - 7.9) < 3.0 and abs(self.robot_state[1,0] + 7.5) < 3.0 and self.leaving_room != 2:
    #         queue_request_msg = QueueRequest()
    #         queue_request_msg.room_id = 2
    #         queue_request_msg.robot_name = self.name
    #         self.queue_request_pub.publish(queue_request_msg)

    def run_controller(self):
        # if self.print_count > 100:
        #     print(f"Planner init: {self.planner_init}")  
        # if not self.planner_init:
        #     return

        # # Get current position and publish
        current_pose = PoseStamped()
        current_pose.header.frame_id = 'map'
        current_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        current_pose.pose.position.x = self.robot_state[0,0]
        current_pose.pose.position.y = self.robot_state[1,0]
        current_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
        current_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )
        self.robot_location_pub.publish(current_pose)

        # # if self.print_count > 10:
        # #     self.navigator.setInitialPose(current_pose)


        # # set goal for first time
        # # if self.print_count > 100:
        # # print(f"goal_init: {self.goal_init}")
        # # print(f"new_goal_pose: {self.new_goal_poses}")
        # # if (not self.goal_init or self.replan_count > 100) and self.new_goal_pose is not None:
        if not self.path_active:
            # print("Path is inactive")
            control = Twist()
            control.linear.x = 0.0
            control.angular.z = 0.0
            self.robot_command_pub.publish(control)
        # if not self.goal_init and self.new_goal_poses is not None and self.name == self.robot_cluster[0]:
        #     # print(self.robot_state_valid, self.human_states_valid, self.obstacles_valid)
        #     if (self.robot_state_valid and self.human_states_valid and self.obstacles_valid):
        #         # if self.print_count > 100:
        #         #     print("Start planning...")
        #         goal = self.new_goal_poses.current_waypoint
        #         self.goal_pose = goal
        #         self.goal = np.array([ goal.pose.position.x, goal.pose.position.y ]).reshape(-1,1)
        #         # Get current position and publish
        #         # initial_pose = PoseStamped()
        #         # initial_pose.header.frame_id = 'map'
        #         # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        #         # initial_pose.pose.position.x = self.robot_state[0,0]
        #         # initial_pose.pose.position.y = self.robot_state[1,0]
        #         # initial_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
        #         # initial_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )
        #         # if self.print_count > 10:
        #         #     print(f"Initial pose: {current_pose.pose.position}")
        #         initial_fail = False
        #         if self.initial_goal:
        #             # self.navigator.waitUntilNav2Active()
        #             success = False
        #             tries = 0
        #             while not success and tries < 100:
        #                 self.path_active = False
        #                 try:
        #                     self.navigator.setInitialPose(current_pose)
        #                     path = self.navigator.getPath(current_pose, self.goal_pose) # replace with naman's planner
        #                     # if self.print_count > 10:
        #                     #     print(f"Should be initial pose: {path.poses[0].pose.position}")
        #                     #     print(f"Requested initial pose: {initial_pose.pose.position}")
        #                     assert path is not None
        #                     initial_pose_close_x = abs(path.poses[0].pose.position.x - current_pose.pose.position.x) < 0.05
        #                     initial_pose_close_y = abs(path.poses[0].pose.position.y - current_pose.pose.position.y) < 0.05
        #                     goal_pose_close_x = abs(path.poses[-1].pose.position.x - self.goal_pose.pose.position.x) < 0.05
        #                     goal_pose_close_y = abs(path.poses[-1].pose.position.y - self.goal_pose.pose.position.y) < 0.05
        #                     assert initial_pose_close_x and initial_pose_close_y and goal_pose_close_x and goal_pose_close_y
        #                     # print(f"Updated {self.name}'s path")
        #                     # if close_x and close_y:
        #                     self.path = path
        #                     self.path2 = path
        #                     self.path_end = path.poses[-1]
        #                     # self.nav2_path_publisher.publish(self.path)
        #                     success = True
        #                     self.initial_goal = False
        #                     return
        #                 except Exception as e:
        #                     # print(f"Trying to find path again")
        #                     success = False
        #                     self.path_active = False
        #                 tries += 1
        #             if tries >= 100:
        #                 self.get_logger().info(f"{self.name} tried to make initial plan too many times")
        #                 initial_fail = True
        #                 self.path_active = False

        #         success = False
        #         tries = 0
        #         while not self.initial_goal and not success and not initial_fail and tries < 100:
        #             self.path_active = False
        #             next_goal = self.new_goal_poses.next_waypoint
        #             # self.goal = np.array([ goal.pose.position.x, goal.pose.position.y ]).reshape(-1,1)
        #             # Get current position and publish
        #             # start_pose = self.new_goal_poses.current_waypoint
        #             start_pose = self.path_end
        #             start_pose.header.frame_id = "map"
        #             start_pose.header.stamp = self.get_clock().now().to_msg()
        #             # self.get_logger().info(f"Ending pose: {self.path_end} vs current waypoint: {self.new_goal_poses.current_waypoint}")
        #             # start_pose = PoseStamped()
        #             # start_pose.header.frame_id = 'map'
        #             # start_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        #             # start_pose.pose.position.x = self.robot_state[0,0]
        #             # start_pose.pose.position.y = self.robot_state[1,0]
        #             # start_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
        #             # start_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )
        #             # if self.print_count > 10:
        #             #     print(f"Initial pose: {current_pose.pose.position}")
        #             # self.navigator.clearGlobalCostmap()
        #             self.navigator.setInitialPose(start_pose)
        #             # self.navigator.waitUntilNav2Active()
        #             try:
        #                 path = self.navigator.getPath(start_pose, next_goal) # replace with naman's planner
        #                 # if self.print_count > 10:
        #                 #     print(f"Should be initial pose: {path.poses[0].pose.position}")
        #                 #     print(f"Requested initial pose: {initial_pose.pose.position}")
        #                 assert path is not None
        #                 initial_pose_close_x = abs(path.poses[0].pose.position.x - start_pose.pose.position.x) < 0.05
        #                 initial_pose_close_y = abs(path.poses[0].pose.position.y - start_pose.pose.position.y) < 0.05
        #                 goal_pose_close_x = abs(path.poses[-1].pose.position.x - next_goal.pose.position.x) < 0.05
        #                 goal_pose_close_y = abs(path.poses[-1].pose.position.y - next_goal.pose.position.y) < 0.05
        #                 assert initial_pose_close_x and initial_pose_close_y and goal_pose_close_x and goal_pose_close_y
        #                 # print(f"Updated {self.name}'s path")
        #                 # if close_x and close_y:
        #                 self.path.poses = self.path2.poses + path.poses
        #                 self.path2 = path
        #                 self.path_end = path.poses[-1]
        #                 # self.nav2_path_publisher.publish(self.path)
        #                 self.path_active = True
        #                 self.goal_init = True
        #                 success = True
        #                 self.new_goal_poses = None
        #                 self.initial_goal = False
        #                 return
        #             except Exception as e:
        #                 # print(f"Trying to find path again")
        #                 success = False
        #                 self.path_active = False
        #             tries += 1 
        #         if tries >= 100:
        #             self.get_logger().info(f"{self.name} tried to get secondary plan too many times")
        #     else:
        #         self.get_logger().info("Invalid world information")
            
        # Get next waypoint to follow from given path. It finds the next waypoint that is atleast 1 m away and removes the waypoints occurring before this 1 m point
        if (self.path_active and len(self.path.poses) > 0 and (self.robot_state_valid and self.human_states_valid and self.obstacles_valid)):
            # Select closest waypoint from received path
            # self.get_logger().info(f"Controlling {self.name}")
            assert np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]) is not None
            goal = np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]).reshape(-1,1)
            while (np.linalg.norm(goal[:,0] - self.robot_state[0:2,0])<0.5):#0.8
                if len(self.path.poses)>1:
                    self.path.poses = self.path.poses[1:]
                    assert np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]) is not None
                    goal = np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]).reshape(-1,1)
                else:
                    # self.get_logger().info(f"End of {self.name}'s path.")
                    break

            # Publish path for visualization (no other use)
            self.nav2_path_publisher.publish(self.path)
            goal_msg = PoseStamped()
            goal_msg.pose.position.x = goal[0,0]
            goal_msg.pose.position.y = goal[1,0]
            goal_msg.pose.position.z = 0.0
            if len(self.path.poses)>1:
                theta = np.arctan2( self.path.poses[1].pose.position.y - self.path.poses[0].pose.position.y, self.path.poses[1].pose.position.x - self.path.poses[0].pose.position.x )
                goal_msg.pose.orientation.z = np.sin( theta/2 )
                goal_msg.pose.orientation.w = np.cos( theta/2 )
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = self.navigator.get_clock().now().to_msg()
            self.robot_local_goal_pub.publish( goal_msg )
            # self.get_logger().info(f"Goal: {goal}")
            
            t_new = self.get_clock().now().nanoseconds
            dt = (t_new - self.time_prev)/10**9
            control = Twist()
            if not self.finalized_cluster:
                # self.get_logger().info(f"{self.name}'s cluster has not been finalized")
                control.linear.x = 0.0
                control.angular.z = 0.0
                self.robot_command_pub.publish(control)
            # elif (self.finalized_cluster and self.name != self.robot_cluster[0]):
            #     # self.get_logger().info(f"{self.name} is not the leader.")
            #     control.linear.x = 0.0
            #     control.angular.z = 0.0
            elif (self.finalized_cluster and self.name == self.robot_cluster[0] and all(self.other_robot_obstacles_valid)):
                # try:                
                other_robot_states_list = []
                other_robots = []
                num_other_robots = 0
                for i, robot in enumerate(self.other_robots):
                    if robot in self.robot_cluster[1]:
                        other_robots.append(robot)
                        other_robot_states_list.append(np.array(self.other_robot_states[:,i]).reshape(-1,1))
                        num_other_robots += 1
                if len(other_robot_states_list) > 0:
                    other_robot_states = np.vstack(other_robot_states_list)
                    # self.get_logger().info(f'Other robot states: {other_robot_states}')
                    # self.get_logger().info(f"Size of robot state: {self.robot_state.shape} and size of other robot states: {other_robot_states.shape}")
                    complete_state = np.vstack((self.robot_state, other_robot_states))
                    if not self.cluster_controller_active or self.cluster_controller is None:
                        # TODO: Change obstacles to include obstacles for all robots
                        # self.cluster_controller = cbf_controller(complete_state, self.num_dynamic_obstacles, self.num_obstacles * num_other_robots, 1.0, 2.0)
                        # print("Initializing cluster controller")
                        self.cluster_controller = multi_cbf_controller(complete_state, self.num_dynamic_obstacles*(num_other_robots+1), self.num_obstacles, 1.0, 2.0)
                        self.cluster_controller_active = True
                        # speed, omega, h_dyn_obs_min, h_obs_min = 0.0, 0.0, 0, 0
                    if self.controller_id == 0:
                        # start_time = time.time()
                        other_obstacles = []
                        for robot in other_robots:
                            other_obstacles.append(self.other_robot_obstacle_states[robot])
                        speed, omega, h_dyn_obs_min, h_obs_min = self.cluster_controller.policy_cbf( self.robot_state, other_robot_states, goal, self.robot_radius, self.dynamic_obstacle_states, self.dynamic_obstacle_states_dot, self.obstacle_states, other_obstacles, dt , slow = not self.active)
                        # self.get_logger().info(f"Time to calculate policy: {time.time() - start_time}")
                    for i, robot in enumerate(other_robots):
                        # print(f"Calculated control for {robot}: {speed[i+1]}, {omega[i+1]}")
                        control = Twist()
                        control.linear.x = speed[i+1][0]
                        control.angular.z = omega[i+1][0]
                        # if self.finalized_cluster and self.name == self.robot_cluster[0] and self.robots_active[robot] and robot != self.name:
                        if self.finalized_cluster and self.name == self.robot_cluster[0] and robot != self.name:
                            self.remote_control_pub[robot].publish(control)
                else:
                    other_robot_states = np.empty((0,1))
                    if self.controller_id == 0:
                        start_time = time.time()
                        speed, omega, h_dyn_obs_min, h_obs_min = self.controller.policy_cbf( self.robot_state, goal, self.robot_radius, self.dynamic_obstacle_states, self.dynamic_obstacle_states_dot, self.obstacle_states, dt , slow = not self.active)
                        if time.time() - start_time > 0.1:
                            self.get_logger().info(f"Time to calculate {self.name}'s policy was large: {time.time() - start_time}")
                    elif self.controller_id == 1:
                        speed, omega, h_dyn_obs_min, h_obs_min = self.controller.policy_nominal( self.robot_state, goal, dt )
                    speed = [[speed]]
                    omega = [[omega]]
                
                # Check if any collision constraints violated
                if h_dyn_obs_min < -0.01:
                    self.h_min_dyn_obs_count += 1
                    self.get_logger().info(f"dynamic obstacle violate: {self.h_min_dyn_obs_count}")
                if h_obs_min < -0.01:
                    self.h_min_obs_count += 1
                    self.get_logger().info(f"obstacle violate: {self.h_min_obs_count}")
                # except Exception as e:
                #     speed = 0.0
                #     omega = 0.0
                #     self.error_count = self.error_count + 1
                #     print(f"ERROR ******************************** count: {self.error_count} {e}")

                control.linear.x = speed[0][0]
                control.angular.z = omega[0][0]
                self.robot_command_pub.publish(control)
                # self.get_logger().info(f"{self.name} publishing control of {control.linear.x}, {control.angular.z}")
            # else:
            #     self.get_logger().info(f"{self.name} not the leader")
            self.time_prev = t_new
        else:
            # if not self.path_active:
            #     self.get_logger().info(f"Unable to control {self.name} because path is inactive")
            # elif not (self.robot_state_valid and self.human_states_valid and self.obstacles_valid):
            #     self.get_logger().info(f"Unable to control {self.name} because state is invalid")
            control = Twist()
            control.linear.x = 0.0
            control.angular.z = 0.0
            self.robot_command_pub.publish(control)

    def nearby_robots(self):
        # nearby = lambda i: True if self.distance_to_other_robots[i] < self.min_robot_to_robot else False
        neighbors = []
        for i in range(len(self.other_robots)):
            # self.get_logger().info(f"Distance to other robot: {self.distance_to_other_robots[i]}")
            if self.distance_to_other_robots[i] < self.min_robot_to_robot:
                neighbors.append(self.other_robots[i])
        # self.get_logger().info(f"Positions of other robots: {self.other_robot_states}")
        # self.connected_robots = [robot for i, robot in enumerate(self.other_robots) if nearby(i)]
        # self.get_logger().info(f"Robot {self.name} close to {connected_robots}")
        # cluster_set = set()
        # # self.get_logger().info(f"{self.name}'s neighborhood: {[self.other_robots[i] for i in neighbors]}")
        # for i in neighbors:
        #     cluster_set.update(self.other_robots_clusters[i][1])
        # cluster_set.update([self.name])
        # cluster = list(cluster_set)
        G = nx.Graph()
        edges = [(self.name, robot) for robot in neighbors]
        G.add_edges_from(edges)
        for other_robot in self.other_robots:
            edges = [(other_robot, other_robot_neighbor) for other_robot_neighbor in self.other_robots_clusters[other_robot][2]]
            G.add_edges_from(edges)
        DG = G.to_directed()
        TC = nx.transitive_closure(DG)
        def get_agents_cluster(graph, agent):
            # Find all weakly connected components
            components = list(nx.connected_components(graph))
            # Search for the component containing the specified agent
            for component in components:
                if agent in component:
                    # print(f"{self.name}'s cluster: {list(component)}")
                    return list(component)
            return [agent]
        cluster = get_agents_cluster(TC.to_undirected(), self.name)
        # self.get_logger().info(f"Active robots: {self.robots_active}")
        active_cluster = [robot for robot in cluster if self.robots_active[robot] is True]
        # active_cluster = cluster
        cluster_priorities = [self.robot_priorities[robot] for robot in active_cluster]
        if len(active_cluster) > 0:
            # self.get_logger().info(f"{self.name}'s active cluster: {active_cluster}")
            leader = active_cluster[np.argmax(cluster_priorities)]
        else:
            # self.get_logger().info(f"No other active robots in {self.name}'s cluster.")
            leader = self.name
        if leader != self.robot_cluster[0] or sorted(cluster) != sorted(self.robot_cluster[1]):
            self.cluster_controller_active = False
        self.remote_controlled = leader != self.name
        self.robot_cluster = (leader, cluster, neighbors)

        current_pose = PoseStamped()
        current_pose.header.frame_id = 'map'
        current_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        current_pose.pose.position.x = self.robot_state[0,0]
        current_pose.pose.position.y = self.robot_state[1,0]
        current_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
        current_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )

        msg = RobotCluster()
        msg.leader = leader
        msg.active = self.active
        msg.cluster = cluster
        msg.neighbors = neighbors
        self.robot_cluster_pub.publish(msg)
        # self.get_logger().info(f"{self.name}'s cluster: {self.robot_cluster[1]} with leader {leader}.")
    

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    # robot_controller.controller = cbf_controller(np.array([3.0, 4.0, 0.0, 0.1, 4.0, 5.0, 0.2, 0.1]).reshape(-1,1), 0, 1, dynamic_alpha1=1.0, dynamic_alpha2=2.0)
    # robot_goal = np.array([4.0, 5.0]).reshape(-1,1)
    # other_robot_states = np.array([4.0, 5.0, 0.2, 0.1]).reshape(-1,1)
    # output = robot_controller.controller.robot.nominal_controller(robot_goal, other_robot_states, k_x = cbf_controller.k_x, k_v = cbf_controller.k_v )
    # print(output)
    rclpy.spin(robot_controller)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
