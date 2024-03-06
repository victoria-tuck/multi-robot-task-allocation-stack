import rclpy
from rclpy.node import Node

from social_navigation_msgs.msg import HumanStates, RobotClosestObstacle
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
# from geometry_msgs.msg import Point

# from cbf_controller import cbf_controller
from .utils.mppi_obstacle_controller import MPPI_FORESEE
import numpy as np
# import jax.numpy as jnp

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        # 0: cbf controller
        # 1: nominal controller
        self.controller_id = 0#1

        # Variables
        self.num_humans = 20 # upper bound
        self.num_obstacles = 12 # exact
        self.robot_state = np.array([10,10,0.0, 0.1]).reshape(-1,1)
        self.obstacle_states = np.zeros((2,self.num_obstacles))
        self.human_states = 100*np.ones((2,self.num_humans))
        self.human_states_prev = np.zeros((2,self.num_humans))
        # self.t_human = self.get_clock().now().nanoseconds
        self.human_states_dot = np.zeros((2,self.num_humans)) 
        self.robot_radius = 0.18

        self.timer_period = 0.05#0.05 # seconds
        self.time_step = self.timer_period
        self.goal = np.array([0,0]).reshape(-1,1)
        
        #Controller
        self.control_prev = np.array([0.0,0.0])

        # MPPI related parameters
        self.use_GPU = False
        self.human_noise_cov = 4.0
        self.human_noise_mean = 0
        self.human_localization_noise = 0.05
        self.dt = 0.05 
        self.T = 50 # simulation steps
        self.control_bound = 4
        self.kx = 4.0
        self.sensing_radius = 2
        self.factor = 2.0 # no of standard deviations
        self.choice = 0
        self.samples = 500
        self.horizon = 40
        self.human_ci_alpha = 0.05

        # cost terms
        self.human_nominal_speed = np.array([3.0,0]).reshape(-1,1)
        self.human_repulsion_gain = 2.0
        self.costs_lambda = 0.03 
        self.cost_goal_coeff = 0.2 
        self.cost_safety_coeff = 10.0 

        u_guess = np.zeros((self.horizon, 2))

        self.controller = MPPI_FORESEE(horizon=self.horizon, samples=self.samples, input_size=2, dt=self.dt, sensing_radius=self.sensing_radius, human_noise_cov=self.human_noise_cov, std_factor=self.factor, control_bound=self.control_bound, u_guess=u_guess, human_nominal_speed=self.human_nominal_speed, human_repulsion_gain=self.human_repulsion_gain, costs_lambda=self.costs_lambda, cost_goal_coeff=self.cost_goal_coeff, cost_safety_coeff=self.cost_safety_coeff, num_humans=self.num_humans, num_obstacles = self.num_obstacles, use_GPU=self.use_GPU)
        # Call once to initiate JAX JIT
        robot_sampled_states, robot_chosen_states, robot_action, human_mus_traj, human_covs_traj = self.controller.policy_mppi(self.robot_state, self.goal, self.human_stats, self.human_localization_noise * jnp.ones((2,self.num_humans)), self.obstacle_states)
        
        # Subscribers
        self.humans_state_sub = self.create_subscription( HumanStates, '/human_states', self.human_state_callback, 10 )
        self.robot_state_callback = self.create_subscription( Odometry, '/odom', self.robot_state_callback, 10 )        
        self.goal_pose_subscriber = self.create_subscription( PoseStamped, 'goal_pose_custom', self.robot_goal_callback, 10 )
        self.obstacle_subscriber = self.create_subscription( RobotClosestObstacle, '/robot_closest_obstacles', self.obstacle_callback, 10 )
        self.plan_init_sub = self.create_subscription( Bool, '/planner_init', self.controller_plan_init_callback, 10 )
        self.human_states_valid = False
        self.robot_state_valid = False
        self.path_active = False
        self.obstacles_valid = False
        self.pose_init = False
        self.goal_init = False
        self.planner_init = False
        self.error_count = 0
        self.h_min_human_count = 0
        self.h_min_obs_count = 0
        # self.robot_nearest_obstacle_sub = self.create_sunscription(  )

        # Publishers
        self.robot_command_pub = self.create_publisher( Twist, '/cmd_vel', 10 )
        self.nav2_path_publisher = self.create_publisher( Path, '/plan', 1)
        self.robot_local_goal_pub = self.create_publisher( PoseStamped, '/local_goal', 1)
        
        # Planner
        self.navigator = BasicNavigator()
        self.path = Path() 
        self.path_waypoint_index = 0

        
        self.get_logger().info("User Controller is ONLINE")
        self.timer = self.create_timer(self.timer_period, self.controller_callback)
        
        self.time_prev = self.get_clock().now().nanoseconds
        
        print(f"time: {self.time_prev}")
        # exit()


        # Goal
        self.robot_goal = np.array([1,1]).reshape(-1,1)
        
    def controller_plan_init_callback(self, msg):
        self.planner_init = msg.data

    def human_state_callback(self, msg):
        self.human_states_prev = np.copy(self.human_states)
        for i in range( len(msg.states) ):#:self.num_humans):
            self.human_states[:,i] = np.array([ msg.states[i].position.x, msg.states[i].position.y ])
            self.human_states_dot[:,i] = np.array([ msg.velocities[i].linear.x, msg.velocities[i].linear.y ])
        self.human_states_valid = True

    def robot_state_callback(self, msg):
        self.robot_state = np.array(  [msg.pose.pose.position.x, msg.pose.pose.position.y, 2 * np.arctan2( msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ), msg.twist.twist.linear.x]  ).reshape(-1,1)
        self.robot_state_valid = True
        
    def obstacle_callback(self, msg):
        # self.num_obstacles = msg.num_obstacles
        self.obstacle_states_temp = 100*np.ones((2,self.num_obstacles))
        for i in range(min(msg.num_obstacles, self.num_obstacles)):
            self.obstacle_states_temp[:,i] = np.array([ msg.obstacle_locations[i].x, msg.obstacle_locations[i].y ])            
        self.obstacle_states = np.copy(self.obstacle_states_temp)
        self.obstacles_valid = True
        
    def wrap_angle(self,theta):
        return np.arctan2( np.sin(theta), np.cos(theta) )

    def controller_callback(self):     
        if not self.planner_init:
            return
        
        # set goal for first time
        if not self.goal_init:
            if (self.robot_state_valid and self.human_states_valid and self.obstacles_valid):
                success = False
                while not success:
                    msg = PoseStamped()
                    msg.header.frame_id = "map"
                    msg.header.stamp = self.navigator.get_clock().now().to_msg()
                    msg.pose.position.x = 5.0   #6.5#9.0
                    msg.pose.position.y = -18.0   #11.5#13.5
                    msg.pose.position.z = 0.01
                    msg.pose.orientation.x = 0.0
                    msg.pose.orientation.y = 0.0
                    msg.pose.orientation.z = 0.0
                    msg.pose.orientation.w = 1.0
                    self.goal_pose = msg
                    self.goal = np.array([ msg.pose.position.x, msg.pose.position.y ]).reshape(-1,1)
                    initial_pose = PoseStamped()
                    initial_pose.header.frame_id = 'map'
                    initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                    initial_pose.pose.position.x = self.robot_state[0,0]
                    initial_pose.pose.position.y = self.robot_state[1,0]
                    initial_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
                    initial_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )
                    try:
                        self.path = self.navigator.getPath(initial_pose, self.goal_pose) # replace with naman's planner
                        self.nav2_path_publisher.publish(self.path)
                        self.path_waypoint_index = 0
                        self.path_active = True
                        self.goal_init = True
                        success = True
                        return
                    except Exception as e:
                        print(f"Trying to find path again")
                        success = False
            
        # Get next waypoint to follow from given path. It finds the next waypoint that is atleast 1 m away and removes the waypoints occurring before this 1 m point
        if (self.path_active and (self.robot_state_valid and self.human_states_valid and self.obstacles_valid)):
            # Select closest waypoint from received path
            goal = np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]).reshape(-1,1)
            while (np.linalg.norm(goal[:,0] - self.robot_state[0:2,0])<1.0):#0.8
                if len(self.path.poses)>1:
                    self.path.poses = self.path.poses[1:]
                    goal = np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]).reshape(-1,1)
                else:
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
            
            t_new = self.get_clock().now().nanoseconds
            dt = (t_new - self.time_prev)/10**9
            # self.get_logger().info(f"dt: {dt}")
            try:                
                robot_sampled_states, robot_chosen_states, robot_action, human_mus_traj, human_covs_traj = self.controller.policy_mppi(self.robot_state, goal, self.human_states, self.human_localization_noise * np.ones((2,self.num_humans)), self.human_states_dot, self.obstacle_states)
                speed, omega = robot_action[0,0], robot_action[1,0]
                # speed, omega, h_human_min, h_obs_min = self.controller.policy_nominal( self.robot_state, goal, dt )
                
                # # Check if any collision constraints violated
                # if h_human_min < -0.01:
                #     self.h_min_human_count += 1
                #     self.get_logger().info(f"human violate: {self.h_min_human_count}")
                # if h_obs_min < -0.01:
                #     self.h_min_obs_count = 0
                #     self.get_logger().info(f"obstacle violate: {self.h_min_obs_count}")
            except Exception as e:
                speed = self.control_prev[0]  #0.0
                omega = self.control_prev[1]  #0.0
                self.error_count = self.error_count + 1
                print(f"ERROR ******************************** count: {self.error_count} {e}")
                
            self.time_prev = t_new
            # print(f"CBF speed: {speed}, omega: {omega}, dt:{dt}")

            ############## Publish Control Input ###################
            control = Twist()
            control.linear.x = speed
            control.angular.z = omega
            self.robot_command_pub.publish(control)

    def robot_goal_callback(self, msg):
        print(f"Received new goal")
        self.goal_pose = msg
        self.goal = np.array([ msg.pose.position.x, msg.pose.position.y ]).reshape(-1,1)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.robot_state[0,0]
        initial_pose.pose.position.y = self.robot_state[1,0]
        initial_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
        initial_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )

        self.path = self.navigator.getPath(initial_pose, self.goal_pose) # replace with naman's planner

        self.nav2_path_publisher.publish(self.path)
        self.path_waypoint_index = 0
        self.path_active = True
        # Now do control        
        
    
def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
