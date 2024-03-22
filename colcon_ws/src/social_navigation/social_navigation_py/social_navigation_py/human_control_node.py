import rclpy
from rclpy.node import Node

from social_navigation_msgs.msg import HumanStates, RobotClosestObstacle
from geometry_msgs.msg import Twist, Vector3, Point
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import yaml

# from cbf_controller import cbf_controller
from .utils.mppi_obstacle_controller import MPPI_FORESEE
import numpy as np
import jax.numpy as jnp

from .socialforce import socialforce

class HumanController(Node):

    def __init__(self):
        super().__init__('human_controller')

        self.declare_parameter('num_humans', 10)
        self.declare_parameter('timer_period', 0.05)
        self.declare_parameter('human_config_file', 'humans_waypoint_config.yaml')

        self.num_humans =  self.get_parameter('num_humans').get_parameter_value().integer_value#10
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value #  0.05
        self.human_config_file = self.get_parameter('human_config_file').get_parameter_value().string_value  #'humans_waypoint_config.yaml'


        # Subscribers
        self.humans_state_sub = self.create_subscription( HumanStates, '/human_states', self.human_state_callback, 10 )
        self.robot_state_callback = self.create_subscription( Odometry, '/odom', self.robot_state_callback, 10 )        
        self.human_obstacle_subscriber = self.create_subscription( RobotClosestObstacle, '/human_closest_obstacles', self.obstacle_callback, 10 )
        # self.human_single_obstacle_subscriber = self.create_subscription( RobotClosestObstacle, '/human_single_closest_obstacles', self.single_obstacle_callback, 10 )

        self.robot_state_valid = False
        self.human_states_valid = False
        self.obstacles_valid = False
        self.goals_valid = False

        self.human_states = np.zeros((2,self.num_humans))
        self.human_states_dot = np.zeros((2,self.num_humans))
        self.robot_state = np.zeros((3,1))
        self.robot_state_dot = np.zeros((3,1))
        self.human_goals = np.zeros((2,self.num_humans))

        self.humans_socialforce = socialforce.Simulator( delta_t = self.timer_period )

        

        self.human_command_pubs = []
        for i in range(self.num_humans):
            self.human_command_pubs.append( self.create_publisher( Twist, '/actor'+str(i+1)+'/cmd_vel', 1) )
        
        self.controller_timer = self.create_timer(self.timer_period, self.controller_callback)
        self.decide_waypoint_timer = self.create_timer(self.timer_period, self.decide_next_waypoint)

        # Read waypoints for humans. should match with naming in world files
        self.waypoints = [] # list(2 x N)
        self.current_waypoint_ids = [0]*self.num_humans # (N,)

        # read waypoints from the yaml file
        self.dist_threshold = 0.3
        with open(self.human_config_file, 'r') as file:
            f = yaml.safe_load(file)
        for actor in f:
            self.waypoints.append( np.asarray(f[actor]).T )
        if len(self.waypoints) != self.num_humans:
            self.get_logger.error("Num of waypoints do not match num of humans")
            exit()
        # print(f"waypoints: {self.waypoints}")
        self.decide_next_waypoint()
        # print(f"goals: {self.human_goals}")

        
        self.socialforce = socialforce.Simulator( delta_t = self.timer_period )                                                                                  
        self.MAX_SPEED_MULTIPLIER = 1.3
        initial_speed = 1.0 #1.0
        self.initial_speeds = jnp.ones((self.num_humans)) * initial_speed
        self.max_speeds = self.MAX_SPEED_MULTIPLIER * self.initial_speeds

    def __del__(self):
        for i in range(self.num_humans):
            # self.get_logger().info("Human Callback")
            vel = Twist()
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.human_command_pubs[i].publish(vel)

    def decide_next_waypoint(self):

        if not self.human_states_valid:
            return
        human_goals = []
        for i in range(self.num_humans):
            pos = self.human_states[0:2,i]
            waypoint = self.waypoints[i][:,self.current_waypoint_ids[i]]
            dist_to_waypoint = np.linalg.norm( pos - waypoint  )
            if dist_to_waypoint < self.dist_threshold:
                if self.current_waypoint_ids[i] < (self.waypoints[i].shape[1]-1):
                    self.current_waypoint_ids[i] = self.current_waypoint_ids[i] + 1
                else:
                    self.current_waypoint_ids[i] = 0
            if i==0:
                human_goals = self.waypoints[i][:,[self.current_waypoint_ids[i]]]
            else:
                human_goals = np.append( human_goals, self.waypoints[i][:,[self.current_waypoint_ids[i]]], axis=1 )
        self.human_goals = human_goals         
        # print(f"goals: {self.human_goals}")
        self.goals_valid = True

    def normalize(self,vec):
        vec_norm = np.linalg.norm( vec )
        if vec_norm > 0.1:
            return vec / vec_norm
        else:
            return np.ones((2,1))/np.sqrt(2)
        
    def human_state_callback(self, msg):
        self.human_states_prev = np.copy(self.human_states)
        for i in range( len(msg.states) ):#:self.num_humans):
            self.human_states[:,i] = np.array([ msg.states[i].position.x, msg.states[i].position.y ])
            self.human_states_dot[:,i] = np.array([ msg.velocities[i].linear.x, msg.velocities[i].linear.y ])
        self.human_states_valid = True
        # self.get_logger().info("Human Callback")

    def robot_state_callback(self, msg):
        self.robot_state = np.array(  [msg.pose.pose.position.x, msg.pose.pose.position.y, 2 * np.arctan2( msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ), msg.twist.twist.linear.x]  ).reshape(-1,1)
        self.robot_state_valid = True
        # self.get_logger().info("Robot Callback")

    def obstacle_callback(self, msg):
        # self.num_obstacles = msg.num_obstacles
        self.obstacle_states_temp = 100*np.ones((2,self.num_obstacles))
        for i in range(min(msg.num_obstacles, self.num_obstacles)):
            self.obstacle_states_temp[:,i] = np.array([ msg.obstacle_locations[i].x, msg.obstacle_locations[i].y ])            
        self.obstacle_states = np.copy(self.obstacle_states_temp)
        self.obstacles_valid = True

    def controller_callback(self):

        if not ( self.human_states_valid and self.robot_state_valid and self.goals_valid ):
            return

        human_states = np.copy(self.human_states)
        human_states_dot = np.copy(self.human_states_dot)
        # Set state here

        socialforce_initial_state = np.append( np.append( human_states.T, human_states_dot.T , axis = 1 ), self.human_goals.T, axis=1   )
        # robot_goal = self.robot_state[0:2] + self.normalize(self.robot_state_dot[0:2]) * 10 # 10 meters away in the direction of current velocity
        # robot_social_state = np.array([ self.robot_state[0,0], self.robot_state[1,0], self.robot_states_dot[0,0], self.robot_states_dot[1,0], robot_goal[0,0], self.robot_goal[1,0]])
        # humans_socialforce.state[-1,0:6] = robot_social_state
        

        tau = 0.5
        tau = tau * jnp.ones(socialforce_initial_state.shape[0])
        social_state = jnp.concatenate((socialforce_initial_state, jnp.expand_dims(tau, -1)), axis=-1)

        # print(f"social_state: {social_state}")

        F, state = self.socialforce.step(social_state, self.initial_speeds, self.max_speeds, self.timer_period)
        
        human_controls = state[:,2:4].copy().T
        # print(f"human_controls: {human_controls}")

        for i in range(self.num_humans):
            # self.get_logger().info("Human Callback")
            vel = Twist()
            vel.linear.x = float(human_controls[0,i])
            vel.linear.y = float(human_controls[1,i])
            vel.angular.z = 0.0
            self.human_command_pubs[i].publish(vel)

        
    
def main(args=None):
    rclpy.init(args=args)
    human_controller = HumanController()
    rclpy.spin(human_controller)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

