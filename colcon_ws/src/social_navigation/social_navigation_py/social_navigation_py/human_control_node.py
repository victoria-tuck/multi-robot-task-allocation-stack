import rclpy
from rclpy.node import Node

from social_navigation_msgs.msg import HumanStates, RobotClosestObstacle
from geometry_msgs.msg import Twist, Vector3, Point
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


# from cbf_controller import cbf_controller
from .utils.mppi_obstacle_controller import MPPI_FORESEE
import numpy as np
import jax.numpy as jnp

import socialforce

class HumanController(Node):

    def __init__(self):
        super().__init('human_controller')

        self.num_humans = 10


        # Subscribers
        self.humans_state_sub = self.create_subscription( HumanStates, '/human_states', self.human_state_callback, 10 )
        self.robot_state_callback = self.create_subscription( Odometry, '/odom', self.robot_state_callback, 10 )        
        self.human_obstacle_subscriber = self.create_subscription( RobotClosestObstacle, '/human_closest_obstacles', self.obstacle_callback, 10 )
        # self.human_single_obstacle_subscriber = self.create_subscription( RobotClosestObstacle, '/human_single_closest_obstacles', self.single_obstacle_callback, 10 )
        
        self.robot_state_valid = False
        self.human_states_valid = False
        self.obstacles_valid = False

        self.human_states = np.zeros((2,self.num_humans))
        self.human_states_dot = np.zeros((2,self.num_humans))
        self.robot_state = np.zeros((3,1))
        self.robot_state_dot = np.zeros((3,1))

        socialforce_initial_state = np.append( np.append( np.copy( self.humans_states.T ), np.copy( self.humans_states_dot.T ) , axis = 1 ), self.humans.goals.T, axis=1   )
        self.humans_socialforce = socialforce.Simulator( socialforce_initial_state, delta_t = dt )

        

        self.human_command_pubs = []
        for i in range(self.num_humans):
            self.human_command_pubs.append( self.create_publisher( PoseStamped, '/actor'+str(i)+'/cmd_vel', 1) )
        self.timer_period = 0.05
        self.controller_timer = self.create_timer(self.timer_period, self.controller_callback)
        self.decide_waypoint_timer = self.create_timer(self.timer_period, self.decide_waypoint_callback)

        # Read waypoints for humans. should match with naming in world files
        self.waypoints = [] # list(2 x N)
        self.current_waypoint_ids = [0]*self.num_humans # (N,)
        self.dist_threshold = 0.3

        self.socialforce = socialforce.Simulator( delta_t = self.timer_period )                                                                                                                                                                         QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
        self.MAX_SPEED_MULTIPLIER = 1.3
        initial_speed = 2.0 #1.0
        self.initial_speeds = jnp.ones((self.num_humans+1)) * initial_speed
        self.max_speeds = self.MAX_SPEED_MULTIPLIER * self.initial_speeds

        self.human_goals = np.array([
                                [-5, 11],
                                [5, 11]
                            ])

    def decide_next_waypoint(self):

        human_goals = []
        for i in range(self.humans):
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

    def controller_callback(self):

        if not ( self.human_states_valid and self.robot_state_valid ):
            return

        human_states = np.copy(self.human_states)
        human_states_dot = np.copy(self.human_states_dot)
        # Set state here

        socialforce_initial_state = np.append( np.append( human_states.T, self.human_states_dot.T , axis = 1 ), self.humans_goals, axis=1   )
        # robot_goal = self.robot_state[0:2] + self.normalize(self.robot_state_dot[0:2]) * 10 # 10 meters away in the direction of current velocity
        # robot_social_state = np.array([ self.robot_state[0,0], self.robot_state[1,0], self.robot_states_dot[0,0], self.robot_states_dot[1,0], robot_goal[0,0], self.robot_goal[1,0]])
        # humans_socialforce.state[-1,0:6] = robot_social_state
        
        humans_socialforce.state = robot_social_state

        humans_controls = self.humans_socialforce.step().state.copy()[:,2:4].copy().T

        for i in range(self.num_humans):
            vel = Twist()
            vel.linear.x = humans_controls[0,i]
            vel.linear.y = humans_controls[1,i]
            self.human_command_pubs[i].publish(vel)


