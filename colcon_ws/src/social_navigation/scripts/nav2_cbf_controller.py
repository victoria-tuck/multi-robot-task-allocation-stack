import rclpy
from rclpy.node import Node

from social_navigation_msgs.msg import HumanStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from cbf_controller import cbf_controller
import numpy as np
import jax.numpy as jnp

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # Variables
        self.num_humans = 4
        self.robot_state = np.array([0,0,0, 0.1]).reshape(-1,1)
        self.human_states = np.zeros((2,4))
        self.human_states_dot = np.zeros((2.4)) 

        self.timer_period = 0.1 # seconds
        self.time_step = self.timer_period

        # Subscribers
        self.humans_state_sub = self.create_subscription( HumanStates, '/human_states', self.human_state_callback, 10 )
        self.robot_state_callback = self.create_subscription( Odometry, '/odom', self.robot_state_callback, 10 )
        # self.robot_nearest_obstacle_sub = self.create_sunscription(  )

        # Publishers
        self.robot_command_pub = self.create_publisher( Twist, '/cmd_vel', 10 )

        # Planner
        self.navigator = BasicNavigator()

        #Controller
        self.controller = cbf_controller( self.robot_state, self.human_states, self.human_states_dot, self.time_step)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


        # Goal
        self.robot_goal = np.array([1,1]).reshape(-1,1)

    def human_state_callback(self, msg):
        for i in range(self.num_humans):
            self.human_states[:,i] = np.array([ msg.states[i].position.x, msg.states[i].position.y ])
            self.human_states_dot[:,i] = np.array([ msg.velocities[i].linear.x, msg.velocities[i].linear.y ])

    def robot_state_callback(self, msg):
        self.robot_state = np.array(  msg.pose.pose.position.x, msg.pose.pose.position.x, 2 * np.atan2( msg.pose.pose.orientation.z, msg.pose.pose.orienattion.w ), msg.twist.twist.linear.x  )

    def timer_callback(self):

        self.controller.policy( np.copy(self.robot_state),   np.copy(self.human_states), np.copy(self.human_states_dot),  )

    def plan_paths():
        return


