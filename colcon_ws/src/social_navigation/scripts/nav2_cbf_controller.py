import rclpy
from rclpy.node import Node

from social_navigation_msgs.msg import HumanStates
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

from cbf_controller import cbf_controller
import numpy as np
# import jax.numpy as jnp

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # Variables
        self.num_humans = 11
        self.num_obstacles = 0
        self.robot_state = np.array([10,10,0.0, 0.1]).reshape(-1,1)
        self.human_states = np.zeros((2,self.num_humans))
        self.human_states_prev = np.zeros((2,self.num_humans))
        # self.t_human = self.get_clock().now().nanoseconds
        self.human_states_dot = np.zeros((2,self.num_humans)) 
        self.robot_radius = 0.8

        self.timer_period = 0.05 # seconds
        self.time_step = self.timer_period
        self.goal = np.array([0,0]).reshape(-1,1)
        
        #Controller
        self.control_prev  = np.array([0.0,0.0])
        self.controller = cbf_controller( self.robot_state, self.num_humans, self.num_obstacles)
        # Call once to initiate JAX JIT
        self.controller.policy_cbf(self.robot_state, self.goal, self.robot_radius, self.human_states, self.human_states_dot, self.time_step)


        # Subscribers
        self.humans_state_sub = self.create_subscription( HumanStates, '/human_states', self.human_state_callback, 10 )
        self.robot_state_callback = self.create_subscription( Odometry, '/odom', self.robot_state_callback, 10 )        
        self.goal_pose_subscriber = self.create_subscription( PoseStamped, 'goal_pose_custom', self.robot_goal_callback, 10 )
        self.human_states_valid = False
        self.robot_state_valid = False
        self.path_active = False
        # self.robot_nearest_obstacle_sub = self.create_sunscription(  )

        # Publishers
        self.robot_command_pub = self.create_publisher( Twist, '/cmd_vel', 10 )
        self.nav2_path_publisher = self.create_publisher( Path, '/plan', 1)
        
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

    def human_state_callback(self, msg):
        self.human_states_prev = np.copy(self.human_states)
        for i in range(self.num_humans):
            self.human_states[:,i] = np.array([ msg.states[i].position.x, msg.states[i].position.y ])
            self.human_states_dot[:,i] = np.array([ msg.velocities[i].linear.x, msg.velocities[i].linear.y ])
        self.human_states_valid = True
        # print(f"callback human_states: {self.human_states_dot}")

    def robot_state_callback(self, msg):
        self.robot_state = np.array(  [msg.pose.pose.position.x, msg.pose.pose.position.y, 2 * np.arctan2( msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ), msg.twist.twist.linear.x]  ).reshape(-1,1)
        self.robot_state_valid = True
        # print(f"robot state: {self.robot_state}")
        
    def wrap_angle(self,theta):
        return np.arctan2( np.sin(theta), np.cos(theta) )

    def controller_callback(self):          
            
        # if self.robot_state_valid and self.human_states_valid:
        if (self.path_active and (self.robot_state_valid and self.human_states_valid)):
            
            # Select closest waypoint from received path
            goal = np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]).reshape(-1,1)
            while (np.linalg.norm(goal[:,0] - self.robot_state[0:2,0])<0.5):
                if len(self.path.poses)>1:
                    # self.get_logger().info('hello "%d"' % len(self.path.poses))
                    self.path.poses = self.path.poses[1:]
                    goal = np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]).reshape(-1,1)
                else:
                    break
            self.nav2_path_publisher.publish(self.path)
            
            t_new = self.get_clock().now().nanoseconds
            dt = (t_new - self.time_prev)/10**9
            try:
                # speed, omega = self.controller.policy_nominal( self.robot_state, self.goal, self.robot_radius, self.human_states, self.human_states_dot, dt )
                speed, omega = self.controller.policy_cbf( self.robot_state, self.goal, self.robot_radius, self.human_states, self.human_states_dot, dt )
                self.control_prev = np.array([speed, omega])
            except Exception as e:
                speed = self.control_prev[0]  #0.0
                omega = self.control_prev[1]  #0.0
                print(f"ERROR *******************************************")
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




            ########### Nominal Controller ##################
            # print(f"loc: {self.robot_state}, goal:{self.goal}")
            # print(f"robot state: {self.robot_state.T}, goal: {self.goal.T}")
            # self.goal = np.array([0,8.7]).reshape(-1,1)
            
            
            # error = self.goal[:,0] - self.robot_state[0:2,0]
            # theta_desired = np.arctan2( error[1], error[0] )
            # e_theta = self.wrap_angle(theta_desired - self.robot_state[2,0])
            # omega = 1.0 * e_theta
            # if np.linalg.norm(error)>0.05:
            #     speed = min(0.5 * np.linalg.norm(error) * np.cos(e_theta), 0.4)
            # else:
            #     speed = 0.0
                # self.path_active = False
            # print(f"speed: {speed}, omega: {omega}")
            ############## CBF Controller #########################
            # print(f"goal: {self.goal}")
            
                    # t_new = self.get_clock().now().nanoseconds
        # self.human_states_dot = (self.human_states - self.human_states_prev)/ ((t_new - self.t_human)/10**9)
        # self.t_human = t_new
        
        # control = Twist()
        # control.linear.x = 0.0
        # control.angular.z = 0.0
        # self.robot_command_pub.publish(control)
        # return 