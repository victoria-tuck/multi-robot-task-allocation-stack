import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from social_navigation_msgs.msg import HumanStates, RobotClosestObstacle
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import pickle
# from geometry_msgs.msg import Point

# from cbf_controller import cbf_controller
from .utils.cbf_obstacle_controller import cbf_controller
import numpy as np
# import jax.numpy as jnp

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class RobotController(Node):

    def __init__(self, name='default'):
        super().__init__(f'robot_controller_{name}')
        if name != "":
            prefix = "/" + name
        else:
            prefix = ""
        print(f'Robot prefix: {prefix}')

        # 0: cbf controller
        # 1: nominal controller
        self.controller_id = 0#1
        self.name = name
        self.prefix = prefix

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
        self.replan_count = 0
        self.print_count = 0

        self.timer_period = 0.05#0.05 # seconds
        self.time_step = self.timer_period
        self.goal = np.array([0,0]).reshape(-1,1)
        
        #Controller
        self.control_prev  = np.array([0.0,0.0])
        self.controller = cbf_controller( self.robot_state, self.num_humans, self.num_obstacles)

        # Call once to initiate JAX JIT
        if self.controller_id == 0:
            self.controller.policy_cbf(self.robot_state, self.goal, self.robot_radius, self.human_states, self.human_states_dot, self.obstacle_states, self.time_step)
        elif self.controller_id == 1:
            self.controller.policy_nominal(self.robot_state, self.goal, self.time_step)

        # Subscribers
        self.humans_state_sub = self.create_subscription( HumanStates, '/human_states', self.human_state_callback, 10 )
        self.robot_state_subscriber = self.create_subscription( Odometry, self.prefix + '/odom', self.robot_state_callback, 10 )
        # self.goal_pose_subscriber = self.create_subscription( PoseStamped, '/goal_pose_custom', self.robot_goal_callback, 10 )
        self.obstacle_subscriber = self.create_subscription( RobotClosestObstacle, self.prefix + '/robot_closest_obstacles', self.obstacle_callback, 10 )
        # self.robot_state_subscriber = self.create_subscription( Odometry, prefix + '/odom', self.robot_state_callback, 10 )        
        # self.goal_pose_subscriber = self.create_subscription( PoseStamped, prefix + '/goal_pose_custom', self.robot_goal_callback, 10 )
        # self.obstacle_subscriber = self.create_subscription( RobotClosestObstacle, prefix + '/robot_closest_obstacles', self.obstacle_callback, 10 )
        self.plan_init_sub = self.create_subscription( Bool, '/planner_init', self.controller_plan_init_callback, 10 )
        self.goal_listener = self.create_subscription( PoseStamped, self.prefix + '/goal_location', self.new_goal_callback, 1 )
        # self.goal_listener = self.create_subscription( PoseStamped, prefix + '/goal_location', self.new_goal_callback, 1 )
        self.new_goal_pose = None
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
        self.state_plotting_data = [[0],[0],[0]]
        self.goal_plotting_data = [[0],[0]]
        self.command_plotting_data = [[0],[0]]
        # self.robot_nearest_obstacle_sub = self.create_sunscription(  )

        # Publishers
        # self.robot_command_pub = self.create_publisher( Twist, prefix + '/cmd_vel', 10 )
        # self.nav2_path_publisher = self.create_publisher( Path, prefix + '/plan', 1)
        # self.robot_local_goal_pub = self.create_publisher( PoseStamped, prefix + '/local_goal', 1)
        # self.robot_location_pub = self.create_publisher( PoseStamped, prefix + '/robot_location', 1)
        self.robot_command_pub = self.create_publisher( Twist, self.prefix + '/cmd_vel', 10 )
        self.nav2_path_publisher = self.create_publisher( Path, self.prefix + '/plan', 1)
        self.robot_local_goal_pub = self.create_publisher( PoseStamped, self.prefix + '/local_goal', 1)
        self.robot_location_pub = self.create_publisher( PoseStamped, self.prefix + '/robot_location', 1)
        
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
        print(f"Started {self.name}")

    def human_state_callback(self, msg):
        self.human_states_prev = np.copy(self.human_states)
        for i in range( len(msg.states) ):#:self.num_humans):
            self.human_states[:,i] = np.array([ msg.states[i].position.x, msg.states[i].position.y ])
            self.human_states_dot[:,i] = np.array([ msg.velocities[i].linear.x, msg.velocities[i].linear.y ])
        self.human_states_valid = True

    def robot_state_callback(self, msg):
        self.robot_state = np.array(  [msg.pose.pose.position.x, msg.pose.pose.position.y, 2 * np.arctan2( msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ), msg.twist.twist.linear.x]  ).reshape(-1,1)
        if self.print_count > 10:
            # print(f"Current robot state: {self.robot_state}")
            self.print_count = 0
        else:
            self.print_count += 1
        self.robot_state_valid = True
        
    def obstacle_callback(self, msg):
        # self.num_obstacles = msg.num_obstacles
        self.obstacle_states_temp = 100*np.ones((2,self.num_obstacles))
        for i in range(min(msg.num_obstacles, self.num_obstacles)):
            self.obstacle_states_temp[:,i] = np.array([ msg.obstacle_locations[i].x, msg.obstacle_locations[i].y ]) 
        # print(f"Obstacles for {self.name}: {self.obstacle_states}")
        self.obstacle_states = np.copy(self.obstacle_states_temp)
        self.obstacles_valid = True
        
    def wrap_angle(self,theta):
        return np.arctan2( np.sin(theta), np.cos(theta) )

    def new_goal_callback(self, msg):
        msg.header.stamp = self.navigator.get_clock().now().to_msg()
        self.new_goal_pose = msg
        self.goal_init = False
        print(f"{self.name} received new goal: {msg}")
    
    def controller_callback(self):
        if self.print_count > 100:
            print(f"Planner init: {self.planner_init}")  
        if not self.planner_init:
            return

        # # Get current position and publish
        current_pose = PoseStamped()
        current_pose.header.frame_id = 'map'
        current_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        current_pose.pose.position.x = self.robot_state[0,0]
        current_pose.pose.position.y = self.robot_state[1,0]
        current_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
        current_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )
        self.robot_location_pub.publish(current_pose)
        # if self.print_count > 10:
        #     self.navigator.setInitialPose(current_pose)


        # set goal for first time
        # if self.print_count > 100:
        #     print(f"goal_init: {self.goal_init}")
        #     print(f"new_goal_pose: {self.new_goal_pose}")
        # if (not self.goal_init or self.replan_count > 100) and self.new_goal_pose is not None:
        if not self.goal_init and self.new_goal_pose is not None:
            print(self.robot_state_valid, self.human_states_valid, self.obstacles_valid)
            if (self.robot_state_valid and self.human_states_valid and self.obstacles_valid):
                # if self.print_count > 100:
                #     print("Start planning...")
                success = False
                while not success:
                    # msg = PoseStamped()
                    # msg.header.frame_id = "map"
                    # msg.header.stamp = self.navigator.get_clock().now().to_msg()
                    # msg.pose.position.x = 5.0   #6.5#9.0
                    # msg.pose.position.y = -18.0   #11.5#13.5
                    # msg.pose.position.z = 0.01
                    # msg.pose.orientation.x = 0.0
                    # msg.pose.orientation.y = 0.0
                    # msg.pose.orientation.z = 0.0
                    # msg.pose.orientation.w = 1.0
                    msg = self.new_goal_pose
                    self.goal_pose = msg
                    self.goal = np.array([ msg.pose.position.x, msg.pose.position.y ]).reshape(-1,1)
                    # Get current position and publish
                    initial_pose = PoseStamped()
                    initial_pose.header.frame_id = 'map'
                    initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                    initial_pose.pose.position.x = self.robot_state[0,0]
                    initial_pose.pose.position.y = self.robot_state[1,0]
                    initial_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
                    initial_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )
                    if self.print_count > 10:
                        print(f"Initial pose: {initial_pose.pose.position}")
                    # self.navigator.clearGlobalCostmap()
                    self.navigator.setInitialPose(initial_pose)
                    # self.navigator.waitUntilNav2Active()
                    try:
                        path = self.navigator.getPath(initial_pose, self.goal_pose) # replace with naman's planner
                        if self.print_count > 10:
                            print(f"Should be initial pose: {path.poses[0].pose.position}")
                            print(f"Requested initial pose: {initial_pose.pose.position}")
                        assert path is not None
                        close_x = abs(path.poses[0].pose.position.x - initial_pose.pose.position.x) < 0.05
                        close_y = abs(path.poses[0].pose.position.y - initial_pose.pose.position.y) < 0.05
                        assert close_x and close_y
                        print(f"Updated {self.name}'s path")
                        # if close_x and close_y:
                        self.path = path
                        # self.nav2_path_publisher.publish(self.path)
                        self.path_waypoint_index = 0
                        self.path_active = True
                        self.goal_init = True
                        self.replan_count = 0
                        success = True
                        self.new_goal_pose = None
                        return
                    except Exception as e:
                        print(f"Trying to find path again")
                        success = False
            
        # Get next waypoint to follow from given path. It finds the next waypoint that is atleast 1 m away and removes the waypoints occurring before this 1 m point
        if (self.path_active and (self.robot_state_valid and self.human_states_valid and self.obstacles_valid)):
            # Select closest waypoint from received path
            assert np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]) is not None
            goal = np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]).reshape(-1,1)
            while (np.linalg.norm(goal[:,0] - self.robot_state[0:2,0])<1.0):#0.8
                if len(self.path.poses)>1:
                    self.path.poses = self.path.poses[1:]
                    assert np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]) is not None
                    goal = np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]).reshape(-1,1)
                else:
                    break
            
            # if self.print_count > 10:
            #     print(f"{self.name}'s Current goal: {goal}")

            # Publish path for visualization (no other use)
            # self.nav2_path_publisher.publish(self.path)
            # goal_msg = PoseStamped()
            # goal_msg.pose.position.x = goal[0,0]
            # goal_msg.pose.position.y = goal[1,0]
            # goal_msg.pose.position.z = 0.0
            # if len(self.path.poses)>1:
            #     theta = np.arctan2( self.path.poses[1].pose.position.y - self.path.poses[0].pose.position.y, self.path.poses[1].pose.position.x - self.path.poses[0].pose.position.x )
            #     goal_msg.pose.orientation.z = np.sin( theta/2 )
            #     goal_msg.pose.orientation.w = np.cos( theta/2 )
            # goal_msg.header.frame_id = "map"
            # goal_msg.header.stamp = self.navigator.get_clock().now().to_msg()
            # self.robot_local_goal_pub.publish( goal_msg )
            
            t_new = self.get_clock().now().nanoseconds
            dt = (t_new - self.time_prev)/10**9
            # self.get_logger().info(f"dt: {dt}")
            try:                
                if self.controller_id == 0:
                    speed, omega, h_human_min, h_obs_min = self.controller.policy_cbf( self.robot_state, goal, self.robot_radius, self.human_states, self.human_states_dot, self.obstacle_states, dt )
                elif self.controller_id == 1:
                    speed, omega, h_human_min, h_obs_min = self.controller.policy_nominal( self.robot_state, goal, dt )
                
                # Check if any collision constraints violated
                if h_human_min < -0.01:
                    self.h_min_human_count += 1
                    self.get_logger().info(f"human violate: {self.h_min_human_count}")
                if h_obs_min < -0.01:
                    self.h_min_obs_count = 0
                    self.get_logger().info(f"obstacle violate: {self.h_min_obs_count}")
            except Exception as e:
                speed = self.control_prev[0]  #0.0
                omega = self.control_prev[1]  #0.0
                self.error_count = self.error_count + 1
                print(f"ERROR ******************************** count: {self.error_count} {e}")
                
            self.time_prev = t_new
            # print(f"CBF speed: {speed}, omega: {omega}, dt:{dt}")

            ############## Publish Control Input ###################
            control = Twist()
            if self.name == "robot2":
                control.linear.x = speed
                control.angular.z = omega
                # control.linear.x = 0.0
                # control.angular.z = -2.0
                # control.linear.x = 0.0
                # control.angular.z = 0.0
            elif self.name == "robot1":
                control.linear.x = speed
                control.angular.z = omega
                # control.linear.x = 0.2
                # control.angular.z = 0.0
            # control.linear.x = speed
            # control.angular.z = omega
            if self.print_count > 10:
                print(f"Robot's name: {self.name}")
                # print(f"{self.name}'s control: {control}")
                # print(f"{self.name}'s intended control: {speed}, {omega}")
            self.robot_command_pub.publish(control)
            self.replan_count += 1
            # self.robot_state_valid = False

            # self.state_plotting_data[0].append(self.robot_state[0,0])
            # self.state_plotting_data[1].append(self.robot_state[1,0])
            # self.state_plotting_data[2].append(self.robot_state[2,0])
            # self.goal_plotting_data[0].append(goal[0])
            # self.goal_plotting_data[1].append(goal[1])
            # self.command_plotting_data[0].append(speed)
            # self.command_plotting_data[1].append(omega)

            # state_file = open(f"{self.name}_state_data.pkl", "wb")
            # pickle.dump(self.state_plotting_data, state_file)
            # state_file.close()

            # goal_file = open(f"{self.name}_goal_data.pkl", "wb")
            # pickle.dump(self.goal_plotting_data, goal_file)
            # goal_file.close()

            # command_file = open(f"{self.name}_command_data.pkl", "wb")
            # pickel.dump(self.command_plotting_data, command_file)
            # command_file.close()


    # def robot_goal_callback(self, msg):
    #     print(f"Received new goal")
    #     self.goal_pose = msg
    #     self.goal = np.array([ msg.pose.position.x, msg.pose.position.y ]).reshape(-1,1)
    #     initial_pose = PoseStamped()
    #     initial_pose.header.frame_id = 'map'
    #     initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
    #     initial_pose.pose.position.x = self.robot_state[0,0]
    #     initial_pose.pose.position.y = self.robot_state[1,0]
    #     initial_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
    #     initial_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )

    #     self.path = self.navigator.getPath(initial_pose, self.goal_pose) # replace with naman's planner

    #     self.nav2_path_publisher.publish(self.path)
    #     self.path_waypoint_index = 0
    #     self.path_active = True
    #     # Now do control        
        
    
def main(args=None):
    rclpy.init(args=args)
    # robot_controller = RobotController("robot1")
    # robot_controller = RobotController('')
    robot_controller2 = RobotController("robot2")
    rclpy.spin(robot_controller2)
    # rclpy.spin(robot_controller)
    # executor = MultiThreadedExecutor()
    # executor.add_node(robot_controller)
    # executor.add_node(robot_controller2)
    # executor.spin()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()