import rclpy
from rclpy.node import Node

from social_navigation_msgs.msg import HumanStates, RobotClosestObstacle, RobotCluster, PoseStampedPair
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Bool
# from geometry_msgs.msg import Point
from tf2_ros.transform_broadcaster import TransformBroadcaster

# from cbf_controller import cbf_controller
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
        self.num_dynamic_obstacles = self.num_other_robots + self.num_humans
        self.other_robot_states = 100*np.ones((2,self.num_other_robots))
        self.other_robot_states_prev = np.zeros((2,self.num_other_robots))
        self.other_robot_states_dot = np.zeros((2,self.num_other_robots))
        self.distance_to_other_robots = 100*np.ones((self.num_other_robots,1))
        self.connected_robots = []
        self.human_states = 100*np.ones((2,self.num_humans))
        self.human_states_prev = np.zeros((2,self.num_humans))
        self.human_states_dot = np.zeros((2,self.num_humans))

        # Parameters
        self.robot_radius = 0.12 # Previous values have been 0.2 and 0.18
        self.min_robot_to_robot = 4
        self.replan_count = 0
        self.print_count = 0
        self.timer_period_s = 0.05
        self.goal = np.array([0,0]).reshape(-1,1)
        
        #Controller
        self.control_prev  = np.array([0.0,0.0])
        self.controller = cbf_controller( self.robot_state, self.num_dynamic_obstacles, self.num_obstacles, 1.0, 2.0)

        # Call once to initiate JAX JIT
        self.controller_id = 0
        dummy_time_step = self.timer_period_s
        if self.controller_id == 0:
            self.dynamic_obstacle_states_valid, self.human_states_valid, self.all_other_robot_states_valid = True, True, True
            self.update_dynamic_obstacles()
            self.controller.policy_cbf(self.robot_state, self.goal, self.robot_radius, self.dynamic_obstacle_states, self.dynamic_obstacle_states_dot, self.obstacle_states, dummy_time_step)
        elif self.controller_id == 1:
            self.controller.policy_nominal(self.robot_state, self.goal, dummy_time_step)

        # Subscribers
        self.humans_state_sub = self.create_subscription( HumanStates, '/human_states', self.human_state_callback, 10 )
        self.other_robot_state_sub = { robot: self.create_subscription( Odometry,  f'/{robot}/odom', self.make_other_robot_state_callback(i), 10) for i, robot in enumerate(self.other_robots) } 
        self.robot_state_subscriber = self.create_subscription( Odometry, self.prefix + '/odom', self.robot_state_callback, 10 )
        self.obstacle_subscriber = self.create_subscription( RobotClosestObstacle, self.prefix + '/robot_closest_obstacles', self.obstacle_callback, 10 )
        self.plan_init_sub = self.create_subscription( Bool, '/planner_init', self.controller_plan_init_callback, 10 )
        self.goal_subscriber = self.create_subscription(PoseStampedPair, f'{self.prefix}/goal_location', self.new_goal_callback, 1)
        self.cluster_sub = { robot : self.create_subscription(RobotCluster, f'{robot}/cluster', self.make_cluster_callback(i, robot), 10) for i, robot in enumerate(self.other_robots)}
        self.activity_sub = self.create_subscription(Bool, f'{self.prefix}/active', self.status_callback, 10)

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
        self.error_count = 0
        self.h_min_dyn_obs_count = 0
        self.h_min_obs_count = 0
        self.finalized_each_cluster = [False] * self.num_other_robots
        self.finalized_cluster = False
        self.robot_cluster = (self.name, [self.name])
        self.other_robots_clusters = [(robot, [robot]) for robot in self.other_robots]
        self.robots_active = dict(zip(all_robots, [False] * (self.num_other_robots + 1)))

        # Publishers
        self.robot_command_pub = self.create_publisher( Twist, self.prefix + '/cmd_vel', 10 )
        self.nav2_path_publisher = self.create_publisher( Path, self.prefix + '/plan', 1)
        self.robot_local_goal_pub = self.create_publisher( PoseStamped, self.prefix + '/local_goal', 1)
        self.robot_location_pub = self.create_publisher( PoseStamped, self.prefix + '/robot_location', 1)
        self.robot_new_odom_pub = self.create_publisher(Odometry, f"{self.prefix}/new_odom", 10)
        self.robot_cluster_pub = self.create_publisher(RobotCluster, f"{self.prefix}/cluster", 10)
        
        # Frame broadcaster
        self.robot_tf_broadcaster = TransformBroadcaster(self)

        # Connect to planner
        self.navigator = BasicNavigator()
        self.path = Path()
        
        # Start controller
        self.time_prev = self.get_clock().now().nanoseconds
        self.get_logger().info(f"Current time: {self.time_prev}")
        self.controller_timer = self.create_timer(self.timer_period_s, self.run_controller)
        self.nearby_robots_timer = self.create_timer(self.timer_period_s, self.nearby_robots)
        self.get_logger().info("User Controller is ONLINE")

    def update_dynamic_obstacles(self):
        if self.human_states_valid and self.all_other_robot_states_valid and self.dynamic_obstacle_states_valid:
            self.dynamic_obstacle_states = np.hstack((self.other_robot_states, self.human_states))
            self.dynamic_obstacle_states_prev = np.hstack((self.other_robot_states_prev, self.human_states_prev))
            self.dynamic_obstacle_states_dot = np.hstack((self.other_robot_states_dot, self.human_states_dot))
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
            other_robot_pos = np.array([position.x, position.y])
            self.other_robot_states[:,index] = other_robot_pos
            # self.get_logger().info(f"Calculated distance between {self.name} and {self.other_robots[index]}: {np.linalg.norm(other_robot_pos - self.robot_pos)}")
            # if self.print_count > 10:
            #     self.get_logger().info(f"Current robot position: {self.robot_pos}")
            #     self.get_logger().info(f"Difference between robots: {other_robot_pos.reshape((2,1)) - self.robot_pos.reshape((2,1))}")
            self.distance_to_other_robots[index] = np.linalg.norm(other_robot_pos.reshape((2,1)) - self.robot_pos.reshape((2,1)))
            # if self.print_count > 10:
            #     self.get_logger().info(f"Distance to other robot: {np.linalg.norm(other_robot_pos.reshape((2,1)) - self.robot_pos.reshape((2,1)))}")
            velocity = msg.twist.twist.linear
            self.other_robot_states_dot[:, index] = np.array([velocity.x, velocity.y])
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
            self.other_robots_clusters[index] = (msg.leader, msg.cluster)
            self.finalized_cluster = all(self.finalized_each_cluster)
        return cluster_callback

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
        print(f"{self.name} received new goals: {msg}")

    def status_callback(self, msg):
        self.active = bool(msg.data)
        self.robots_active[self.name] = bool(msg.data)
        if msg.data:
            self.get_logger().info(f'{self.name} is active')
        else:
            self.get_logger().info(f'{self.name} is not active')
    
    def run_controller(self):
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
        # print(f"goal_init: {self.goal_init}")
        # print(f"new_goal_pose: {self.new_goal_poses}")
        # if (not self.goal_init or self.replan_count > 100) and self.new_goal_pose is not None:
        if not self.path_active:
            control = Twist()
            control.linear.x = 0.0
            control.angular.z = 0.0
            self.robot_command_pub.publish(control)
        if not self.goal_init and self.new_goal_poses is not None:
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
                if self.initial_goal:
                    # self.navigator.waitUntilNav2Active()
                    success = False
                    tries = 0
                    while not success:
                        try:
                            self.navigator.setInitialPose(current_pose)
                            path = self.navigator.getPath(current_pose, self.goal_pose) # replace with naman's planner
                            # if self.print_count > 10:
                            #     print(f"Should be initial pose: {path.poses[0].pose.position}")
                            #     print(f"Requested initial pose: {initial_pose.pose.position}")
                            assert path is not None
                            initial_pose_close_x = abs(path.poses[0].pose.position.x - current_pose.pose.position.x) < 0.05
                            initial_pose_close_y = abs(path.poses[0].pose.position.y - current_pose.pose.position.y) < 0.05
                            goal_pose_close_x = abs(path.poses[-1].pose.position.x - self.goal_pose.pose.position.x) < 0.05
                            goal_pose_close_y = abs(path.poses[-1].pose.position.y - self.goal_pose.pose.position.y) < 0.05
                            assert initial_pose_close_x and initial_pose_close_y and goal_pose_close_x and goal_pose_close_y
                            # print(f"Updated {self.name}'s path")
                            # if close_x and close_y:
                            self.path = path
                            self.path2 = path
                            # self.nav2_path_publisher.publish(self.path)
                            success = True
                            self.initial_goal = False
                            return
                        except Exception as e:
                            # print(f"Trying to find path again")
                            success = False
                        tries += 1

                success = False
                tries = 0
                while not self.initial_goal and not success:
                    next_goal = self.new_goal_poses.next_waypoint
                    # self.goal = np.array([ goal.pose.position.x, goal.pose.position.y ]).reshape(-1,1)
                    # Get current position and publish
                    start_pose = self.new_goal_poses.current_waypoint
                    # start_pose = PoseStamped()
                    # start_pose.header.frame_id = 'map'
                    # start_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                    # start_pose.pose.position.x = self.robot_state[0,0]
                    # start_pose.pose.position.y = self.robot_state[1,0]
                    # start_pose.pose.orientation.w = np.cos( self.robot_state[2,0]/2 )
                    # start_pose.pose.orientation.z = np.sin( self.robot_state[2,0]/2 )
                    # if self.print_count > 10:
                    #     print(f"Initial pose: {current_pose.pose.position}")
                    # self.navigator.clearGlobalCostmap()
                    self.navigator.setInitialPose(start_pose)
                    # self.navigator.waitUntilNav2Active()
                    try:
                        path = self.navigator.getPath(start_pose, next_goal) # replace with naman's planner
                        # if self.print_count > 10:
                        #     print(f"Should be initial pose: {path.poses[0].pose.position}")
                        #     print(f"Requested initial pose: {initial_pose.pose.position}")
                        assert path is not None
                        initial_pose_close_x = abs(path.poses[0].pose.position.x - start_pose.pose.position.x) < 0.05
                        initial_pose_close_y = abs(path.poses[0].pose.position.y - start_pose.pose.position.y) < 0.05
                        goal_pose_close_x = abs(path.poses[-1].pose.position.x - next_goal.pose.position.x) < 0.05
                        goal_pose_close_y = abs(path.poses[-1].pose.position.y - next_goal.pose.position.y) < 0.05
                        assert initial_pose_close_x and initial_pose_close_y and goal_pose_close_x and goal_pose_close_y
                        print(f"Updated {self.name}'s path")
                        # if close_x and close_y:
                        self.path.poses = self.path2.poses + path.poses
                        self.path2 = path
                        # self.nav2_path_publisher.publish(self.path)
                        self.path_active = True
                        self.goal_init = True
                        success = True
                        self.new_goal_poses = None
                        self.initial_goal = False
                        return
                    except Exception as e:
                        # print(f"Trying to find path again")
                        success = False
                    tries += 1 
            
        # Get next waypoint to follow from given path. It finds the next waypoint that is atleast 1 m away and removes the waypoints occurring before this 1 m point
        if (self.path_active and (self.robot_state_valid and self.human_states_valid and self.obstacles_valid)):
            # Select closest waypoint from received path
            assert np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]) is not None
            goal = np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]).reshape(-1,1)
            while (np.linalg.norm(goal[:,0] - self.robot_state[0:2,0])<0.5):#0.8
                if len(self.path.poses)>1:
                    self.path.poses = self.path.poses[1:]
                    assert np.array([self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]) is not None
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
                if self.controller_id == 0:
                    speed, omega, h_dyn_obs_min, h_obs_min = self.controller.policy_cbf( self.robot_state, goal, self.robot_radius, self.dynamic_obstacle_states, self.dynamic_obstacle_states_dot, self.obstacle_states, dt )
                elif self.controller_id == 1:
                    speed, omega, h_dyn_obs_min, h_obs_min = self.controller.policy_nominal( self.robot_state, goal, dt )
                
                # Check if any collision constraints violated
                if h_dyn_obs_min < -0.01:
                    self.h_min_dyn_obs_count += 1
                    self.get_logger().info(f"dynamic obstacle violate: {self.h_min_dyn_obs_count}")
                if h_obs_min < -0.01:
                    self.h_min_obs_count += 1
                    self.get_logger().info(f"obstacle violate: {self.h_min_obs_count}")
            except Exception as e:
                speed = 0.0
                omega = 0.0
                self.error_count = self.error_count + 1
                print(f"ERROR ******************************** count: {self.error_count} {e}")
                
            self.time_prev = t_new

            ############## Publish Control Input ###################
            control = Twist()
            if not self.finalized_cluster:
                # self.get_logger().info(f"{self.name}'s cluster has not been finalized")
                control.linear.x = 0.0
                control.angular.z = 0.0
            elif (self.finalized_cluster and self.name != self.robot_cluster[0]):
                # self.get_logger().info(f"{self.name} is not the leader.")
                control.linear.x = 0.0
                control.angular.z = 0.0
            else:
                control.linear.x = speed
                control.angular.z = omega
            self.robot_command_pub.publish(control)
        else:
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
                neighbors.append(i)
        # self.get_logger().info(f"Positions of other robots: {self.other_robot_states}")
        # self.connected_robots = [robot for i, robot in enumerate(self.other_robots) if nearby(i)]
        # self.get_logger().info(f"Robot {self.name} close to {connected_robots}")
        cluster_set = set()
        # self.get_logger().info(f"{self.name}'s neighborhood: {[self.other_robots[i] for i in neighbors]}")
        for i in neighbors:
            cluster_set.update(self.other_robots_clusters[i][1])
        cluster_set.update([self.name])
        cluster = list(cluster_set)
        self.get_logger().info(f"Active robots: {self.robots_active}")
        active_cluster = [robot for robot in cluster if self.robots_active[robot] is True]
        # active_cluster = cluster
        cluster_priorities = [self.robot_priorities[robot] for robot in active_cluster]
        if len(active_cluster) > 0:
            self.get_logger().info(f"{self.name}'s active cluster: {active_cluster}")
            leader = active_cluster[np.argmax(cluster_priorities)]
        else:
            self.get_logger().info(f"No other active robots in {self.name}'s cluster.")
            leader = self.name
        self.robot_cluster = (leader, cluster)

        msg = RobotCluster()
        msg.leader = leader
        msg.active = self.active
        msg.cluster = cluster
        self.robot_cluster_pub.publish(msg)
        self.get_logger().info(f"{self.name}'s cluster: {self.robot_cluster[1]} with leader {leader}.")
    

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
