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

class HumanStop(Node):

    def __init__(self):
        super().__init__('human_controller')

        self.declare_parameter('num_humans', 10)
        self.declare_parameter('timer_period', 0.05)
     
        self.num_humans =  self.get_parameter('num_humans').get_parameter_value().integer_value#10
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value #  0.05
     
        self.human_command_pubs = []
        for i in range(self.num_humans):
            self.human_command_pubs.append( self.create_publisher( Twist, '/actor'+str(i+1)+'/cmd_vel', 1) )   
        self.controller_timer = self.create_timer(self.timer_period, self.controller_callback)
   
    def controller_callback(self):

        for i in range(self.num_humans):
            # self.get_logger().info("Human Callback")
            vel = Twist()
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.human_command_pubs[i].publish(vel)

        
    
def main(args=None):
    rclpy.init(args=args)
    human_controller = HumanStop()
    rclpy.spin(human_controller)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

