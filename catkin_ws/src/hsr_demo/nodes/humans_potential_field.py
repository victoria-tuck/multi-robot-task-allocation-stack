import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState
from geometry_msgs.msg import PoseStamped, Twist

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

import numpy as np
import casadi as cd
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from crowd import crowd

import roslib
import functools
#roslib.load_manifest('my_package')


import numpy as np

class human_controller:    
    def __init__(self, num_human = 10):

        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.get_model_srv = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

        self.num_human = num_human
        self.poses = np.zeros((3,num_human))        
        
        self.vel_pub = [ rospy.Publisher("/human"+str(i+1)+"/cmd_vel", Twist, queue_size=10)  for i in range(self.num_human) ]
        self.path_pub = [ rospy.Publisher("/human"+str(i+1)+"/cmd_path", Twist, queue_size=10)  for i in range(self.num_human) ]
        
        # self.pose_sub = [ rospy.Subscriber("/human"+str(i+1)+"/pose", PoseStamped, functools.partial(self.pose_callback, i) ) for i in range(self.num_human) ]
        self.get_model_srv = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)        
        
def wrap_angle(angle):
    return np.arctan2( np.sin(angle), np.cos(angle) ) 
        
        

def talker():
  
    
    num_human = 10
    human_controller_gazebo = human_controller(num_human)
    
    # Set Figure
    plt.ion()
    fig = plt.figure()
    ax = plt.axes(xlim=(-10,10), ylim=(-10,10))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    
    sim_frequency = 15
    dt = 1.0/sim_frequency

    # Load human trajectories    
    paths_file = []#'/home/hsr/catkin_ws/src/hsr_demo/files/paths.npy'
    dt_human = 0.5 #0.2
    tf_human = 10#40.0
    horizon_human = int(tf_human/dt_human)
    humans = crowd(ax, crowd_center = np.array([0,0]), num_people = num_human, dt = dt_human, horizon = horizon_human, paths_file = paths_file)#social-navigation/
    
    t = 0
    rospy.init_node('human_controller', anonymous=True)
    rate = rospy.Rate(sim_frequency) # 10hz
    
    # Set initial human location
    state_msg = ModelState()
    state_msg.pose.position.z = 1.2138
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    
    # Set initial pose of humans
    rospy.wait_for_service('/gazebo/set_model_state')
    
    iter = 0
    while not rospy.is_shutdown():
        for i in range(num_human):            
            vel = Twist()
            vel.linear.x = 0
            vel.linear.y = 0.0
            vel.linear.z = 0.0            
            vel.angular.x = 0.0
            vel.angular.y = 0.0 
            vel.angular.z = 0.0        
            human_controller_gazebo.vel_pub[i].publish(vel)
        iter += 1
        if iter>10:
            break
        rate.sleep()
    
    iter = 0
    while not rospy.is_shutdown():
        for i in range(num_human):
            des_heading = 0.0 * np.pi / 4 
            des_heading = np.arctan2( humans.goals[1,i]-humans.X0[1,i], humans.goals[0,i]-humans.X0[0,i] )
            # print(f"desired_heading :{des_heading}, {np.arctan2(-humans.X0[1,i], -humans.X0[0,i])}, pos:{humans.X0[0:2,i]}, goal:{humans.goals[0:2,i]}")
            state_msg = ModelState()
            state_msg.model_name = 'human'+str(i+1)
            state_msg.pose.position.x = humans.X0[0,i]
            state_msg.pose.position.y = humans.X0[1,i]
            state_msg.pose.position.z = 1.2138
            state_msg.pose.orientation.x = 0
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = np.sin((des_heading+np.pi/2)/2.0)
            state_msg.pose.orientation.w = np.cos((des_heading+np.pi/2)/2.0)
            
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state( state_msg )
            except rospy.ServiceException as e:
                print(f"Service call failed: :{e}")
                exit()
        rate.sleep()
        iter += 1
        if iter>3:
            break
    
    reset_vel = False
    reset_pos = False
    reset_iter = 0
    
    alternate = False
    while not rospy.is_shutdown():
        
        if (alternate == False):
            # Get actual human location
            human_poses = np.zeros((3,num_human))
            model = GetModelStateRequest()
            for i in range(num_human):
                model.model_name = 'human'+str(i+1)
                result = human_controller_gazebo.get_model_srv(model)
                human_poses[0,i] = result.pose.position.x
                human_poses[1,i] = result.pose.position.y
                human_poses[2,i] = wrap_angle(2.0*np.arctan2(result.pose.orientation.z, result.pose.orientation.w) - np.pi/2)
            alternate = True
        
        else:
            # Publish speed
            for i in range(num_human):
                control =  humans.potential_field(human_poses[0:2,:], num_human, i, humans.goals, 1.0)
                print(f"control:{control}")
                humans.controls[0,i] = control[0,0]
                humans.controls[1,i] = control[1,0]
                vel = Twist()
                vel.linear.x = np.linalg.norm( control )
                vel.linear.y = 0.0
                vel.linear.z = 0.0            
                des_heading = np.arctan2( control[1,0], control[0,0] )
                vel.angular.x = 0.0
                vel.angular.y = 0.0 
                vel.angular.z = - 2.0 * wrap_angle( human_poses[2,i] - des_heading )
                
                human_controller_gazebo.vel_pub[i].publish(vel)                
            alternate = False
                
        rate.sleep()
           
    
    
    

if __name__ == "__main__":
    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass  