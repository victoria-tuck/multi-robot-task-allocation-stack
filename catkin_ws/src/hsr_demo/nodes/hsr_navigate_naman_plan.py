# Ros libraries
import rospy
import roslib
import functools # for making copies of subscriber function
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist

# gazebo specific
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

import rospy
from mharp_msgs.srv import StartServer, GetMHARPMotionPlan

# HSR libraries
import controller_manager_msgs.srv

# Other libraries
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter

def wrap_angle(angle):
    return np.arctan2( np.sin(angle), np.cos(angle) ) 

# Set Figure
plt.ion()
fig = plt.figure()
ax = plt.axes(xlim=(-10,10), ylim=(-10,10))
ax.set_xlabel("X")
ax.set_ylabel("Y")

goal = np.array([9.0, 9.0]).reshape(-1,1)

# Set ROS node
sim_frequency = 15
dt = 1.0/sim_frequency
rospy.init_node('hsr_naman_control_node', anonymous=True)
rate = rospy.Rate(sim_frequency) # 10hz

########## Robot Setup ##############################################################################
pub1 = rospy.Publisher('/hsrb1/hsrb/command_velocity',Twist, queue_size=10)
while pub1.get_num_connections() == 0:                                         # wait to establish connection between the controller
    rospy.sleep(0.1)

# make sure the controller is running
rospy.wait_for_service('/hsrb1/hsrb/controller_manager/list_controllers')
list_controllers = rospy.ServiceProxy(
    '/hsrb1/hsrb/controller_manager/list_controllers',
    controller_manager_msgs.srv.ListControllers)
running = False
while running is False:
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        if c.name == 'omni_base_controller' and c.state == 'running':
            running = True

# Setup Robot Pose
rospy.wait_for_service('/gazebo/get_model_state')
rospy.wait_for_service('/gazebo/set_model_state')
get_model_srv = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
model = GetModelStateRequest()
model.model_name = 'hsrb1'

# Set robot initial pose
robotX = np.array([-9.0,-9.0, 0.0, 0.0]).reshape(-1,1)
state_msg = ModelState()
state_msg.model_name = 'hsrb1'
state_msg.pose.position.x = robotX[0,0]
state_msg.pose.position.y = robotX[1,0]
state_msg.pose.position.z = 0.0
state_msg.pose.orientation.x = 0
state_msg.pose.orientation.y = 0
state_msg.pose.orientation.z = np.sin(robotX[2,0]/2.0)
state_msg.pose.orientation.w = np.cos(robotX[2,0]/2.0)

try:
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state( state_msg )
except rospy.ServiceException as e:
    print(f"Service call failed: :{e}")
#     exit()

# Get robot pose
result = get_model_srv(model)
robot_pose = np.zeros((4,1))
robot_pose[0,0] = result.pose.position.x
robot_pose[1,0] = result.pose.position.y
robot_pose[2,0] = wrap_angle(2.0*np.arctan2(result.pose.orientation.z, result.pose.orientation.w))
robot_pose[3,0] = 0.0

# Robot Control Message
robot_control = Twist()
robot_control.linear.x = 0.0
robot_control.linear.y = 0.0
robot_control.linear.z = 0.0
robot_control.angular.x = 0.0
robot_control.angular.y = 0.0
robot_control.angular.z = 0.0
####################################################################################


#####################################################

def f_unicycle(X):
        return np.array([X[3,0]*np.cos(X[2,0]),
                         X[3,0]*np.sin(X[2,0]),
                         0,0]).reshape(-1,1)
        
def g_unicycle(X):
        return np.array([ [0, 0],[0, 0], [0, 1], [1, 0] ])
    
def step(X, U, dt): #Just holonomic X,T acceleration
        U = U.reshape(-1,1)
        X_next = X + ( f_unicycle(X) + g_unicycle(X) @ U )*dt
        return X_next
    
def nominal_controller(X, targetX):
        # print(f"pose:{self.X.T}, target:{targetX.T}")
        k_omega = 2.0 
        k_v = 1.4#0.15
        k_x = 0.7
        distance = np.linalg.norm( X[0:2]-targetX[0:2] )
        desired_heading = np.arctan2( targetX[1,0]-X[1,0], targetX[0,0]-X[0,0] )
        error_heading = wrap_angle( desired_heading - X[2,0] )
        print(f"error_heading: {error_heading}, distance: {distance}")    
        omega = k_omega * error_heading
        if distance>0.1:
            speed = k_x * distance  * np.cos(error_heading)
        else:
            speed = 0
        u_r = k_v * ( speed - X[3,0] )          
        return np.array([u_r, omega]).reshape(-1,1)

# strart server for each robot    
rospy.wait_for_service("start_motion_planning_server")
try: 
    start_server_proxy = rospy.ServiceProxy("start_motion_planning_server", StartServer)
    response = start_server_proxy("/home/naman/catkin_ws/src/naman_planner/mharp/parameters.yaml")
except rospy.ServiceException as e: 
    print(f"Initializing server failed!")
    print (e)
        
# sunscribe to service for 
rospy.wait_for_service("/get_mharm_motion_plan")

try: 
    mp_proxy = rospy.ServiceProxy("get_mharm_motion_plan", GetMHARPMotionPlan)
    # response = mp_proxy(1,[-8,-8,0],[-8,-6,0])
    # mp_proxy = rospy.ServiceProxy("get_mharm_motion_plan", GetMHARPMotionPlan)
    # print(f"robotX:{robotX}, goal:{goal}")
    response = mp_proxy(1,[robotX[0,0], robotX[1,0], robotX[2,0]],[goal[0,0],goal[1,0],0])
    motion_plan = response.motion_plan
    print (f"motion_plan:{motion_plan}")
except rospy.ServiceException as e: 
    print(e)

waypoint_index = 0
waypoint_index_max = len(motion_plan)

input("Press Enter to continue...")

while not rospy.is_shutdown():
    
    # Get robot pose
    result = get_model_srv(model)
    robot_pose[0,0] = result.pose.position.x
    robot_pose[1,0] = result.pose.position.y
    robot_pose[2,0] = wrap_angle(2.0*np.arctan2(result.pose.orientation.z, result.pose.orientation.w))
    robot_pose[3,0] = robot_control.linear.x
    robotX = np.copy(robot_pose)
    
    # get next waypoint
    print(f"robotX: {robotX[0:3,0]}")
    print(f"motion plan: {np.asarray(motion_plan[waypoint_index].wp)}")
    if np.linalg.norm(robotX[0:2,0]-np.asarray(motion_plan[waypoint_index].wp[0:2]))<0.2:
        waypoint_index = min( waypoint_index+1, waypoint_index_max-1)
    
    # Find control input
    U_ref = nominal_controller( robotX, np.asarray(motion_plan[waypoint_index].wp[0:2]).reshape(-1,1) )
    # U_ref = nominal_controller( robotX, np.array([-8.85000038, -8.85000038]).reshape(-1,1) )
    print(f"U_ref: {U_ref}")
    robotX_next = step(robotX, U_ref, dt)
    robot_control.linear.x = robotX_next[3,0]    
    robot_control.angular.z = U_ref[1,0]
    # exit()
    pub1.publish(robot_control)

    
    
        
    


        
        # exit()
        # u_temp = np.array([[-100],[0]])
        # opti_mpc.set_value(robot_input_ref, u_temp)
        # opti_mpc.set_initial( robot_inputs, np.repeat( u_temp, mpc_horizon, 1 ) ) 
        # try:
        #     mpc_sol = opti_mpc.solve();
        #     robot.step(mpc_sol.value(robot_inputs[:,0]))
        # except Exception as e:
        #     print(f"********************************* Adaptive: MPC Failed ********************************")
        #     adaptive_sim = False

    






