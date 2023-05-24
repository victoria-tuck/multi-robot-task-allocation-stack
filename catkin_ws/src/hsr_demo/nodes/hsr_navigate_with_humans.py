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

# HSR libraries
import controller_manager_msgs.srv

# Other libraries
from robot_controller_setup import *
from crowd import crowd

paths_file = '/home/hsr/catkin_ws/src/hsr_demo/files/paths.npy'

# Set Figure
plt.ion()
fig = plt.figure()
ax = plt.axes(xlim=(-10,10), ylim=(-10,10))
ax.set_xlabel("X")
ax.set_ylabel("Y")

# Simulation Parameters
dt_human = 0.5 #0.2
tf_human = 10#40.0
horizon_human = int(tf_human/dt_human)
humans = crowd(ax, crowd_center = np.array([0,0]), num_people = num_people, dt = dt_human, horizon = horizon_human, paths_file = paths_file)#social-navigation/
h_curr_humans = np.zeros(num_people)   

adaptive_input_prev = np.zeros((2,mpc_horizon))
human_position_prev = humans.current_position(t, dt)

adaptive_sim = True

# Set ROS node
sim_frequency = 15
dt = 1.0/sim_frequency
rospy.init_node('hsr_control_node', anonymous=True)
rate = rospy.Rate(sim_frequency) # 10hz

########## Robot Setup ##############################
pub = rospy.Publisher('/hsrb/command_velocity',Twist, queue_size=10)

# wait to establish connection between the controller
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)

# make sure the controller is running
rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
list_controllers = rospy.ServiceProxy(
    '/hsrb/controller_manager/list_controllers',
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
model.model_name = 'hsrb'

# Set robot initial pose
state_msg = ModelState()
state_msg.model_name = 'hsrb'
state_msg.pose.position.x = robot.X[0,0]
state_msg.pose.position.y = robot.X[1,0]
state_msg.pose.position.z = 0.0
state_msg.pose.orientation.x = 0
state_msg.pose.orientation.y = 0
state_msg.pose.orientation.z = np.sin(robot.X[2,0]/2.0)
state_msg.pose.orientation.w = np.cos(robot.X[2,0]/2.0)

try:
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state( state_msg )
except rospy.ServiceException as e:
    print(f"Service call failed: :{e}")
    exit()

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
#####################################################
t_robot = rospy.Time.now().secs
t_robot_prev = rospy.Time.now().secs
while t < tf:

    # Get robot pose
    result = get_model_srv(model)
    robot_pose[0,0] = result.pose.position.x
    robot_pose[1,0] = result.pose.position.y
    robot_pose[2,0] = wrap_angle(2.0*np.arctan2(result.pose.orientation.z, result.pose.orientation.w))
    robot_pose[3,0] = robot_control.linear.x
    robot.X = np.copy(robot_pose)
    
    human_positions = humans.current_position(t, dt)
    human_future_positions = humans.get_future_states(t,dt,mpc_horizon)
    
    for i in range(num_people):

        dist = robot.X[0:2] - human_positions[0:2,i].reshape(-1,1)                 
        h_curr_humans[i] = (dist.T @ dist - d_human**2)[0,0]
        if h_curr_humans[i]<-0.01:
            print(f"Adaptive safety violated")
        h_curr_humans[i] = max(h_curr_humans[i], 0.01) # to account for numerical issues
    
    # Find control input
    U_ref = robot.nominal_controller( goal )
    opti_mpc.set_value(robot_current_state, robot.X)
    opti_mpc.set_value(humans_state, human_future_positions)
    opti_mpc.set_value(h_human, h_curr_humans)
    opti_mpc.set_value(robot_input_ref, U_ref)
    opti_mpc.set_value(alpha_nominal_humans, robot.alpha_nominal)

    try:

        mpc_sol = opti_mpc.solve();
        t_robot = rospy.Time.now().secs
        robot.step(mpc_sol.value(robot_inputs[:,0]), t_robot - t_robot_prev)
        robot_control.linear.x = robot.X[3,0]    
        robot_control.angular.z = mpc_sol.value(robot_inputs[1,0])
        # print(f"inputs: {robot_inputs[:,0]}")
        pub.publish(robot_control)

        t_robot_prev = t_robot
        t_robot = rospy.Time.now().secs
    except Exception as e:
        print(e)
        pub.publish(robot_control)
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

    human_position_prev = np.copy(human_positions)
    
    t = t + dt
        
        
    






