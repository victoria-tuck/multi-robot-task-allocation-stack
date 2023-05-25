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

paths_file = []#'/home/hsr/catkin_ws/src/hsr_demo/files/paths.npy'

class human_controller:    
    def __init__(self, num_people = 10):

        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.get_model_srv = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

        self.num_people = num_people
        self.poses = np.zeros((3,num_people))        
        
        self.vel_pub = [ rospy.Publisher("/human"+str(i+1)+"/cmd_vel", Twist, queue_size=10)  for i in range(self.num_people) ]
        self.path_pub = [ rospy.Publisher("/human"+str(i+1)+"/cmd_path", Twist, queue_size=10)  for i in range(self.num_people) ]
        
        # self.pose_sub = [ rospy.Subscriber("/human"+str(i+1)+"/pose", PoseStamped, functools.partial(self.pose_callback, i) ) for i in range(self.num_people) ]
        self.get_model_srv = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)        
        
def wrap_angle(angle):
    return np.arctan2( np.sin(angle), np.cos(angle) ) 

# Set Figure
plt.ion()
fig = plt.figure()
ax = plt.axes(xlim=(-10,10), ylim=(-10,10))
ax.set_xlabel("X")
ax.set_ylabel("Y")

# Simulation Parameters-
dt_human = 0.5 #0.2
tf_human = 10#40.0
horizon_human = int(tf_human/dt_human)
humans = crowd(ax, crowd_center = np.array([0,0]), num_people = num_people, dt = dt_human, horizon = horizon_human, paths_file = paths_file)#social-navigation/
h_curr_humans = np.zeros(num_people)   
human_controller_gazebo = human_controller(num_people)

# hard code positions and speeds
humans.X[0,0] = 0.0; humans.X[1,0] = 0.0;
humans.X[0,1] = 1.0; humans.X[1,1] = 0.0;
humans.X[0,2] = -1.0; humans.X[1,2] = 0.0;
humans.X[0,3] = 2.0; humans.X[1,3] = 0.0;
humans.X[0,4] = -2.0; humans.X[1,4] = 0.0;
humans.X[0,5] = -4.0; humans.X[1,5] = 1.0;
humans.X[0,6] = -4.0; humans.X[1,6] = 1.5;
humans.X[0,7] = -4.0; humans.X[1,7] = 2.0;
humans.X[0,8] = -4.0; humans.X[1,8] = 2.5;
humans.X[0,9] = -5.0; humans.X[1,9] = 0.0;

# human.controls = np.zeros((2,num_people))
humans.controls[0,0] = 0.0; humans.controls[1,0] = 1.0;
humans.controls[0,1] = 0.0; humans.controls[1,1] = 0.5;
humans.controls[0,2] = 0.0; humans.controls[1,2] = 0.5;
humans.controls[0,3] = 0.0; humans.controls[1,3] = 1.0;
humans.controls[0,4] = 0.0; humans.controls[1,4] = 1.0;
humans.controls[0,5] = 0.5; humans.controls[1,5] = 0.5;
humans.controls[0,6] = 0.5; humans.controls[1,6] = 0.5;
humans.controls[0,7] = 0.5; humans.controls[1,7] = 0.5;
humans.controls[0,8] = 0.5; humans.controls[1,8] = 0.5;
humans.controls[0,9] = 0.5; humans.controls[1,9] = 0.5;
humans.controls = humans.controls / 2.0

adaptive_input_prev = np.zeros((2,mpc_horizon))

adaptive_sim = True

# Set ROS node
sim_frequency = 20
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
model_robot = GetModelStateRequest()
model_robot.model_name = 'hsrb'

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
result = get_model_srv(model_robot)
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

######## Set Initial Human Pose ######################
model_human = GetModelStateRequest()

# Set initial human location
state_msg = ModelState()
state_msg.pose.position.z = 1.2138
state_msg.pose.orientation.x = 0
state_msg.pose.orientation.y = 0

# Set initial pose of humans
rospy.wait_for_service('/gazebo/set_model_state')

iter = 0
while not rospy.is_shutdown():
    for i in range(num_people):            
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
    for i in range(num_people):
        des_heading = 0.0 * np.pi / 4 
        des_heading = np.arctan2( humans.controls[1,i], humans.controls[0,i] )
        # print(f"desired_heading :{des_heading}, {np.arctan2(-humans.X0[1,i], -humans.X0[0,i])}, pos:{humans.X0[0:2,i]}, goal:{humans.goals[0:2,i]}")
        state_msg = ModelState()
        state_msg.model_name = 'human'+str(i+1)
        state_msg.pose.position.x = humans.X[0,i]
        state_msg.pose.position.y = humans.X[1,i]
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

#############################################################################################


t_robot = rospy.Time.now().nsecs
t_robot_prev = rospy.Time.now().nsecs
human_poses = np.zeros((3,num_people))
while not rospy.is_shutdown():

    ###### Get robot pose
    result = get_model_srv(model_robot)
    robot_pose[0,0] = result.pose.position.x
    robot_pose[1,0] = result.pose.position.y
    robot_pose[2,0] = wrap_angle(2.0*np.arctan2(result.pose.orientation.z, result.pose.orientation.w))
    robot_pose[3,0] = robot_control.linear.x
    robot.X = np.copy(robot_pose)
    ###################################################

    ### Get and Set Human pose

    # Get human pose
    # Get actual human location
    
    for i in range(num_people):
        model_human.model_name = 'human'+str(i+1)
        result = human_controller_gazebo.get_model_srv(model_human)
        humans.X[0,i] = result.pose.position.x
        humans.X[1,i] = result.pose.position.y
        humans.Xheading[0,i] = wrap_angle(2.0*np.arctan2(result.pose.orientation.z, result.pose.orientation.w) - np.pi/2)

    # Compute human velocity
    controls = np.zeros((2,num_people))
    for i in range(num_people):
        humans.controls[0,i] = humans.controls[0,i]
        humans.controls[1,i] = humans.controls[1,i]
        vel = Twist()
        vel.linear.x = np.linalg.norm( humans.controls[:,i] )
        vel.linear.y = 0.0
        vel.linear.z = 0.0            
        des_heading = np.arctan2( humans.controls[1,i], humans.controls[0,i] )
        vel.angular.x = 0.0
        vel.angular.y = 0.0 
        vel.angular.z = - 4.0 * wrap_angle( humans.Xheading[0,i] - des_heading )
        
        human_controller_gazebo.vel_pub[i].publish(vel)                
    
    human_positions = np.copy(humans.X)
    human_future_positions = humans.get_future_states_with_input(dt,mpc_horizon)
    
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
        t_robot = rospy.Time.now().nsecs
        delta_t = max((t_robot - t_robot_prev)/1000000000.0, 0.01)
        robot.step(mpc_sol.value(robot_inputs[:,0]), delta_t )
        # print(f"U_ref:{U_ref.T}, U:{ mpc_sol.value(robot_inputs[:,0]) }, {robot.X.T}, dt:{delta_t}")
        robot_control.linear.x = robot.X[3,0]    
        robot_control.angular.z = mpc_sol.value(robot_inputs[1,0])
        # print(f"inputs: {robot_inputs[:,0]}")
        pub.publish(robot_control)

        t_robot_prev = t_robot
        t_robot = rospy.Time.now().nsecs
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

    






