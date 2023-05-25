# Other libraries
import numpy as np
import casadi as cd
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from bicycle import bicycle

def wrap_angle(angle):
    return np.arctan2( np.sin(angle), np.cos(angle) ) 

num_people = 10

t = 0
tf = 40.0
dt = 0.05##0.05
U_ref = np.array([2.0, 0.0]).reshape(-1,1)
control_bound = np.array([200.0, 200.0]).reshape(-1,1) # works if control input bound very large
d_human = 0.7#1.0#0.5#0.5
mpc_horizon = 6


alpha_cbf_nominal1 = 0.05#0.05#0.2 #
alpha_cbf_nominal2 = 0.9
h_offset = 0.07#0.07
# higher: more conservative in Discrete time
# lower: less conservative in Discrete time

# goal = np.array([7.5, 7.5]).reshape(-1,1)
# robot = bicycle(pos = np.array([0.0,0.0,np.pi/2, 0]), dt = dt, color = 'red', alpha_nominal = alpha_cbf_nominal1*np.ones(num_people), plot_label='more conservative')#2.5,-2.5,0

goal = np.array([0.0, 10.5]).reshape(-1,1)
robot = bicycle(pos = np.array([0.0,-1.0,np.pi/2, 0]), dt = dt, color = 'red', alpha_nominal = alpha_cbf_nominal1*np.ones(num_people), plot_label='more conservative')#2.5,-2.5,0

## BUILD MPC here #############################################################################################
opti_mpc = cd.Opti()

# Parameters to set inside time loop
humans_state = opti_mpc.parameter(2,(mpc_horizon+1)*num_people)
# humans_state_dot = opti_mpc.parameter(2,(mpc_horizon+1)*num_people)
h_human = opti_mpc.parameter(num_people)
robot_current_state = opti_mpc.parameter( robot.X.shape[0],1 )
robot_input_ref = opti_mpc.parameter(robot.U.shape[0], robot.U.shape[1])
alpha_nominal_humans = opti_mpc.parameter(num_people)

# Variables to solve for
robot_states = opti_mpc.variable(robot.X.shape[0], mpc_horizon+1)
robot_inputs = opti_mpc.variable(robot.U.shape[0], mpc_horizon)
alpha_human = opti_mpc.variable(num_people)
alpha1_human = opti_mpc.variable(num_people)
alpha2_human = opti_mpc.variable(num_people)

# alpha constraints
opti_mpc.subject_to( alpha_human >= np.zeros(num_people) )
opti_mpc.subject_to( alpha_human <= 0.99*np.ones(num_people) )
opti_mpc.subject_to( alpha1_human >= np.zeros(num_people) )
opti_mpc.subject_to( alpha2_human >= np.zeros(num_people) )
    
## Initial state constraint 
opti_mpc.subject_to( robot_states[:,0] == robot_current_state )

## Time Loop
objective  = 0.0
for k in range(mpc_horizon+1): # +1 For loop over time horizon
    
    ################ Dynamics ##########################
    if (k < mpc_horizon):
        opti_mpc.subject_to(  robot_states[:,k+1] == robot_states[:,k] + robot.f_casadi(robot_states[:,k])*dt + cd.mtimes(robot.g_casadi(robot_states[:,k])*dt, robot_inputs[:,k]) )
        opti_mpc.subject_to( robot_inputs[:,k] <= control_bound )
        opti_mpc.subject_to( robot_inputs[:,k] >= -control_bound )
        # current state-input contribution to objective ####
        U_error = robot_inputs[:,k] - robot_input_ref 
        objective += 100 * cd.mtimes( U_error.T, U_error )
    
    if 1:#(k > 0):
        ################ Collision avoidance with humans
        human_states_horizon = humans_state[0:2, k*num_people:(k+1)*num_people]
        # human_states_dot_horizon = humans_state_dot[0:2, k*num_people:(k+1)*num_people]

        if (k < mpc_horizon):
            humans_state_horizon_next = humans_state[0:2, (k+1)*num_people:(k+2)*num_people]
            human_states_dot_horizon = (humans_state_horizon_next - human_states_horizon)/dt
        for i in range(num_people):
            dist = robot_states[0:2,k] - human_states_horizon[0:2,i]  # take horizon step into account               
            h = cd.mtimes(dist.T , dist) - d_human**2
            if (k < mpc_horizon) and (k>0): 
                # opti_mpc.subject_to( h >= 0)
                # First order CBF condition
                opti_mpc.subject_to( h >= alpha_human[i]**k * h_human[i] + h_offset ) # CBF constraint # h_human is based on current state
                # if (k>0):
                #     opti_mpc.subject_to( h1 >= 0 )
        
# find control input ###############################          
alpha_humans_diff = alpha_human-alpha_nominal_humans
alpha1_humans_diff = alpha1_human-alpha_nominal_humans
alpha2_humans_diff = alpha2_human-30*alpha_nominal_humans
objective += 1.0 *(  cd.mtimes( alpha_humans_diff.T, alpha_humans_diff ) )  + 1.0 *(  cd.mtimes( alpha1_humans_diff.T, alpha1_humans_diff ) )  + 1.0 *(  cd.mtimes( alpha2_humans_diff.T, alpha2_humans_diff ) ) 
opti_mpc.minimize(objective)
    
option_mpc = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
opti_mpc.solver("ipopt", option_mpc)
    
    ############################################################################################