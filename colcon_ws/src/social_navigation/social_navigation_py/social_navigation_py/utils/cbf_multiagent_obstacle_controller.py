import time
import numpy as np
import cvxpy as cp
import polytope as pt
import matplotlib.pyplot as plt

from social_navigation_py.utils.multi_dynamic_unicycle import multi_dynamic_unicycle
from jax import lax
from social_navigation_py.utils.polytope_utils import *
from matplotlib.animation import FFMpegWriter
import jax.numpy as jnp

# from jaxopt import CvxpyQP
# from jaxopt import OSQP

class cbf_controller:
    
    num_people = 4
    num_obstacles = 2
    alpha1 = 1.0
    alpha2 = 2.0
    alpha_polytope = 1.0#0.7
    robot = 0
    k_x = 2.0#30.0
    k_v = 3.5#1.5
    dt = 0.01
    controller2_base_layer = 0
    
    def __init__(self, robot_init_state, num_people, num_obstacles, dynamic_alpha1=1.0, dynamic_alpha2=2.0):

        # Sim parameters
        self.t = 0
        cbf_controller.dt = 0.03
        cbf_controller.num_people = num_people
        cbf_controller.num_obstacles = num_obstacles
        cbf_controller.alpha1_human = dynamic_alpha1*np.ones(cbf_controller.num_people)
        # cbf_controller.alpha1_human = 1*np.ones(cbf_controller.num_people) #1 # v5:1, v6: 1
        cbf_controller.alpha2_human = dynamic_alpha2*np.ones(cbf_controller.num_people) #3 # v5:2, v6: 2
        cbf_controller.alpha1_obstacle = 2*np.ones(cbf_controller.num_obstacles+1) #2 # v5:2, v6: 2
        cbf_controller.alpha2_obstacle = 6*np.ones(cbf_controller.num_obstacles+1) # 6 # v5:6, v6: 4
        cbf_controller.alpha_polytope = 2.0 #2.0
        self.control_bound = 3.0
        self.goal = np.array([-3.0,1.0]).reshape(-1,1)
        cbf_controller.k_x = 1.5#2.0#1.5#  1.5#0.5#30.0
        cbf_controller.k_v = 3.0#2.5#3.0#  3.0#1.5
        self.d_min_human = 0.2#0.5
        
        ######### holonomic controller
        n = 4 + cbf_controller.num_people + 1 # number of constraints

        ##########
        # cvxpylayer base controller
        self.n_base = 4 + cbf_controller.num_people + cbf_controller.num_obstacles
        self.u2_base = cp.Variable((2,1))
        self.u2_ref_base = cp.Parameter((2,1))
        self.A2_base = cp.Parameter((self.n_base,2))
        self.b2_base = cp.Parameter((self.n_base,1))
        self.const2_base = [self.A2_base @ self.u2_base >= self.b2_base]
        # self.objective2_base = cp.Minimize( cp.sum_squares(self.u2_base-self.u2_ref_base) )
        self.objective2_base = cp.Minimize( cp.norm(self.u2_base-self.u2_ref_base) )
        self.controller2_base = cp.Problem( self.objective2_base, self.const2_base )
        cbf_controller.controller2_base_layer = CvxpyLayer(self.controller2_base, parameters=[self.u2_ref_base, self.A2_base, self.b2_base], variables=[self.u2_base])

        ##################

        plt.ion()
        self.fig1, self.ax1 = plt.subplots( 2, 1, figsize=(4, 9), gridspec_kw={'height_ratios': [5, 5]})# )#, gridspec_kw={'height_ratios': [1, 1]} )
        self.ax1[0].set_xlim([-10,10])
        self.ax1[0].set_ylim([-10,10])
        self.offset = 3.0
        self.ax1[1].set_xlim([-self.control_bound-self.offset, self.control_bound+self.offset])
        self.ax1[1].set_ylim([-self.control_bound-self.offset, self.control_bound+self.offset])

        # Robot
        cbf_controller.robot = multi_dynamic_unicycle( self.ax1[0], pos = np.array([ robot_init_state[0,0], robot_init_state[1,0], robot_init_state[2,0], 0.0 ]), dt = self.dt, plot_polytope=False, num_agents=1)
        self.control_input_limit_points = np.array([ [self.control_bound, self.control_bound+2.0], [-self.control_bound, self.control_bound+2.0], [-self.control_bound, -self.control_bound-2.0], [self.control_bound, -self.control_bound-2.0] ])
        # self.control_input_limit_points = np.array([ [self.control_bound, self.control_bound], [-self.control_bound, self.control_bound], [-self.control_bound, -self.control_bound], [self.control_bound, -self.control_bound] ])
        self.control_bound_polytope = pt.qhull( self.control_input_limit_points )
        print(f"hull: {self.control_bound_polytope}")

    @staticmethod
    @jit
    def construct_barrier_from_states(robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius ):         
        # barrier function
        A1, b1, h_human_min = cbf_controller.construct_barrier_from_states_humans(robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius )
        A2, b2, h_obs_min = cbf_controller.construct_barrier_from_states_obstacles(robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius )
        return jnp.append(A1, A2, axis=0), jnp.append(b1, b2, axis=0), h_human_min, h_obs_min
            
    @staticmethod
    @jit
    def construct_barrier_from_states_humans(robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius ):         
        # barrier function
        A = jnp.zeros((cbf_controller.num_people,2)); b = jnp.zeros((cbf_controller.num_people,1))
        h_min = 100
        def body(i, inputs):
            A, b, h_min = inputs
            # dh_dot_dx1, dh_dot_dx2, h_dot, h = cbf_controller.robot.barrier_humans_alpha_jax( robot_state, humans_states[:,i].reshape(-1,1), human_states_dot[:,i].reshape(-1,1), d_min = robot_radius+0.3)
            dh_dot_dx1, dh_dot_dx2, h_dot, h = cbf_controller.robot.barrier_humans_alpha_jax( robot_state, humans_states[:,i].reshape(-1,1), human_states_dot[:,i].reshape(-1,1), d_min = robot_radius+0.15)
            A = A.at[i, :].set( (dh_dot_dx1 @ cbf_controller.robot.g_jax(robot_state))[0,:  ]  )
            b = b.at[i,:].set( (- dh_dot_dx1 @ cbf_controller.robot.f_jax(robot_state) - dh_dot_dx2 @ human_states_dot[:,i].reshape(-1,1) - alpha1_human[i] * h_dot - alpha2_human[i] * (h_dot + alpha1_human[i]*h))[0,:] )
            h_min = jnp.min( jnp.array([h_min, h[0,0]]) )
            return A, b, h_min
        return lax.fori_loop( 0, cbf_controller.num_people, body, (A, b, h_min) )
    
    @staticmethod
    @jit
    def construct_barrier_from_states_obstacles(robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius ):         
        # barrier function
        A = jnp.zeros((cbf_controller.num_obstacles,2)); b = jnp.zeros((cbf_controller.num_obstacles,1))
        h_min = 100
        def body(i, inputs):
            A, b, h_min = inputs
            dh_dot_dx1, dh_dot_dx2, h_dot, h = cbf_controller.robot.barrier_alpha_jax( robot_state, obstacle_states[:,i].reshape(-1,1), d_min = robot_radius)
            A = A.at[i,:].set((dh_dot_dx1 @ cbf_controller.robot.g_jax(robot_state))[0,:] )
            b = b.at[i,:].set((- dh_dot_dx1 @ cbf_controller.robot.f_jax(robot_state) - alpha1_obstacle[i] * h_dot - alpha2_obstacle[i] * (h_dot + alpha1_obstacle[i]*h))[0,:] )
            h_min = jnp.min( jnp.array([h_min, h[0,0]]) )
            return A, b, h_min
        return lax.fori_loop( 0, cbf_controller.num_obstacles, body, (A, b, h_min) )

    def policy_nominal(self, robot_state, robot_goal, dt):
        
        self.robot.set_state(robot_state)
        self.u2_ref_base.value = cbf_controller.robot.nominal_controller( robot_goal, k_x = cbf_controller.k_x, k_v = cbf_controller.k_v )
        self.robot.step( self.u2_ref_base.value, dt )
        return self.robot.X[3,0], self.u2_ref_base.value[1,0]
    
                
    def policy_cbf(self, robot_state, robot_goal, robot_radius, human_states, human_states_dot, obstacle_states, dt):
        
        self.robot.set_state(robot_state)
        A, b, h_human_min, h_obs_min = self.construct_barrier_from_states(jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.array(obstacle_states), self.alpha1_human, self.alpha2_human, self.alpha1_obstacle, self.alpha2_obstacle, robot_radius )
        self.A2_base.value = np.append( np.asarray(A), -self.control_bound_polytope.A, axis=0 )
        self.b2_base.value = np.append( np.asarray(b), -self.control_bound_polytope.b.reshape(-1,1), axis=0 )
        self.u2_ref_base.value = cbf_controller.robot.nominal_controller( robot_goal, k_x = cbf_controller.k_x, k_v = cbf_controller.k_v )
        self.controller2_base.solve(solver=cp.GUROBI)
        self.robot.step( self.u2_base.value, dt )         
        
        # Plot polytope       
        # self.ax1[1].clear()
        # self.ax1[1].set_xlim([-self.control_bound-self.offset, self.control_bound+self.offset])
        # self.ax1[1].set_ylim([-self.control_bound-self.offset, self.control_bound+self.offset])
        # hull = pt.Polytope( -self.A2_base.value, -self.b2_base.value )
        # hull_plot = hull.plot(self.ax1[1], color = 'g')
        # plot_polytope_lines( self.ax1[1], hull, self.control_bound )
        # self.ax1[0].clear()
        # self.ax1[0].scatter(human_states[0,:], human_states[1,:], c = 'g')
        # self.ax1[0].scatter(robot_state[0,0], robot_state[1,0], c = 'r')    
        # self.ax1[0].scatter(obstacle_states[0,:], obstacle_states[1,:], c = 'k')
        # self.ax1[0].set_xlim([robot_state[0,0]-5, robot_state[0,0]+5])
        # self.ax1[0].set_ylim([robot_state[1,0]-5, robot_state[1,0]+5])
        # self.ax1[1].scatter(self.u2_ref_base.value[0,0], self.u2_ref_base.value[1,0],s = 70, edgecolors='r', facecolors='none' )
        # self.ax1[1].scatter(self.u2_base.value[0,0], self.u2_base.value[1,0],s = 50, edgecolors='r', facecolors='r' )
        
        # self.fig1.canvas.draw()
        # self.fig1.canvas.flush_events
        return self.robot.X[3,0], self.u2_base.value[1,0], h_human_min, h_obs_min
    
        # vels = self.robot.wheel_to_vel_g() @ np.array([ self.robot.X[3,0], self.u2_base.value[1,0] ]).reshape(-1,1)
        # return self.robot.X[3,0], vels[1,0]
    