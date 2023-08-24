import time
import numpy as np
import cvxpy as cp
import polytope as pt
import matplotlib.pyplot as plt

from dynamic_unicycle import dynamic_unicycle
from polytope_utils import *
from matplotlib.animation import FFMpegWriter
import jax.numpy as jnp

class cbf_controller:
    
    num_people = 4
    num_obstacles = 2
    alpha1 = 0.5
    alpha2 = 2.0
    robot = 0
    k_x = 0.5#30.0
    k_v = 3.0#1.5
    dt = 0.01
    controller2_base_layer = 0
    
    def __init__(self, robot_init_state, num_people, num_obstacles):

        # Sim parameters
        self.t = 0
        cbf_controller.dt = 0.03
        cbf_controller.num_people = num_people
        cbf_controller.num_obstacles = num_obstacles
        cbf_controller.alpha1_human = 2*np.ones(cbf_controller.num_people)
        cbf_controller.alpha2_human = 6*np.ones(cbf_controller.num_people)
        self.control_bound = 10.0
        self.goal = np.array([-3.0,1.0]).reshape(-1,1)
        cbf_controller.k_x = 0.5#30.0
        cbf_controller.k_v = 2.5#1.5
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
        self.objective2_base = cp.Minimize( cp.sum_squares(self.u2_base-self.u2_ref_base) )
        self.controller2_base = cp.Problem( self.objective2_base, self.const2_base )
        cbf_controller.controller2_base_layer = CvxpyLayer(self.controller2_base, parameters=[self.u2_ref_base, self.A2_base, self.b2_base], variables=[self.u2_base])


        ##################

        plt.ion()
        self.volume2 = []
        self.fig1, self.ax1 = plt.subplots( 1, 3, figsize=(18, 6), gridspec_kw={'width_ratios': [5, 5, 2]})# )#, gridspec_kw={'height_ratios': [1, 1]} )
        self.ax1[0].set_xlim([-3,5])
        self.ax1[0].set_ylim([-3,5])
        self.offset = 3.0
        self.ax1[1].set_xlim([-self.control_bound-self.offset, self.control_bound+self.offset])
        self.ax1[1].set_ylim([-self.control_bound-self.offset, self.control_bound+self.offset])

        # Robot
        cbf_controller.robot = dynamic_unicycle( self.ax1[0], pos = np.array([ robot_init_state[0,0], robot_init_state[1,0], robot_init_state[2,0], 0.0 ]), dt = self.dt, plot_polytope=False )
        self.control_input_limit_points = np.array([ [self.control_bound, self.control_bound], [-self.control_bound, self.control_bound], [-self.control_bound, -self.control_bound], [self.control_bound, -self.control_bound] ])
        self.control_bound_polytope = pt.qhull( self.control_input_limit_points )
        print(f"hull: {self.control_bound_polytope}")
        # exit()

    @staticmethod
    # @jit
    def construct_barrier_from_states(robot_state, humans_states, human_states_dot, alpha1_human, alpha2_human, robot_radius ):         
                # barrier function
                A = jnp.zeros((1,2)); b = jnp.zeros((1,1))
                for i in range(cbf_controller.num_people):
                    dh_dot_dx1, dh_dot_dx2, h_dot, h = cbf_controller.robot.barrier_humans_alpha_jax( robot_state, humans_states[:,i].reshape(-1,1), human_states_dot[:,i].reshape(-1,1), d_min = robot_radius)
                    # print(f"h:{h}, h_dot:{h_dot}")
                    A = jnp.append( A, dh_dot_dx1 @ cbf_controller.robot.g_jax(robot_state), axis = 0 )
                    b = jnp.append( b, - dh_dot_dx1 @ cbf_controller.robot.f_jax(robot_state) - dh_dot_dx2 @ human_states_dot[:,i].reshape(-1,1) - alpha1_human[i] * h_dot - alpha2_human[i] * (h_dot + alpha1_human[i]*h), axis = 0 )
                # for i in range(cbf_controller.num_obstacles):
                      
                return A[1:], b[1:]
    
    def policy_cbf(self, robot_state, robot_goal, robot_radius, human_states, human_states_dot, dt):
        
        self.robot.set_state(robot_state)
        # print(f"human_states: {human_states.T}")
        A, b = self.construct_barrier_from_states(jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), self.alpha1_human, self.alpha2_human, robot_radius )
        self.A2_base.value = np.append( np.asarray(A), -self.control_bound_polytope.A, axis=0 )
        self.b2_base.value = np.append( np.asarray(b), -self.control_bound_polytope.b.reshape(-1,1), axis=0 )
        self.u2_ref_base.value = cbf_controller.robot.nominal_controller( robot_goal, k_x = cbf_controller.k_x, k_v = cbf_controller.k_v )
        # print(f"solve: {self.controller2_base.solve()}")
        self.controller2_base.solve()
    
        
        # Plot polytope       
        self.ax1[1].clear()
        self.ax1[1].set_xlim([-self.control_bound-self.offset, self.control_bound+self.offset])
        self.ax1[1].set_ylim([-self.control_bound-self.offset, self.control_bound+self.offset])
        hull = pt.Polytope( -self.A2_base.value, -self.b2_base.value )
        hull_plot = hull.plot(self.ax1[1], color = 'g')
        plot_polytope_lines( self.ax1[1], hull, self.control_bound )

        # self.volume2.append(np.array(mc_polytope_volume( jnp.array(hull.A), jnp.array(hull.b.reshape(-1,1)), bounds = self.control_bound)))
        # self.ax1[2].plot( self.volume2, 'r' )
        # self.ax1[2].set_title('Polytope Volume')
        # print(f"CBF nominal acc: {self.u2_ref_base.value[0,0]}, omega:{self.u2_ref_base.value[1,0]}, final acc: {self.u2_base.value[0,0]}, omega: {self.u2_base.value[1,0]}")
        self.robot.step( self.u2_base.value, dt )
        
        # self.ax1[1].scatter( self.u2_base.value[0,0], self.u2_base.value[1,0], c = 'r', label = 'CBF-QP chosen control' )
        # self.robot.render_plot()

        # self.ax1[1].set_xlabel('Linear Acceleration'); self.ax1[1].set_ylabel('Angular Velocity')
        # self.ax1[1].legend()
        # self.ax1[1].set_title('Feasible Space for Control')

        self.fig1.canvas.draw()
        self.fig1.canvas.flush_events()
        # print(f"returned values :{self.robot.X[3,0]}, {self.u2_base.value[1,0]}")
        return self.robot.X[3,0], self.u2_base.value[1,0]
    
    def policy_nominal(self, robot_state, robot_goal, robot_radius, human_states, human_states_dot, dt):
        
        self.robot.set_state(robot_state)
        self.u2_ref_base.value = cbf_controller.robot.nominal_controller( robot_goal, k_x = cbf_controller.k_x, k_v = cbf_controller.k_v )
        self.robot.step( self.u2_ref_base.value, dt )
        return self.robot.X[3,0], self.u2_ref_base.value[1,0]
