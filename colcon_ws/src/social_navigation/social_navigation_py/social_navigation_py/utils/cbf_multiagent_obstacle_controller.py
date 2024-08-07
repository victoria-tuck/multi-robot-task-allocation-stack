import time
import pdb
import numpy as np
import cvxpy as cp
import polytope as pt
import matplotlib.pyplot as plt

from social_navigation_py.utils.multi_dynamic_unicycle import multi_dynamic_unicycle
from jax import lax
from social_navigation_py.utils.polytope_utils import *
from matplotlib.animation import FFMpegWriter
import jax.numpy as jnp
from functools import partial

# from jaxopt import CvxpyQP
# from jaxopt import OSQP

class multi_cbf_controller:
    
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
        multi_cbf_controller.dt = 0.03
        multi_cbf_controller.num_people = num_people
        multi_cbf_controller.num_obstacles = num_obstacles
        multi_cbf_controller.alpha1_human = dynamic_alpha1*np.ones(multi_cbf_controller.num_people)
        # multi_cbf_controller.alpha1_human = 1*np.ones(multi_cbf_controller.num_people) #1 # v5:1, v6: 1
        multi_cbf_controller.alpha2_human = dynamic_alpha2*np.ones(multi_cbf_controller.num_people) #3 # v5:2, v6: 2
        multi_cbf_controller.alpha1_obstacle = 2*np.ones(multi_cbf_controller.num_obstacles+1) #2 # v5:2, v6: 2
        multi_cbf_controller.alpha2_obstacle = 6*np.ones(multi_cbf_controller.num_obstacles+1) # 6 # v5:6, v6: 4
        multi_cbf_controller.alpha_polytope = 2.0 #2.0
        self.control_bound = 1.0 #3.0
        self.goal = np.array([-3.0,1.0]).reshape(-1,1)
        multi_cbf_controller.k_x = 1.5#2.0#1.5#  1.5#0.5#30.0
        multi_cbf_controller.k_v = 3.0#2.5#3.0#  3.0#1.5
        self.d_min_human = 0.2#0.5
        # multi_cbf_controller.num_agents = int(len(robot_init_state)/4)
        # print(f"Initial state: {robot_init_state}")
        self.num_agents = int(len(robot_init_state)/4)
        # print(f"Number of agents: {self.num_agents}")
        
        ######### holonomic controller
        n = 4 + multi_cbf_controller.num_people + 1 # number of constraints

        ##########
        # cvxpylayer base controller
        # self.n_base = self.num_agents*(4 + multi_cbf_controller.num_people + multi_cbf_controller.num_obstacles)
        self.n_base = self.num_agents * (4 + multi_cbf_controller.num_people + multi_cbf_controller.num_obstacles) + self.num_agents*(self.num_agents-1)
        self.u2_base = cp.Variable((2*self.num_agents,1))
        # print(f"Size told to u2 base: {(2*self.num_agents,1)}. Actual size: {(self.u2_base.size)}")
        self.u2_ref_base = cp.Parameter((2*self.num_agents,1))
        self.A2_base = cp.Parameter((self.n_base,2*self.num_agents))
        self.b2_base = cp.Parameter((self.n_base,1))
        self.const2_base = [self.A2_base @ self.u2_base >= self.b2_base]
        # self.objective2_base = cp.Minimize( cp.sum_squares(self.u2_base-self.u2_ref_base) )
        self.objective2_base = cp.Minimize( cp.norm(self.u2_base-self.u2_ref_base) )
        self.controller2_base = cp.Problem( self.objective2_base, self.const2_base )
        self.controller2_base_layer = CvxpyLayer(self.controller2_base, parameters=[self.u2_ref_base, self.A2_base, self.b2_base], variables=[self.u2_base])

        ##################

        # plt.ion()
        # self.fig1, self.ax1 = plt.subplots( 2, 1, figsize=(4, 9), gridspec_kw={'height_ratios': [5, 5]})# )#, gridspec_kw={'height_ratios': [1, 1]} )
        # self.ax1[0].set_xlim([-10,10])
        # self.ax1[0].set_ylim([-10,10])
        # self.offset = 3.0
        # self.ax1[1].set_xlim([-self.control_bound-self.offset, self.control_bound+self.offset])
        # self.ax1[1].set_ylim([-self.control_bound-self.offset, self.control_bound+self.offset])

        # Robot
        robots_init = []
        for i in range(self.num_agents):
            robots_init.append(np.array([robot_init_state[4*i,0], robot_init_state[4*i+1,0], robot_init_state[4*i+2,0], 0.0]).reshape(-1,1))
        robots_init_state = np.hstack(robots_init)
        # print(f"Initialize dynamic unicycle with {self.num_agents} agents")
        self.robot = multi_dynamic_unicycle( None, pos = robots_init_state, dt = self.dt, plot_polytope=False, num_agents=self.num_agents)
        def control_bounds(num_agents, ctrl_bound):
            A = np.zeros((4*num_agents, 2*num_agents))
            b = np.zeros((4*num_agents, 1))
            for i in range(num_agents):
                subarray = [ [1, 0], [-1, 0], [0, 1], [0, -1] ]
                A[4*i:4*(i+1), 2*i:2*(i+1)] = subarray
                b[4*i:4*(i+1)] = [[-ctrl_bound], [-ctrl_bound], [-ctrl_bound-1.0], [-ctrl_bound-1.0]]
                # Previous values that were way too fast:
                # b[4*i:4*(i+1)] = [[-ctrl_bound], [-ctrl_bound], [-ctrl_bound-2.0], [-ctrl_bound-2.0]]
            return np.array(A).reshape(-1,2*num_agents), np.array(b).reshape(-1,1)
        self.control_A, self.control_b = control_bounds(self.num_agents, self.control_bound)
        # self.control_input_limit_points = control_bounds(self.num_agents, self.control_bound)
        # self.control_input_limit_points = np.array([ [self.control_bound, self.control_bound], [-self.control_bound, self.control_bound], [-self.control_bound, -self.control_bound], [self.control_bound, -self.control_bound] ])
        # self.control_bound_polytope = pt.qhull( self.control_input_limit_points )
        # print(f"Control constraints: {self.control_A}u >= {self.control_b}")

    # @staticmethod
    @partial(jit, static_argnums=(0,))
    def construct_barrier_from_states(self, robot_state, other_robot_states, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius, alpha1_agent_agent, alpha2_agent_agent, num_agents ):         
        # barrier function
        # print(f"robot state: {robot_state}")
        # print(f"Other robot state: {other_robot_states}")
        A1, b1, h_human_min = self.construct_barrier_from_states_humans(robot_state, other_robot_states, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius)
        A2, b2, h_obs_min = self.construct_barrier_from_states_obstacles(robot_state, other_robot_states, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius)
        agent_agent_inputs = (robot_state, other_robot_states, alpha1_agent_agent, alpha2_agent_agent, robot_radius)
        num_agents = int(len(other_robot_states)/4) + 1
        # print(f"Num agents: {num_agents}")
        A3, b3, h_agent_min = self.construct_agent_to_agent_barrier(agent_agent_inputs)
        # A, b, h_agent_min = lax.cond(num_agents > 1, self.construct_agent_to_agent_barrier, self.empty_multiagent, agent_agent_inputs)
        # A, b, h_agent_min = self.construct_agent_to_agent_barrier(robot_state, other_robot_states, alpha1_agent_agent, alpha2_agent_agent, robot_radius)
        # A1_agent1 = lax.dynamic_slice(A1, (0,0), (multi_cbf_controller.num_people,2))
        # b1_agent1 = lax.dynamic_slice(b1, (0,0), (multi_cbf_controller.num_people,1))
        # A2_agent1 = lax.dynamic_slice(A2, (0,0), (multi_cbf_controller.num_obstacles,2))
        # b2_agent1 = lax.dynamic_slice(b2, (0,0), (multi_cbf_controller.num_obstacles,1))
        return jnp.append(jnp.append(A1, A2, axis=0), A3, axis=0), jnp.append(jnp.append(b1, b2, axis=0), b3), h_human_min, h_obs_min, h_agent_min
    
    # @partial(jit, static_argnums=(0,))
    # def empty_multiagent(self, inputs):
    #     print(f"Creating empty array")
    #     return jnp.empty((0,2)), jnp.empty((0,1)), 0.0
            
    # @staticmethod
    @partial(jit, static_argnums=(0,))
    def construct_barrier_from_states_humans(self, robot_state, other_robot_states, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius):         
        # barrier function
        complete_state = jnp.vstack((robot_state, other_robot_states))
        A_multi = jnp.zeros((multi_cbf_controller.num_people*self.num_agents,2*self.num_agents)); b_multi = jnp.zeros((multi_cbf_controller.num_people*self.num_agents,1))
        h_overallmin = 100
        def outer_body(j, outer_inputs):
            A_multi, b_multi, h_overallmin = outer_inputs
            A = jnp.zeros((multi_cbf_controller.num_people,2)); b = jnp.zeros((multi_cbf_controller.num_people,1))
            def body(i, inputs):
                A, b, h_min = inputs
                # dh_dot_dx1, dh_dot_dx2, h_dot, h = multi_cbf_controller.robot.barrier_humans_alpha_jax( robot_state, humans_states[:,i].reshape(-1,1), human_states_dot[:,i].reshape(-1,1), d_min = robot_radius+0.3)
                state = lax.dynamic_slice(complete_state, (4*i, 0), (4, 1))
                dh_dot_dx1, dh_dot_dx2, h_dot, h = self.robot.barrier_humans_alpha_jax( state, humans_states[:,i].reshape(-1,1), human_states_dot[:,i].reshape(-1,1), d_min = 4*robot_radius)
                A = A.at[i, :].set( (dh_dot_dx1 @ self.robot.g_jax_i(state))[0,:  ]  )
                b = b.at[i,:].set( (- dh_dot_dx1 @ self.robot.f_jax_i(state) - dh_dot_dx2 @ human_states_dot[:,i].reshape(-1,1) - alpha1_human[i] * h_dot - alpha2_human[i] * (h_dot + alpha1_human[i]*h))[0,:] )
                h_min = jnp.min( jnp.array([h_min, h[0,0]]) )
                return A, b, h_min
            A_update, b_update, h_overallmin = lax.fori_loop( 0, multi_cbf_controller.num_people, body, (A, b, h_overallmin) )
            A_multi = lax.dynamic_update_slice(A_multi, A_update, (multi_cbf_controller.num_people*j, 2*j))
            b_multi = lax.dynamic_update_slice(b_multi, b_update, (multi_cbf_controller.num_people*j, 0))
            # A_multi = A_multi.at[multi_cbf_controller.num_people*j:multi_cbf_controller.num_people*(j+1),2*j:2*(j+1)].set(A_update)
            # b_multi = b_multi.at[multi_cbf_controller.num_people*j:multi_cbf_controller.num_people*(j+1),j:(j+1)].set(b_update)
            return A_multi, b_multi, h_overallmin
        return lax.fori_loop(0, self.num_agents, outer_body, (A_multi, b_multi, h_overallmin))
    
    # @staticmethod
    @partial(jit, static_argnums=(0,))
    def construct_barrier_from_states_obstacles(self, robot_state, other_robot_states, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius):         
        # barrier function
        complete_state = jnp.vstack((robot_state, other_robot_states))
        A_multi = jnp.zeros((multi_cbf_controller.num_obstacles*self.num_agents, 2*self.num_agents)); b_multi = jnp.zeros((multi_cbf_controller.num_obstacles*self.num_agents,1))
        h_overallmin = 100
        def outer_body(j, outer_inputs):
            A_multi, b_multi, h_overallmin = outer_inputs
            A = jnp.zeros((multi_cbf_controller.num_obstacles,2)); b = jnp.zeros((multi_cbf_controller.num_obstacles,1))
            def body(i, inputs):
                A, b, h_min = inputs
                state = lax.dynamic_slice(complete_state, (4*i, 0), (4, 1))
                dh_dot_dx1, dh_dot_dx2, h_dot, h = self.robot.barrier_alpha_jax( state, obstacle_states[:,multi_cbf_controller.num_obstacles*j + i].reshape(-1,1), d_min = 4*robot_radius)
                A = A.at[i,:].set((dh_dot_dx1 @ self.robot.g_jax_i(state))[0,:] )
                b = b.at[i,:].set((- dh_dot_dx1 @ self.robot.f_jax_i(state) - alpha1_obstacle[i] * h_dot - alpha2_obstacle[i] * (h_dot + alpha1_obstacle[i]*h))[0,:] )
                h_min = jnp.min( jnp.array([h_min, h[0,0]]) )
                return A, b, h_min
            A_update, b_update, h_overallmin = lax.fori_loop( 0, multi_cbf_controller.num_obstacles, body, (A, b, h_overallmin) )
            A_multi = lax.dynamic_update_slice(A_multi, A_update, (multi_cbf_controller.num_obstacles*j, 2*j))
            b_multi = lax.dynamic_update_slice(b_multi, b_update, (multi_cbf_controller.num_obstacles*j, 0))
            return A_multi, b_multi, h_overallmin
        return lax.fori_loop(0, self.num_agents, outer_body, (A_multi, b_multi, h_overallmin))
    
    # @staticmethod
    @partial(jit, static_argnums=(0,))
    def construct_agent_to_agent_barrier(self, inputs):        
        robot_state, other_robot_states, alpha1_agent_agent, alpha2_agent_agent, robot_radius = inputs 
        # barrier function
        complete_state = jnp.vstack((robot_state, other_robot_states))
        # print(f"agent_agent_complete_state: {complete_state}")
        A_multi = jnp.zeros((self.num_agents*(self.num_agents-1), 2*self.num_agents)); b_multi = jnp.zeros((self.num_agents*(self.num_agents-1),1))
        h_overallmin = 100
        def outer_body(j, outer_inputs):
            A_multi, b_multi, h_overallmin, current_index = outer_inputs
            state_j = lax.dynamic_slice(complete_state, (4*j, 0), (4, 1))
            # A = jnp.zeros((multi_cbf_controller.num_obstacles,2)); b = jnp.zeros((multi_cbf_controller.num_obstacles,1))
            def body(i, inputs):
                A_multi, b_multi, h_min, current_index = inputs
                state_i = lax.dynamic_slice(complete_state, (4*i, 0), (4, 1))
                dh_dot_dx1, dh_dot_dx2, h_dot, h = self.robot.barrier_agent_agent_jax_ij( state_i, state_j, d_min = 4*robot_radius)
                # print(f"Derivatives: {dh_dot_dx1} and {dh_dot_dx2}")
                A_multi = lax.dynamic_update_slice(A_multi, dh_dot_dx1.T @ self.robot.g_jax_i(state_i), (current_index, 2*i))
                A_multi = lax.dynamic_update_slice(A_multi, dh_dot_dx2.T @ self.robot.g_jax_i(state_j), (current_index, 2*j))
                # A_multi = A_multi.at[current_index,:].set((dh_dot_dx1.T @ self.robot.g_jax_i(state_i) + dh_dot_dx2.T @ self.robot.g_jax_i(state_j))[0,:] )
                # b_multi = lax.dynamic_update_slice(b_multi, dh_dot_dx1.T @ self.robot.f_jax_i(state_i), (current_index, 2*i))
                b_multi = b_multi.at[current_index,:].set((- dh_dot_dx1.T @ self.robot.f_jax_i(state_i) - dh_dot_dx2.T @ self.robot.f_jax_i(state_j) - alpha1_agent_agent[current_index] * h_dot - alpha2_agent_agent[current_index] * (h_dot + alpha1_agent_agent[current_index]*h))[0,:] )
                h_min = jnp.min( jnp.array([h_min, h]) )
                return A_multi, b_multi, h_min, current_index + 1
            return lax.fori_loop( j+1, self.num_agents, body, (A_multi, b_multi, h_overallmin, current_index) )
            # A_multi = lax.dynamic_update_slice(A_multi, A_update, (multi_cbf_controller.num_obstacles*j, 2*j))
            # b_multi = lax.dynamic_update_slice(b_multi, b_update, (multi_cbf_controller.num_obstacles*j, 0))
            # return A_multi, b_multi, h_overallmin, new_index
        A_multi, b_multi, h_overallmin, _ = lax.fori_loop(0, self.num_agents, outer_body, (A_multi, b_multi, h_overallmin, 0))
        return A_multi, b_multi, h_overallmin

    def policy_nominal(self, robot_state, other_robot_states, robot_goal, dt):
        
        self.robot.set_state(robot_state)
        self.u2_ref_base.value = self.robot.nominal_controller( robot_goal, other_robot_states, k_x = multi_cbf_controller.k_x, k_v = multi_cbf_controller.k_v )
        self.robot.step( self.u2_ref_base.value, dt )
        return self.robot.X[3,0], self.u2_ref_base.value[1,0]
    
                
    def policy_cbf(self, robot_state, other_robot_states, robot_goal, robot_radius, human_states, human_states_dot, obstacle_states, other_obstacles, dt):
        
        complete_state = np.vstack((robot_state, other_robot_states))
        # print(f"Complete state: {complete_state}")
        self.robot.set_state(complete_state)
        alpha1_agent_agent = 2*np.ones(self.num_agents*(self.num_agents-1))
        alpha2_agent_agent = 5*np.ones(self.num_agents*(self.num_agents-1))
        start_time = time.time()
        num_agents = int(len(other_robot_states)/4) + 1
        # print(f"Numb agents: {num_agents}")
        reshaped_other_obstacles = jnp.array(other_obstacles).transpose(1,0,2).reshape(2,-1)
        obstacles = jnp.hstack((obstacle_states, reshaped_other_obstacles))
        A, b, h_human_min, h_obs_min, h_agent_min = self.construct_barrier_from_states(jnp.asarray(robot_state), jnp.asarray(other_robot_states), jnp.asarray(human_states), jnp.asarray(human_states_dot), obstacles, self.alpha1_human, self.alpha2_human, self.alpha1_obstacle, self.alpha2_obstacle, robot_radius, alpha1_agent_agent, alpha2_agent_agent, num_agents )
        # print(f"Shape of A: {A.shape}")
        # print(f"Shape of b: {b.shape}")
        # print(f"Time to create barriers: {time.time() - start_time}")
        self.A2_base.value = np.append( np.asarray(A), self.control_A, axis=0 )
        self.b2_base.value = np.append( np.asarray(b).reshape(-1,1), self.control_b, axis=0 )
        start_time = time.time()
        nominal_ctrl = self.robot.nominal_controller( robot_goal, other_robot_states, k_x = multi_cbf_controller.k_x, k_v = multi_cbf_controller.k_v )
        # print(f"Time to calculate nominal: {time.time() - start_time}")
        # self.u2_ref_base.value = nominal_ctrl[0:2,:]
        # print(f"U2 ref base: {self.u2_ref_base}")
        self.u2_ref_base.value = nominal_ctrl[0:2*num_agents]
        start_time = time.time()
        self.controller2_base.solve(solver=cp.GUROBI)
        if self.u2_base.value is None:
            print("Reoptimizing")
            self.controller2_base.solve(solver=cp.GUROBI, reoptimize=True)
        print(f"Time to run QP: {time.time() - start_time}")
        # if len(other_robot_states) > 0:
        #     exit()
        print(f"Calculated control: {self.u2_base.value}")
        # if self.num_agents > 1:
        #     U = np.vstack((self.u2_base.value, np.array([0,0]).reshape(-1,1)))
        # else:
        #     U = self.u2_base.value
        U = self.u2_base.value
        if U is not None:
            self.robot.step( U, dt )
            speed = np.zeros((num_agents, 1))
            omega = np.zeros((num_agents, 1))
            for i in range(num_agents):
                speed[i] = self.robot.X[4*i + 3, 0]
                omega[i] = self.u2_base.value[2*i + 1, 0]
        else:
            speed = np.zeros((num_agents, 1))
            omega = np.zeros((num_agents, 1))
        
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
        return speed, omega, h_human_min, h_obs_min
    
        # vels = self.robot.wheel_to_vel_g() @ np.array([ self.robot.X[3,0], self.u2_base.value[1,0] ]).reshape(-1,1)
        # return self.robot.X[3,0], vels[1,0]
    
def main():
    controller1 = multi_cbf_controller(np.array([3.0, 4.0, 0.0, 0.1]).reshape(-1,1), 0, 1, dynamic_alpha1=1.0, dynamic_alpha2=2.0)
    controller2 = multi_cbf_controller(np.array([3.0, 4.0, 0.0, 0.1, 4.0, 5.0, 0.2, 0.1]).reshape(-1,1), 0, 1, dynamic_alpha1=1.0, dynamic_alpha2=2.0)
    robot_goal = np.array([4.0, 5.0]).reshape(-1,1)
    other_robot_states = np.array([4.0, 5.0, 0.2, 0.1]).reshape(-1,1)
    output = controller2.robot.nominal_controller(robot_goal, other_robot_states, k_x = multi_cbf_controller.k_x, k_v = multi_cbf_controller.k_v )
    print(output)

if __name__ == '__main__':
    main()