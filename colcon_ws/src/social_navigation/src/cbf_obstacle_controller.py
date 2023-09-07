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
    alpha_polytope = 1.0
    robot = 0
    k_x = 2.0#30.0
    k_v = 3.5#1.5
    dt = 0.01
    controller2_base_layer = 0
    controller2_vol_layer = 0
    polytope_circle_volume_from_states_grad = []
    get_next_step_circle_volume_grad = []
    
    def __init__(self, robot_init_state, num_people, num_obstacles):

        # Sim parameters
        self.t = 0
        cbf_controller.dt = 0.03
        cbf_controller.num_people = num_people
        cbf_controller.num_obstacles = num_obstacles
        cbf_controller.alpha1_human = 2*np.ones(cbf_controller.num_people)
        cbf_controller.alpha2_human = 6*np.ones(cbf_controller.num_people)
        cbf_controller.alpha1_obstacle = 2*np.ones(cbf_controller.num_obstacles+1)
        cbf_controller.alpha2_obstacle = 6*np.ones(cbf_controller.num_obstacles+1)
        cbf_controller.alpha_polytope = 2.0
        self.control_bound = 3.0
        self.goal = np.array([-3.0,1.0]).reshape(-1,1)
        cbf_controller.k_x = 1.5#0.5#30.0
        cbf_controller.k_v = 3.0#1.5
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

        # cvxpy volume barrier cbf controller
        self.n_vol = 4 + cbf_controller.num_people + cbf_controller.num_obstacles + 1 
        self.u2_vol = cp.Variable((2,1))
        self.u2_ref_vol = cp.Parameter((2,1))
        self.A2_vol = cp.Parameter((self.n_vol,2))
        self.b2_vol = cp.Parameter((self.n_vol,1))
        self.const2_vol = [self.A2_vol @ self.u2_vol >= self.b2_vol]
        self.objective2_vol = cp.Minimize( cp.sum_squares(self.u2_vol-self.u2_ref_vol) )
        self.controller2_vol = cp.Problem( self.objective2_vol, self.const2_vol )
        cbf_controller.controller2_vol_layer = CvxpyLayer(self.controller2_vol, parameters=[self.u2_ref_vol, self.A2_vol, self.b2_vol], variables=[self.u2_vol])
        self.min_polytope_volume_circle = 0
        self.volume_circle2 = []
        ##################

        plt.ion()
        self.volume2 = []
        self.fig1, self.ax1 = plt.subplots( 1, 2, figsize=(9, 3), gridspec_kw={'width_ratios': [5, 5]})# )#, gridspec_kw={'height_ratios': [1, 1]} )
        self.ax1[0].set_xlim([-10,10])
        self.ax1[0].set_ylim([-10,10])
        self.offset = 3.0
        self.ax1[1].set_xlim([-self.control_bound-self.offset, self.control_bound+self.offset])
        self.ax1[1].set_ylim([-self.control_bound-self.offset, self.control_bound+self.offset])

        # Robot
        cbf_controller.robot = dynamic_unicycle( self.ax1[0], pos = np.array([ robot_init_state[0,0], robot_init_state[1,0], robot_init_state[2,0], 0.0 ]), dt = self.dt, plot_polytope=False )
        self.control_input_limit_points = np.array([ [self.control_bound, self.control_bound], [-self.control_bound, self.control_bound], [-self.control_bound, -self.control_bound], [self.control_bound, -self.control_bound] ])
        self.control_bound_polytope = pt.qhull( self.control_input_limit_points )
        print(f"hull: {self.control_bound_polytope}")
        
        # Volume grad
        cbf_controller.polytope_circle_volume_from_states_grad = grad( cbf_controller.compute_circle_volume_from_states, argnums = (0,1,2) )
        cbf_controller.get_next_step_circle_volume_grad = grad( cbf_controller.get_next_step_circle_volume, argnums=(0,1,2,3,4) )
        
    @staticmethod
    @jit
    def construct_barrier_from_states(robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius ):         
                # barrier function
                A = jnp.zeros((1,2)); b = jnp.zeros((1,1))
                for i in range(cbf_controller.num_people):
                    dh_dot_dx1, dh_dot_dx2, h_dot, h = cbf_controller.robot.barrier_humans_alpha_jax( robot_state, humans_states[:,i].reshape(-1,1), human_states_dot[:,i].reshape(-1,1), d_min = robot_radius+0.3)
                    A = jnp.append( A, dh_dot_dx1 @ cbf_controller.robot.g_jax(robot_state), axis = 0 )
                    b = jnp.append( b, - dh_dot_dx1 @ cbf_controller.robot.f_jax(robot_state) - dh_dot_dx2 @ human_states_dot[:,i].reshape(-1,1) - alpha1_human[i] * h_dot - alpha2_human[i] * (h_dot + alpha1_human[i]*h), axis = 0 )
                for i in range(cbf_controller.num_obstacles):
                    dh_dot_dx1, dh_dot_dx2, h_dot, h = cbf_controller.robot.barrier_alpha_jax( robot_state, obstacle_states[:,i].reshape(-1,1), d_min = robot_radius)
                    A = jnp.append( A, dh_dot_dx1 @ cbf_controller.robot.g_jax(robot_state), axis = 0 )
                    b = jnp.append( b, - dh_dot_dx1 @ cbf_controller.robot.f_jax(robot_state) - alpha1_obstacle[i] * h_dot - alpha2_obstacle[i] * (h_dot + alpha1_obstacle[i]*h), axis = 0 )
                return A[1:], b[1:]
    
    @staticmethod        
    @jit
    def circle_volume( r, c ):
        return jnp.pi * jnp.square(r)
            
    @staticmethod
    def compute_circle_from_states(robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius, control_A, control_B):
        A, b = cbf_controller.construct_barrier_from_states(robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius )
        A2 = jnp.append( -A, control_A, axis=0 )
        A2_root = jnp.linalg.norm( A2, axis=1 )
        b2 = jnp.append( -b, control_B, axis=0 )
        solution = circle_cvxpylayer( A2, A2_root, b2 )
        r = solution[0]
        c = solution[1]
        return r, c, cbf_controller.circle_volume( r, c )
    
    @staticmethod
    def compute_circle_volume_from_states(robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius, control_A, control_B):
        out = cbf_controller.compute_circle_from_states(robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius, control_A, control_B)[2]
        return out
        
    @staticmethod
    def get_next_step_circle( robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius, control_A, control_B, robot_goal, dt ):
        A, b = cbf_controller.construct_barrier_from_states(robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius)
        u_ref = cbf_controller.robot.nominal_controller_jax( robot_goal, k_x = cbf_controller.k_x, k_v = cbf_controller.k_v )
        A = jnp.append( A, -control_A, axis=0 )
        b = jnp.append( b, -control_B, axis=0 )
        solution = cbf_controller.controller2_base_layer(u_ref, A, b)
        robot_next_state = robot_state + cbf_controller.robot.xdot_jax(robot_state, solution[0]) * dt
        humans_next_state = humans_states + human_states_dot * dt
        out = cbf_controller.compute_circle_from_states( robot_next_state, humans_next_state, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius, control_A, control_B )        
        return out
    
    def get_next_step_circle_volume( robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius, control_A, control_B, robot_goal, dt ):
        return cbf_controller.get_next_step_circle( robot_state, humans_states, human_states_dot, obstacle_states, alpha1_human, alpha2_human, alpha1_obstacle, alpha2_obstacle, robot_radius, control_A, control_B, robot_goal, dt )[2]
        
    def policy_nominal(self, robot_state, robot_goal, robot_radius, human_states, human_states_dot, dt):
        
        self.robot.set_state(robot_state)
        self.u2_ref_base.value = cbf_controller.robot.nominal_controller( robot_goal, k_x = cbf_controller.k_x, k_v = cbf_controller.k_v )
        self.robot.step( self.u2_ref_base.value, dt )
        return self.robot.X[3,0], self.u2_ref_base.value[1,0]
    
    
    def policy_cbf(self, robot_state, robot_goal, robot_radius, human_states, human_states_dot, obstacle_states, dt):
        
        self.robot.set_state(robot_state)
        A, b = self.construct_barrier_from_states(jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.array(obstacle_states), self.alpha1_human, self.alpha2_human, self.alpha1_obstacle, self.alpha2_obstacle, robot_radius )
        self.A2_base.value = np.append( np.asarray(A), -self.control_bound_polytope.A, axis=0 )
        self.b2_base.value = np.append( np.asarray(b), -self.control_bound_polytope.b.reshape(-1,1), axis=0 )
        self.u2_ref_base.value = cbf_controller.robot.nominal_controller( robot_goal, k_x = cbf_controller.k_x, k_v = cbf_controller.k_v )
        self.controller2_base.solve(solver=cp.GUROBI)
        self.robot.step( self.u2_base.value, dt )         
        
        # Plot polytope       
        self.ax1[1].clear()
        self.ax1[1].set_xlim([-self.control_bound-self.offset, self.control_bound+self.offset])
        self.ax1[1].set_ylim([-self.control_bound-self.offset, self.control_bound+self.offset])
        hull = pt.Polytope( -self.A2_base.value, -self.b2_base.value )
        hull_plot = hull.plot(self.ax1[1], color = 'g')
        plot_polytope_lines( self.ax1[1], hull, self.control_bound )
        self.ax1[0].clear()
        self.ax1[0].scatter(human_states[0,:], human_states[1,:], c = 'g')
        self.ax1[0].scatter(robot_state[0,0], robot_state[1,0], c = 'r')    
        self.ax1[0].scatter(obstacle_states[0,:], obstacle_states[1,:], c = 'k')
        self.ax1[0].set_xlim([robot_state[0,0]-5, robot_state[0,0]+5])
        self.ax1[0].set_ylim([robot_state[1,0]-5, robot_state[1,0]+5])
        self.ax1[1].scatter(self.u2_ref_base.value[0,0], self.u2_ref_base.value[1,0],s = 70, edgecolors='r', facecolors='none' )
        self.ax1[1].scatter(self.u2_base.value[0,0], self.u2_base.value[1,0],s = 50, edgecolors='r', facecolors='r' )
        
        self.fig1.canvas.draw()
        self.fig1.canvas.flush_events
        return self.robot.X[3,0], self.u2_base.value[1,0]
    
    def policy_cbf_volume(self, robot_state, robot_goal, robot_radius, human_states, human_states_dot, obstacle_states, dt):
        self.robot.set_state(robot_state)
        A, b = self.construct_barrier_from_states(jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.array(obstacle_states), self.alpha1_human, self.alpha2_human, self.alpha1_obstacle, self.alpha2_obstacle, robot_radius )
        A = np.append( np.asarray(A), -self.control_bound_polytope.A, axis=0 )
        b = np.append( np.asarray(b), -self.control_bound_polytope.b.reshape(-1,1), axis=0 )
        self.u2_ref_vol.value = cbf_controller.robot.nominal_controller( robot_goal, k_x = cbf_controller.k_x, k_v = cbf_controller.k_v )
        circle_r2, circle_c2, volume_new = cbf_controller.compute_circle_from_states(jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.array(obstacle_states), self.alpha1_human, self.alpha2_human, self.alpha1_obstacle, self.alpha2_obstacle, robot_radius, jnp.asarray(self.control_bound_polytope.A), jnp.asarray(self.control_bound_polytope.b.reshape(-1,1)) )
        volume_grad_robot, volume_grad_humansX, volume_grad_humansU = cbf_controller.polytope_circle_volume_from_states_grad(jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.array(obstacle_states), self.alpha1_human, self.alpha2_human, self.alpha1_obstacle, self.alpha2_obstacle, robot_radius, jnp.asarray(self.control_bound_polytope.A), jnp.asarray(self.control_bound_polytope.b.reshape(-1,1)))
        # print(f" polytope_volume_from_states_grad: {volume_grad_robot} ")
        h_polytope = volume_new - self.min_polytope_volume_circle
        dh_polytope_dx = volume_grad_robot.T
        A_polytope = np.asarray(dh_polytope_dx @ self.robot.g())
        b_polytope = np.asarray(- dh_polytope_dx @ self.robot.f() - self.alpha_polytope * h_polytope - np.sum(volume_grad_humansX * human_states_dot ))
        
        # Plot polytope       
        self.ax1[1].clear()
        self.ax1[1].set_xlim([-self.control_bound-self.offset, self.control_bound+self.offset])
        self.ax1[1].set_ylim([-self.control_bound-self.offset, self.control_bound+self.offset])
        hull = pt.Polytope( -A, -b )
        hull_plot = hull.plot(self.ax1[1], color = 'g')
        plot_polytope_lines( self.ax1[1], hull, self.control_bound )
        self.ax1[0].clear()
        self.ax1[0].scatter(human_states[0,:], human_states[1,:], c = 'g')
        self.ax1[0].scatter(robot_state[0,0], robot_state[1,0], c = 'r')  
        self.ax1[0].scatter(obstacle_states[0,:], obstacle_states[1,:], c = 'k')  
        self.ax1[0].set_xlim([robot_state[0,0]-5, robot_state[0,0]+5])
        self.ax1[0].set_ylim([robot_state[1,0]-5, robot_state[1,0]+5])
        
        self.A2_vol.value = np.append( A, A_polytope, axis=0 )
        self.b2_vol.value = np.append( b, b_polytope, axis=0 )
        
        self.controller2_vol.solve(solver=cp.GUROBI)
        self.robot.step( self.u2_vol.value, dt )   
        if 1:#plot_circle:
            angles   = np.linspace( 0, 2 * np.pi, 100 )
            circle_inner = circle_c2 + circle_r2 * np.append(np.cos(angles).reshape(1,-1) , np.sin(angles).reshape(1,-1), axis=0 )
            self.ax1[1].plot( circle_inner[0,:], circle_inner[1,:], 'c--', label='Inner Circle' )
            self.ax1[1].scatter(self.u2_ref_vol.value[0,0], self.u2_ref_vol.value[1,0],s = 70, edgecolors='r', facecolors='none' )
            self.ax1[1].scatter(self.u2_vol.value[0,0], self.u2_vol.value[1,0],s = 50, edgecolors='r', facecolors='r' )
        
        self.fig1.canvas.draw()
        self.fig1.canvas.flush_events
        return self.robot.X[3,0], self.u2_vol.value[1,0]
    
    def policy_cbf_volume_adapt(self, robot_state, robot_goal, robot_radius, human_states, human_states_dot, dt):

        self.robot.set_state(robot_state)
        
        A, b = self.construct_barrier_from_states(jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), self.alpha1_human, self.alpha2_human, robot_radius )
        A = np.append( np.asarray(A), -self.control_bound_polytope.A, axis=0 )
        b = np.append( np.asarray(b), -self.control_bound_polytope.b.reshape(-1,1), axis=0 )

        # Update Parameters
        circle_r2, circle_c2, volume_new = cbf_controller.get_next_step_circle( jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), self.alpha1_human, self.alpha2_human, robot_radius, jnp.asarray(self.control_bound_polytope.A), jnp.asarray(self.control_bound_polytope.b.reshape(-1,1)), jnp.asarray(robot_goal), dt )
        volume_grad_robot, volume_grad_humansX, volume_grad_humansU, volume_grad_alpha1, volume_grad_alpha2 = cbf_controller.get_next_step_circle_volume_grad( jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), self.alpha1_human, self.alpha2_human, robot_radius, jnp.asarray(self.control_bound_polytope.A), jnp.asarray(self.control_bound_polytope.b.reshape(-1,1)), jnp.asarray(robot_goal), dt )
        # print(f" polytope_volume_from_states_grad: {volume_grad_robot} ")
        
        self.alpha1_human = np.clip(self.alpha1_human + np.clip(volume_grad_alpha1,-2,2) * dt, 0, 20)
        self.alpha2_human = np.clip(self.alpha2_human + np.clip(volume_grad_alpha2,-2,2) * dt, 0, 20)
        
        # print(f"grads alpha1:{volume_grad_alpha1}, alpha2:{volume_grad_alpha2}, alpha1:{self.alpha1_human}, alpha2:{self.alpha2_human}")
        
        self.A2_base.value = A
        self.b2_base.value = b
        self.u2_ref_base.value = cbf_controller.robot.nominal_controller( robot_goal, k_x = cbf_controller.k_x, k_v = cbf_controller.k_v )
        self.controller2_base.solve(solver=cp.GUROBI)
        self.robot.step( self.u2_base.value, dt )         
       
        # Plot polytope       
        self.ax1[1].clear()
        self.ax1[1].set_xlim([-self.control_bound-self.offset, self.control_bound+self.offset])
        self.ax1[1].set_ylim([-self.control_bound-self.offset, self.control_bound+self.offset])
        hull = pt.Polytope( -A, -b )
        hull_plot = hull.plot(self.ax1[1], color = 'g')
        plot_polytope_lines( self.ax1[1], hull, self.control_bound )
        self.ax1[0].clear()
        self.ax1[0].scatter(human_states[0,:], human_states[1,:], c = 'g')
        self.ax1[0].scatter(robot_state[0,0], robot_state[1,0], c = 'r')    
        
        if 0:#plot_circle:
            angles   = np.linspace( 0, 2 * np.pi, 100 )
            circle_inner = circle_c2 + circle_r2 * np.append(np.cos(angles).reshape(1,-1) , np.sin(angles).reshape(1,-1), axis=0 )
            self.ax1[1].plot( circle_inner[0,:], circle_inner[1,:], 'c--', label='Inner Circle' )
        
        self.fig1.canvas.draw()
        self.fig1.canvas.flush_events
        # print(f"hello, u:{self.u2_base}")
        return self.robot.X[3,0], self.u2_base.value[1,0]


# Stop the robot when the controller fails
# The collision then is human's fault
# how many times robot was able to stop before the collision takes place