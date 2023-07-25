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

    def __init__(self, robot_init_state, dt):

        # Sim parameters
        self.t = 0
        self.dt = dt
        self.tf = 15
        self.alpha2 =4.0#5#2.0#3.0#20#6
        self.alpha1 = 1.0#20#4.0#0.5#50#2
        self.control_bound = 2.0
        self.goal = np.array([-3.0,-1.0]).reshape(-1,1)
        self.num_people = humans_init_state.shape[1]
        self.k_v = 1.5
        self.d_min_human = 0.5
        self.d_min_human = 0.5
        self.use_ellipse = False
        self.plot_ellipse = False
        self.use_circle = True
        self.plot_circle = True
        self.alpha_polytope = 1.0
        self.min_polytope_volume_ellipse = -0.5
        self.min_polytope_volume_circle = 0.0

        ######### holonomic controller
        self.n = 4 + self.num_people + 1 # number of constraints
        self.u2 = cp.Variable((2,1))
        self.u2_ref = cp.Parameter((2,1))
        self.objective2 = cp.Minimize( cp.sum_squares( self.u2 - self.u2_ref ) )
        self.A2 = cp.Parameter((self.n,2))
        self.b2 = cp.Parameter((self.n,1))
        self.const2 = [self.A2 @ self.u2 >= self.b2]
        self.controller2 = cp.Problem( self.objective2, self.const2 )

        self.human_states = np.zeros((2,self.num_people))
        self.human_states_dot = np.zeros((2,self.num_people))
        ##########

        plt.ion()
        self.fig1, self.ax1 = plt.subplots( 1, 3, figsize=(18, 6), gridspec_kw={'width_ratios': [5, 5, 2]})# )#, gridspec_kw={'height_ratios': [1, 1]} )
        self.ax1[0].set_xlim([-3,5])
        self.ax1[0].set_ylim([-3,5])
        self.offset = 3.0
        self.ax1[1].set_xlim([-self.control_bound-self.offset, self.control_bound+self.offset])
        self.ax1[1].set_ylim([-self.control_bound-self.offset, self.control_bound+self.offset])

        self.robot = dynamic_unicycle( self.ax1[0], pos = np.array([ robot_init_state[0,0], robot_init_state[1,0], robot_init_state[2,0], 0.0 ]), dt = self.dt, plot_polytope=False )
        self.control_input_limit_points = np.array([ [self.control_bound, self.control_bound], [-self.control_bound, self.control_bound], [-self.control_bound, -self.control_bound], [self.control_bound, -self.control_bound] ])
        self.control_bound_polytope = pt.qhull( self.control_input_limit_points )

        self.volume2 = []
        self.volume_ellipse2 = []
        self.volume_circle2 = []

        self.polytope_ellipse_volume_from_states_grad = grad( self.polytope_ellipse_volume_from_states, argnums=(0,1,2) )
        # polytope_ellipse_volume_from_states_grad2 = grad( polytope_ellipse_volume_from_states, argnums=(0) )

        self.polytope_circle_volume_from_states_grad = grad( self.polytope_circle_volume_from_states, argnums=(0,1,2) )

    @jit
    def construct_barrier_from_states(self, robot_state, humans_states, human_states_dot, robot_radius ):         
                # barrier function
                A = jnp.zeros((1,2)); b = jnp.zeros((1,1))    
                # for i in range(humans_states.shape[1]):
                for i in range(self.num_people):
                    h, dh_dx, _ = self.robot.barrier_humans_jax( robot_state, humans_states[:,i].reshape(-1,1), human_states_dot[:,i].reshape(-1,1), d_min = robot_radius, alpha1 = self.alpha1 )#self.d_min_humans
                    A = jnp.append( A, dh_dx @ self.robot.g_jax(robot_state), axis = 0 )
                    b = jnp.append( b, - self.alpha2 * h - dh_dx @ self.robot.f_jax(robot_state), axis = 0 )
                return A[1:], b[1:]

    @jit       
    def ellipse_volume( self, B, d ):
        return jnp.log( jnp.linalg.det(B) )

    @jit
    def circle_volume( self, r, c ):
        return jnp.pi * jnp.square(r)

    def compute_ellipse_from_states(self, robot_state, humans_states, human_states_dot, control_A, control_b, robot_radius):
        A, b = self.construct_barrier_from_states(robot_state, humans_states, human_states_dot, robot_radius )
        A2 = jnp.append( -A, control_A, axis=0 )
        b2 = jnp.append( -b, control_b, axis=0 )
        solution = ellipse_cvxpylayer( A2, b2 )
        B = solution[0]
        d = solution[1]
        return B, d, self.ellipse_volume( B, d )

    def compute_circle_from_states(self, robot_state, humans_states, human_states_dot, control_A, control_b, robot_radius):
        A, b = self.construct_barrier_from_states(robot_state, humans_states, human_states_dot, robot_radius )
        A2 = jnp.append( -A, control_A, axis=0 )
        A2_root = jnp.linalg.norm( A2, axis=1 )
        b2 = jnp.append( -b, control_b, axis=0 )
        solution = circle_cvxpylayer( A2, A2_root, b2 )
        r = solution[0]
        c = solution[1]
        return r, c, self.circle_volume( r, c )

    def polytope_ellipse_volume_from_states(self,robot_state, humans_states, human_states_dot, control_A, control_b, robot_radius):
        return self.compute_ellipse_from_states(robot_state, humans_states, human_states_dot, control_A, control_b, robot_radius)[2]

    def polytope_circle_volume_from_states(self,robot_state, humans_states, human_states_dot, control_A, control_b, robot_radius):
        return self.compute_circle_from_states(robot_state, humans_states, human_states_dot, control_A, control_b, robot_radius)[2]
    
    
    def policy(self, robot_state, robot_goal, robot_radius, human_states, human_states_dot):
        self.robot.set_state(robot_state)
        self.u2_ref.value = self.robot.nominal_controller( robot_goal, k_v = self.k_v )
        
        A, b = self.construct_barrier_from_states(jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot) , robot_radius)
        A = np.append( np.asarray(A), -self.control_bound_polytope.A, axis=0 )
        b = np.append( np.asarray(b), -self.control_bound_polytope.b.reshape(-1,1), axis=0 )
        self.ax1[1].clear()
        self.ax1[1].set_xlim([-self.control_bound-self.offset, self.control_bound+self.offset])
        self.ax1[1].set_ylim([-self.control_bound-self.offset, self.control_bound+self.offset])
        hull = pt.Polytope( -A, -b )
        hull_plot = hull.plot(self.ax1[1], color = 'g')
        plot_polytope_lines( self.ax1[1], hull, self.control_bound )
        
        self.volume2.append(np.array(mc_polytope_volume( jnp.array(hull.A), jnp.array(hull.b.reshape(-1,1)), bounds = self.control_bound)))
        self.ax1[2].plot( self.volume2, 'g' )
        self.ax1[2].set_title('Polytope Volume')
        # print(f"GRAD : { mc_polytope_volume_grad( jnp.array(hull.A), jnp.array(hull.b.reshape(-1,1)), bounds = control_bound, num_samples=50000 ) } ")

        
        if self.use_ellipse:
            ellipse_B2, ellipse_d2, volume_new = self.compute_ellipse_from_states(jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.asarray(self.control_bound_polytope.A), jnp.asarray(self.control_bound_polytope.b.reshape(-1,1)), robot_radius )
            if self.plot_ellipse:
                angles   = np.linspace( 0, 2 * np.pi, 100 )
                ellipse_inner  = (ellipse_B2 @ np.append(np.cos(angles).reshape(1,-1) , np.sin(angles).reshape(1,-1), axis=0 )) + ellipse_d2# * np.ones( 1, noangles );
                ellipse_outer  = (2* ellipse_B2 @ np.append(np.cos(angles).reshape(1,-1) , np.sin(angles).reshape(1,-1), axis=0 )) + ellipse_d2
                self.volume_ellipse2.append(volume_new)
                self.ax1[2].plot( self.volume_ellipse2, 'g--' )
                self.ax1[1].plot( ellipse_inner[0,:], ellipse_inner[1,:], 'c--', label='Jax Inner Ellipse' )
                self.ax1[1].plot( ellipse_outer[0,:], ellipse_outer[1,:], 'c--', label='Jax Outer Ellipse' )

            volume_grad_robot, volume_grad_humansX, volume_grad_humansU = self.polytope_ellipse_volume_from_states_grad(jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.asarray(self.control_bound_polytope.A), jnp.asarray(self.control_bound_polytope.b.reshape(-1,1)), robot_radius)
            print(f" polytope_volume_from_states_grad: {volume_grad_robot} ")
            
            h_polytope = volume_new - self.min_polytope_volume_ellipse
            dh_polytope_dx = volume_grad_robot.T
            A_polytope = np.asarray(dh_polytope_dx @ self.robot.g())
            b_polytope = np.asarray(- dh_polytope_dx @ self.robot.f() - self.alpha_polytope * h_polytope - np.sum(volume_grad_humansX * human_states_dot ))
            self.A2.value = np.append( A, np.asarray(A_polytope), axis=0 )
            self.b2.value = np.append( b, np.asarray(b_polytope), axis=0 )
        elif self.use_circle:
            circle_r2, circle_c2, volume_new = self.compute_circle_from_states(jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.asarray(self.control_bound_polytope.A), jnp.asarray(self.control_bound_polytope.b.reshape(-1,1)), robot_radius )
            if self.plot_circle:
                angles   = np.linspace( 0, 2 * np.pi, 100 )
                circle_inner = circle_c2 + circle_r2 * np.append(np.cos(angles).reshape(1,-1) , np.sin(angles).reshape(1,-1), axis=0 )
                self.volume_circle2.append(volume_new)
                self.ax1[2].plot( self.volume_circle2, 'g--' )
                self.ax1[1].plot( circle_inner[0,:], circle_inner[1,:], 'c--', label='Inner Circle' )
            volume_grad_robot, volume_grad_humansX, volume_grad_humansU = self.polytope_circle_volume_from_states_grad(jnp.asarray(robot_state), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.asarray(self.control_bound_polytope.A), jnp.asarray(self.control_bound_polytope.b.reshape(-1,1)), robot_radius)
            print(f" polytope_volume_from_states_grad: {volume_grad_robot} ")
            
            h_polytope = volume_new - self.min_polytope_volume_circle
            dh_polytope_dx = volume_grad_robot.T
            A_polytope = np.asarray(dh_polytope_dx @ self.robot.g())
            b_polytope = np.asarray(- dh_polytope_dx @ self.robot.f() - self.alpha_polytope * h_polytope - np.sum(volume_grad_humansX * human_states_dot ))
            self.A2.value = np.append( A, np.asarray(A_polytope), axis=0 )
            self.b2.value = np.append( b, np.asarray(b_polytope), axis=0 )
        
        self.controller2.solve()
        if self.controller2.status == 'infeasible':
            print(f"QP infeasible")
            exit()
        self.robot.step( self.u2.value )
        
        # robot.step( u2_ref.value )
        self.robot.render_plot()

        self.ax1[1].set_xlabel('Linear Acceleration'); self.ax1[1].set_ylabel('Angular Velocity')
        # ax1[1].set_xlabel(r'$u_x$'); ax1[1].set_ylabel(r'$u_y$')
        self.ax1[1].scatter( self.u2.value[0,0], self.u2.value[1,0], c = 'r', label = 'CBF-QP chosen control' )
        self.ax1[1].legend()
        self.ax1[1].set_title('Feasible Space for Control')

        self.fig1.canvas.draw()
        self.fig1.canvas.flush_events()

        return np.array([ self.robot.X[3,0], self.u2.value[1,0] ]).reshape(-1,1)



