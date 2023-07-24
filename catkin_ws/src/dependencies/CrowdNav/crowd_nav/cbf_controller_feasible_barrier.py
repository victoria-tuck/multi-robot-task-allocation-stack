import time
import numpy as np
import cvxpy as cp
import polytope as pt
import matplotlib.pyplot as plt

from bicycle_new import bicycle
from polytope_utils import *
from matplotlib.animation import FFMpegWriter
import jax.numpy as jnp

# Sim parameters
t = 0
dt = 0.03
tf = 15
alpha2 =4.0#5#2.0#3.0#20#6
alpha1 = 1.0#20#4.0#0.5#50#2
control_bound = 2.0
goal = np.array([-3.0,-1.0]).reshape(-1,1)
num_people = 5
k_v = 1.5
d_min_human = 0.5
d_min_human = 0.5
use_ellipse = False
plot_ellipse = False
use_circle = True
plot_circle = True
alpha_polytope = 1.0
min_polytope_volume_ellipse = -0.5
min_polytope_volume_circle = 0.0

######### holonomic controller
n = 4 + num_people + 1 # number of constraints
u2 = cp.Variable((2,1))
u2_ref = cp.Parameter((2,1))
objective2 = cp.Minimize( cp.sum_squares( u2 - u2_ref ) )
A2 = cp.Parameter((n,2))
b2 = cp.Parameter((n,1))
const2 = [A2 @ u2 >= b2]
controller2 = cp.Problem( objective2, const2 )

human_states = np.zeros((2,num_people))
human_states_dot = np.zeros((2,num_people))
##########

plt.ion()
fig1, ax1 = plt.subplots( 1, 3, figsize=(18, 6), gridspec_kw={'width_ratios': [5, 5, 2]})# )#, gridspec_kw={'height_ratios': [1, 1]} )
ax1[0].set_xlim([-3,5])
ax1[0].set_ylim([-3,5])
offset = 3.0
ax1[1].set_xlim([-control_bound-offset, control_bound+offset])
ax1[1].set_ylim([-control_bound-offset, control_bound+offset])

robot = bicycle( ax1[0], pos = np.array([ 1.0, 1.0, np.pi, 1.3 ]), dt = dt, plot_polytope=False )
control_input_limit_points = np.array([ [control_bound, control_bound], [-control_bound, control_bound], [-control_bound, -control_bound], [control_bound, -control_bound] ])
control_bound_polytope = pt.qhull( control_input_limit_points )

volume2 = []
volume_ellipse2 = []
volume_circle2 = []

@jit
def construct_barrier_from_states(robot_state, humans_states, human_states_dot ):         
            # barrier function
            A = jnp.zeros((1,2)); b = jnp.zeros((1,1))    
            # for i in range(humans_states.shape[1]):
            for i in range(num_people):
                h, dh_dx, _ = robot.barrier_humans_jax( robot_state, humans_states[:,i].reshape(-1,1), human_states_dot[:,i].reshape(-1,1), d_min = d_min_humans, alpha1 = alpha1 )
                A = jnp.append( A, dh_dx @ robot.g_jax(robot_state), axis = 0 )
                b = jnp.append( b, - alpha2 * h - dh_dx @ robot.f_jax(robot_state), axis = 0 )
            return A[1:], b[1:]

@jit       
def ellipse_volume( B, d ):
     return jnp.log( jnp.linalg.det(B) )

@jit
def circle_volume( r, c ):
    return jnp.pi * jnp.square(r)

def compute_ellipse_from_states(robot_state, humans_states, human_states_dot, control_A, control_b):
    A, b = construct_barrier_from_states(robot_state, humans_states, human_states_dot )
    A2 = jnp.append( -A, control_A, axis=0 )
    b2 = jnp.append( -b, control_b, axis=0 )
    solution = ellipse_cvxpylayer( A2, b2 )
    B = solution[0]
    d = solution[1]
    return B, d, ellipse_volume( B, d )

def compute_circle_from_states(robot_state, humans_states, human_states_dot, control_A, control_b):
    A, b = construct_barrier_from_states(robot_state, humans_states, human_states_dot )
    A2 = jnp.append( -A, control_A, axis=0 )
    A2_root = jnp.linalg.norm( A2, axis=1 )
    b2 = jnp.append( -b, control_b, axis=0 )
    solution = circle_cvxpylayer( A2, A2_root, b2 )
    r = solution[0]
    c = solution[1]
    return r, c, circle_volume( r, c )

def polytope_ellipse_volume_from_states(robot_state, humans_states, human_states_dot, control_A, control_b):
    return compute_ellipse_from_states(robot_state, humans_states, human_states_dot, control_A, control_b)[2]

def polytope_circle_volume_from_states(robot_state, humans_states, human_states_dot, control_A, control_b):
    return compute_circle_from_states(robot_state, humans_states, human_states_dot, control_A, control_b)[2]
 
polytope_ellipse_volume_from_states_grad = grad( polytope_ellipse_volume_from_states, argnums=(0,1,2) )
# polytope_ellipse_volume_from_states_grad2 = grad( polytope_ellipse_volume_from_states, argnums=(0) )

polytope_circle_volume_from_states_grad = grad( polytope_circle_volume_from_states, argnums=(0,1,2) )
    
while t < tf:

    # desired input
    u2_ref.value = robot.nominal_controller( goal, k_v = k_v )
    
    A, b = construct_barrier_from_states(jnp.asarray(robot.X), jnp.asarray(human_states), jnp.asarray(human_states_dot) )
    A = np.append( np.asarray(A), -control_bound_polytope.A, axis=0 )
    b = np.append( np.asarray(b), -control_bound_polytope.b.reshape(-1,1), axis=0 )
    ax1[1].clear()
    ax1[1].set_xlim([-control_bound-offset, control_bound+offset])
    ax1[1].set_ylim([-control_bound-offset, control_bound+offset])
    hull = pt.Polytope( -A, -b )
    hull_plot = hull.plot(ax1[1], color = 'g')
    plot_polytope_lines( ax1[1], hull, control_bound )
    
    volume2.append(np.array(mc_polytope_volume( jnp.array(hull.A), jnp.array(hull.b.reshape(-1,1)), bounds = control_bound)))
    ax1[2].plot( volume2, 'g' )
    ax1[2].set_title('Polytope Volume')
    # print(f"GRAD : { mc_polytope_volume_grad( jnp.array(hull.A), jnp.array(hull.b.reshape(-1,1)), bounds = control_bound, num_samples=50000 ) } ")

    
    if use_ellipse:
        ellipse_B2, ellipse_d2, volume_new = compute_ellipse_from_states(jnp.asarray(robot.X), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.asarray(control_bound_polytope.A), jnp.asarray(control_bound_polytope.b.reshape(-1,1)) )
        if plot_ellipse:
            angles   = np.linspace( 0, 2 * np.pi, 100 )
            ellipse_inner  = (ellipse_B2 @ np.append(np.cos(angles).reshape(1,-1) , np.sin(angles).reshape(1,-1), axis=0 )) + ellipse_d2# * np.ones( 1, noangles );
            ellipse_outer  = (2* ellipse_B2 @ np.append(np.cos(angles).reshape(1,-1) , np.sin(angles).reshape(1,-1), axis=0 )) + ellipse_d2
            volume_ellipse2.append(volume_new)
            ax1[2].plot( volume_ellipse2, 'g--' )
            ax1[1].plot( ellipse_inner[0,:], ellipse_inner[1,:], 'c--', label='Jax Inner Ellipse' )
            ax1[1].plot( ellipse_outer[0,:], ellipse_outer[1,:], 'c--', label='Jax Outer Ellipse' )

        volume_grad_robot, volume_grad_humansX, volume_grad_humansU = polytope_ellipse_volume_from_states_grad(jnp.asarray(robot.X), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.asarray(control_bound_polytope.A), jnp.asarray(control_bound_polytope.b.reshape(-1,1)))
        print(f" polytope_volume_from_states_grad: {volume_grad_robot} ")
        
        h_polytope = volume_new - min_polytope_volume_ellipse
        dh_polytope_dx = volume_grad_robot.T
        A_polytope = np.asarray(dh_polytope_dx @ robot.g())
        b_polytope = np.asarray(- dh_polytope_dx @ robot.f() - alpha_polytope * h_polytope - np.sum(volume_grad_humansX * human_states_dot ))
        A2.value = np.append( A, np.asarray(A_polytope), axis=0 )
        b2.value = np.append( b, np.asarray(b_polytope), axis=0 )
    elif use_circle:
        circle_r2, circle_c2, volume_new = compute_circle_from_states(jnp.asarray(robot.X), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.asarray(control_bound_polytope.A), jnp.asarray(control_bound_polytope.b.reshape(-1,1)) )
        if plot_circle:
            angles   = np.linspace( 0, 2 * np.pi, 100 )
            circle_inner = circle_c2 + circle_r2 * np.append(np.cos(angles).reshape(1,-1) , np.sin(angles).reshape(1,-1), axis=0 )
            volume_circle2.append(volume_new)
            ax1[2].plot( volume_circle2, 'g--' )
            ax1[1].plot( circle_inner[0,:], circle_inner[1,:], 'c--', label='Inner Circle' )
        volume_grad_robot, volume_grad_humansX, volume_grad_humansU = polytope_circle_volume_from_states_grad(jnp.asarray(robot.X), jnp.asarray(human_states), jnp.asarray(human_states_dot), jnp.asarray(control_bound_polytope.A), jnp.asarray(control_bound_polytope.b.reshape(-1,1)))
        print(f" polytope_volume_from_states_grad: {volume_grad_robot} ")
        
        h_polytope = volume_new - min_polytope_volume_circle
        dh_polytope_dx = volume_grad_robot.T
        A_polytope = np.asarray(dh_polytope_dx @ robot.g())
        b_polytope = np.asarray(- dh_polytope_dx @ robot.f() - alpha_polytope * h_polytope - np.sum(volume_grad_humansX * human_states_dot ))
        A2.value = np.append( A, np.asarray(A_polytope), axis=0 )
        b2.value = np.append( b, np.asarray(b_polytope), axis=0 )
    
    controller2.solve()
    if controller2.status == 'infeasible':
        print(f"QP infeasible")
        exit()
    robot.step( u2.value )
    
    # robot.step( u2_ref.value )
    robot.render_plot()

    ax1[1].set_xlabel('Linear Acceleration'); ax1[1].set_ylabel('Angular Velocity')
    # ax1[1].set_xlabel(r'$u_x$'); ax1[1].set_ylabel(r'$u_y$')
    ax1[1].scatter( u2.value[0,0], u2.value[1,0], c = 'r', label = 'CBF-QP chosen control' )
    ax1[1].legend()
    ax1[1].set_title('Feasible Space for Control')

    fig1.canvas.draw()
    fig1.canvas.flush_events()

    t = t + dt



