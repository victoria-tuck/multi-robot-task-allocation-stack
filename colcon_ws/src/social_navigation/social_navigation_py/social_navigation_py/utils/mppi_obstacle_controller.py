# import os
# os.environ['XLA_FLAGS'] = '--xla_force_host_platform_device_count=100'
import jax
import jax.numpy as jnp
import numpy as np
from jax.random import multivariate_normal
from jax import jit, lax
from .ut_utils_jax import *
import time

class MPPI_FORESEE():

    """
    Model Predictive Path Integral control
    This implementation batch samples the trajectories and so scales well with the number of samples K.
    """

    # MPPI implementation parameters
    samples = []
    horizon = []
    dt = 0.05  
    std_factor = 2.0
    use_gpu = True
    key = 0
    costs_lambda = 300
    cost_goal_coeff = 1.0
    cost_safety_coeff = 10.0    

    # Robot parameters/variables
    robot_n = 3
    robot_m = 2
    control_mu = 0
    conrtol_cov = 0
    control_bound = 0
    control_bound_lb = 0
    control_bound_ub = 0

    # Human parameters/variables
    num_humans = 5
    human_n = 2
    human_nominal_speed = jnp.tile(jnp.array([-3.0,0]).reshape(-1,1), (1,num_humans))
    human_repulsion_gain = 2.0
    indices = 0
    hindex_list = []
    aware = [True, True]
    humans_interact = True
    obstacles_interact = True
    sensing_radius = 2
    human_noise_cov = 0.01

    # Obstacle parameters/variables
    num_obstacles = 2
    obstacle_radius = 1.0
    perturbation_bound = 1.0

    def __init__(self, horizon=10, samples = 10, input_size = 2, dt=0.05, sensing_radius=2, human_noise_cov=0.01, std_factor=1.96, control_bound=7, control_init_ratio=1, u_guess=None, use_GPU=True, human_nominal_speed = jnp.array([-3.0,0]).reshape(-1,1), human_repulsion_gain = 2.0, costs_lambda = 300, cost_goal_coeff = 1.0, cost_safety_coeff = 10.0, num_humans = 5, num_obstacles = 2, aware=[True, True], humans_interact=True, obstacles_interact=True):
        MPPI_FORESEE.key = jax.random.PRNGKey(111)
        MPPI_FORESEE.human_n = 2
        MPPI_FORESEE.robot_n = 3 #2
        MPPI_FORESEE.m = 2
        MPPI_FORESEE.horizon = horizon
        MPPI_FORESEE.samples = samples
        MPPI_FORESEE.sensing_radius = sensing_radius
        MPPI_FORESEE.human_noise_cov = human_noise_cov
        MPPI_FORESEE.dt = dt
        MPPI_FORESEE.std_factor = std_factor
        MPPI_FORESEE.use_gpu = use_GPU
        MPPI_FORESEE.human_nominal_speed = human_nominal_speed
        MPPI_FORESEE.human_repulsion_gain = human_repulsion_gain
        MPPI_FORESEE.costs_lambda = costs_lambda
        MPPI_FORESEE.cost_goal_coeff = cost_goal_coeff
        MPPI_FORESEE.cost_safety_coeff = cost_safety_coeff
        MPPI_FORESEE.num_humans = num_humans
        MPPI_FORESEE.num_obstacles = num_obstacles
        MPPI_FORESEE.indices = jnp.arange(0, MPPI_FORESEE.num_humans)
        MPPI_FORESEE.hindex_list = jnp.arange(1,num_humans).reshape(1,-1)
        for i in range(1,num_humans):
            MPPI_FORESEE.hindex_list = jnp.append( MPPI_FORESEE.hindex_list, jnp.delete( jnp.arange(0,num_humans), i ).reshape(1,-1), axis=0 )    
        MPPI_FORESEE.aware = aware
        MPPI_FORESEE.humans_interact = humans_interact
        MPPI_FORESEE.obstacles_interact = obstacles_interact

        self.input_size = input_size        
        MPPI_FORESEE.control_bound = control_bound
        MPPI_FORESEE.control_mu = jnp.zeros(input_size)
        MPPI_FORESEE.control_cov = 4.0 * jnp.eye(input_size)  #2.0 * jnp.eye(input_size)
        MPPI_FORESEE.control_bound_lb = -jnp.array([[1], [1]])
        MPPI_FORESEE.control_bound_ub = -self.control_bound_lb  
        self.U = u_guess
        # if u_guess != None:
        #     self.U = u_guess
        # else:
        #     self.U = jnp.append(  1.0 * jnp.ones((MPPI_FORESEE.horizon, 1)), 1.0 * jnp.ones((MPPI_FORESEE.horizon,1)), axis=1  ) # T x nu

    # Linear dynamics for now
    @staticmethod
    def robot_dynamics_step(state, input):
        # Unicycle
        theta = state[2,0]
        xdot = jnp.array([ [ input[0,0]*jnp.cos(theta) ],
                            [ input[0,0]*jnp.sin(theta) ],
                            [ input[1,0] ]
                            ])
        return state + xdot * MPPI_FORESEE.dt
    
    @staticmethod
    @jit
    def single_sample_state_cost(robot_state, human_sigma_points, human_sigma_weights, goal, obstaclesX):       
        human_dist_sigma_points = jnp.linalg.norm(robot_state[0:2] - human_sigma_points, axis=0).reshape(1,-1)
        mu_human_dist, cov_human_dist = get_mean_cov( human_dist_sigma_points, human_sigma_weights )
        robot_obstacle_dists = jnp.linalg.norm(robot_state[0:2] - obstaclesX, axis=0)
        cost = MPPI_FORESEE.cost_safety_coeff / jnp.max(  jnp.array([mu_human_dist[0,0] - MPPI_FORESEE.std_factor * jnp.sqrt(cov_human_dist[0,0]), 0.01 ]) )
        cost = cost + MPPI_FORESEE.cost_safety_coeff / jnp.max(jnp.array([jnp.min(robot_obstacle_dists), 0.01])  )
        return cost
    
    @staticmethod
    @jit
    def rollout_control(init_state, actions):
        states = jnp.zeros((MPPI_FORESEE.robot_n, MPPI_FORESEE.horizon+1))
        states = states.at[:,0].set(init_state[:,0])
        def body(i, inputs):
            states = inputs
            states = states.at[:,i+1].set( MPPI_FORESEE.robot_dynamics_step(states[:,[i]], actions[i,:].reshape(-1,1))[:,0] )
            return states
        states = lax.fori_loop(0, MPPI_FORESEE.horizon, body, states)
        return states
        
    @staticmethod
    @jit
    def weighted_sum(U, perturbation, costs):#weights):
        costs = costs - jnp.min(costs)
        costs = costs / jnp.max(costs)
        lambd = MPPI_FORESEE.costs_lambda #300
        weights = jnp.exp( - 1.0/lambd * costs )   
        normalization_factor = jnp.sum(weights)
        def body(i, inputs):
            U = inputs
            U = U + perturbation[i] * weights[i] / normalization_factor
            return U
        return lax.fori_loop( 0, MPPI_FORESEE.samples, body, (U) )
    
    @staticmethod
    @jit
    def true_func(u_human, mu_x, robot_x):
        u_human = u_human - (robot_x[0:2] - mu_x) / jnp.clip(jnp.linalg.norm( mu_x-robot_x[0:2] ), 0.01, None) * ( MPPI_FORESEE.human_repulsion_gain * jnp.tanh( 1.0 / jnp.linalg.norm( mu_x-robot_x[0:2] ) ) )
        return u_human

    @staticmethod
    @jit
    def false_func(u_human, mu_x, robot_x):
        return u_human
    
    @staticmethod
    @jit
    def true_func_obstacle(u_human, obstacle_x, robot_x):
        u_human = u_human - (robot_x[0:2] - obstacle_x) / jnp.clip(jnp.linalg.norm( obstacle_x-robot_x[0:2] ), 0.01, None) * ( MPPI_FORESEE.human_repulsion_gain * jnp.tanh( 1.0 / jnp.clip( jnp.linalg.norm( obstacle_x-robot_x[0:2] )-MPPI_FORESEE.obstacle_radius, 0.01, None ) ) )
        return u_human

    @staticmethod
    @jit
    def multi_human_dynamics(human_x, other_human_x, robot_x, human_speed, obstaclesX, aware, mu_state):

        # u = MPPI_FORESEE.human_nominal_speed
        u = human_speed

        # Human repulsive force -> based only on mean of other robot
        # maybe show that in a cooperative scenario, this works??
        if MPPI_FORESEE.humans_interact:
            dist_humans = jnp.linalg.norm( human_x - other_human_x, axis=0 )
            def body(i, inputs):
                u = inputs
                u = lax.cond( dist_humans[i]<MPPI_FORESEE.sensing_radius, MPPI_FORESEE.true_func, MPPI_FORESEE.false_func, u, human_x, other_human_x[:,[i]])
                return u
            u = lax.fori_loop(0, MPPI_FORESEE.num_humans-1, body, u)

        # robot repulsive force
        u_human = lax.cond( jnp.logical_and((jnp.linalg.norm( human_x-robot_x[0:2] )<MPPI_FORESEE.sensing_radius), (aware[0])), MPPI_FORESEE.true_func, MPPI_FORESEE.false_func, u, human_x, robot_x)

        # Obstacle repulsive force based on nearest obstacle
        if MPPI_FORESEE.obstacles_interact:
            dist_obstacles = jnp.linalg.norm( human_x - obstaclesX, axis=0 )
            min_dist_obs_id = jnp.min(jnp.argmin(dist_obstacles))
            u_human = lax.cond( dist_obstacles[min_dist_obs_id]<MPPI_FORESEE.sensing_radius, MPPI_FORESEE.true_func_obstacle, MPPI_FORESEE.false_func, u_human, human_x, obstaclesX[:,[min_dist_obs_id]])

        # Clip the control input
        # u_human = jnp.clip(u_human, -4.0, 4.0)
        u_human = jnp.clip(u_human, -1.5, 1.5)

        # Propagate dynamics for human
        mu, cov = human_x + u_human * MPPI_FORESEE.dt, MPPI_FORESEE.human_noise_cov * jnp.eye(MPPI_FORESEE.human_n) * MPPI_FORESEE.dt**2

        return mu, cov
    
    @staticmethod
    @jit
    def multi_human_dynamics_mean(human_x, other_human_x, robot_x, human_speed, obstaclesX, aware, mu_state):

        # u = MPPI_FORESEE.human_nominal_speed
        u = human_speed

        # Human repulsive force -> based only on mean of other robot
        # maybe show that in a cooperative scenario, this works??
        if MPPI_FORESEE.humans_interact:
            dist_humans = jnp.linalg.norm( mu_state - other_human_x, axis=0 )
            def body(i, inputs):
                u = inputs
                u = lax.cond( dist_humans[i]<MPPI_FORESEE.sensing_radius, MPPI_FORESEE.true_func, MPPI_FORESEE.false_func, u, mu_state, other_human_x[:,[i]])
                return u
            u = lax.fori_loop(0, MPPI_FORESEE.num_humans-1, body, u)

        # robot repulsive force
        u_human = lax.cond( jnp.logical_and((jnp.linalg.norm( mu_state-robot_x[0:2] )<MPPI_FORESEE.sensing_radius), (aware[0])), MPPI_FORESEE.true_func, MPPI_FORESEE.false_func, u, mu_state, robot_x)

        # Obstacle repulsive force based on nearest obstacle
        if MPPI_FORESEE.obstacles_interact:
            dist_obstacles = jnp.linalg.norm( mu_state - obstaclesX, axis=0 )
            min_dist_obs_id = jnp.min(jnp.argmin(dist_obstacles))
            u_human = lax.cond( dist_obstacles[min_dist_obs_id]<MPPI_FORESEE.sensing_radius, MPPI_FORESEE.true_func_obstacle, MPPI_FORESEE.false_func, u_human, mu_state, obstaclesX[:,[min_dist_obs_id]])

        # Clip the control input
        u_human = jnp.clip(u_human, -4.0, 4.0)

        # Propagate dynamics for human
        mu, cov = human_x + u_human * MPPI_FORESEE.dt, MPPI_FORESEE.human_noise_cov * jnp.eye(MPPI_FORESEE.human_n) * MPPI_FORESEE.dt**2

        return mu, cov
    
    @staticmethod
    @jit
    def multi_human_sigma_point_expand(sigma_points, weights, other_human_mus, robot_state, human_speed, obstaclesX, aware):

        mu_state, cov_state = get_mean(sigma_points, weights)
        new_points = jnp.zeros((MPPI_FORESEE.human_n*(2*MPPI_FORESEE.human_n+1), 2*MPPI_FORESEE.human_n+1 ))
        new_weights = jnp.zeros((2*MPPI_FORESEE.human_n+1, 2*MPPI_FORESEE.human_n+1))

        # loop over sigma points
        def body(i, inputs):
            new_points, new_weights = inputs
            mu, cov = lax.cond( aware[1], MPPI_FORESEE.multi_human_dynamics, MPPI_FORESEE.multi_human_dynamics_mean, sigma_points[:,[i]], other_human_mus, robot_state, human_speed, obstaclesX, aware, mu_state )
            root_term = get_ut_cov_root_diagonal(cov)           
            temp_points, temp_weights = generate_sigma_points_gaussian( mu, root_term, jnp.zeros((MPPI_FORESEE.human_n, 1)), 1.0 )
            new_points = new_points.at[:,i].set( temp_points.reshape(-1,1, order='F')[:,0] )
            new_weights = new_weights.at[:,i].set( temp_weights.reshape(-1,1, order='F')[:,0] * weights[:,i] )   
            return new_points, new_weights
        new_points, new_weights = lax.fori_loop( 0, 2*MPPI_FORESEE.human_n+1, body, (new_points, new_weights) )
        return new_points.reshape( (MPPI_FORESEE.human_n, (2*MPPI_FORESEE.human_n+1)**2), order='F' ), new_weights.reshape(( 1,(2*MPPI_FORESEE.human_n+1)**2 ), order='F')

    @staticmethod
    @jit
    def single_sample_rollout(goal, robot_states_init, perturbed_control, human_sigma_points_init, human_sigma_weights_init, human_mus_init, human_covs_init, human_speed, obstaclesX, aware):
        
        # Initialize variables
        robot_states = jnp.zeros( ( MPPI_FORESEE.robot_n, MPPI_FORESEE.horizon) )
        human_sigma_points = jnp.zeros( ((2*MPPI_FORESEE.human_n+1)*MPPI_FORESEE.human_n, MPPI_FORESEE.num_humans, MPPI_FORESEE.horizon) )
        human_sigma_weights = jnp.zeros( (2*MPPI_FORESEE.human_n+1, MPPI_FORESEE.num_humans, MPPI_FORESEE.horizon) )
        human_mus = jnp.zeros( (MPPI_FORESEE.human_n, MPPI_FORESEE.num_humans, MPPI_FORESEE.horizon) )
        human_covs = jnp.zeros( (MPPI_FORESEE.human_n, MPPI_FORESEE.num_humans, MPPI_FORESEE.horizon) )

        # Set inital value
        robot_states = robot_states.at[:,0].set(robot_states_init)
        human_sigma_points = human_sigma_points.at[:,:,0].set(human_sigma_points_init)
        human_sigma_weights  = human_sigma_weights.at[:,:,0].set(human_sigma_weights_init)
        human_mus = human_mus.at[:,:,0].set(human_mus_init)
        human_covs = human_covs.at[:,:,0].set(human_covs_init)

        # loop over humans? but symmetric so there should be a better way to do this
        @jit
        def loop_over_humans(j, inputs_humans):
            i, cost_sample, robot_state, human_sigma_state, human_sigma_weight, human_sigma_points, human_sigma_weights, human_mus, human_covs, human_speed, obstaclesX = inputs_humans

            # get cost for collision avoidance with humans
            cost_sample = cost_sample + MPPI_FORESEE.single_sample_state_cost( robot_state, human_sigma_state[:,:,j], human_sigma_weight[:,[j]].T,  goal, obstaclesX) / MPPI_FORESEE.num_humans

            # Expand this human's state
            expanded_states, expanded_weights = MPPI_FORESEE.multi_human_sigma_point_expand( human_sigma_state[:,:,j], human_sigma_weight[:,[j]].T, human_mus[:, MPPI_FORESEE.hindex_list[j,:], i], robot_state, human_speed[:,[j]], obstaclesX, aware )
            mu_temp, cov_temp, compressed_states, compressed_weights = sigma_point_compress( expanded_states, expanded_weights )
            human_sigma_points = human_sigma_points.at[:, j, i+1].set( compressed_states.T.reshape(-1,1)[:,0] )
            human_sigma_weights = human_sigma_weights.at[:, j, i+1].set( compressed_weights.T[:,0] )
            human_mus = human_mus.at[:, j, i+1].set( mu_temp[:,0] )   
            human_covs = human_covs.at[:, j, i+1].set( jnp.diag(cov_temp) )
            return i, cost_sample, robot_state, human_sigma_state, human_sigma_weight, human_sigma_points, human_sigma_weights, human_mus, human_covs, human_speed, obstaclesX

        # loop over horizon
        cost_sample = 0
        def body(i, inputs):
            cost_sample, robot_states, human_sigma_points, human_sigma_weights, human_mus, human_covs, human_speed, obstaclesX = inputs
            human_sigma_state = human_sigma_points[:,:,i].reshape((MPPI_FORESEE.human_n, 2*MPPI_FORESEE.human_n+1, MPPI_FORESEE.num_humans), order='F')
            human_sigma_weight = human_sigma_weights[:,:,i]
            robot_state = robot_states[:,[i]]

            # Get goal cost
            cost_sample = cost_sample + MPPI_FORESEE.cost_goal_coeff * ((robot_state[0:2]-goal).T @ (robot_state[0:2]-goal))[0,0]

            # Update robot states
            robot_states = robot_states.at[:,i+1].set( MPPI_FORESEE.robot_dynamics_step( robot_states[:,[i]], perturbed_control[:, [i]] )[:,0] )
            
            # Update human states -> OR based on nearest human, obstacle only???? atleast in gazbeo its based on nearest human/object only so should be fine
            _, cost_sample, robot_state, human_sigma_state, human_sigma_weight, human_sigma_points, human_sigma_weights, human_mus, human_covs, _, _ = lax.fori_loop(0, MPPI_FORESEE.num_humans, loop_over_humans, (i, cost_sample, robot_state, human_sigma_state, human_sigma_weight, human_sigma_points, human_sigma_weights, human_mus, human_covs, human_speed, obstaclesX))            

            return cost_sample, robot_states, human_sigma_points, human_sigma_weights, human_mus, human_covs, human_speed, obstaclesX
        cost_sample, robot_states, human_sigma_points, human_sigma_weights, human_mus, human_covs, _, _ = lax.fori_loop( 0, MPPI_FORESEE.horizon-1, body, (cost_sample, robot_states, human_sigma_points, human_sigma_weights, human_mus, human_covs, human_speed, obstaclesX) )
    
        # Update with cost for final state
        human_sigma_state = human_sigma_points[:,:,MPPI_FORESEE.horizon-1].reshape((MPPI_FORESEE.human_n, 2*MPPI_FORESEE.human_n+1, MPPI_FORESEE.num_humans), order='F')
        human_sigma_weight = human_sigma_weights[:,:, MPPI_FORESEE.horizon-1]
        robot_state = robot_states[:,[MPPI_FORESEE.horizon-1]]
        cost_sample = cost_sample + MPPI_FORESEE.cost_goal_coeff * ((robot_state[0:2]-goal).T @ (robot_state[0:2]-goal))[0,0] # goal cost

        @jit
        def update_final_cost(j, inputs_humans):
            cost_sample, robot_state, human_sigma_state, human_sigma_weight, obstaclesX = inputs_humans
            cost_sample = cost_sample + MPPI_FORESEE.single_sample_state_cost( robot_state, human_sigma_state[:,:,j], human_sigma_weight[:,[j]].T,  goal, obstaclesX) / MPPI_FORESEE.num_humans
            return cost_sample, robot_state, human_sigma_state, human_sigma_weight, obstaclesX
        cost_sample, _, _, _, _ = lax.fori_loop(0, MPPI_FORESEE.num_humans, update_final_cost, (cost_sample, robot_state, human_sigma_state, human_sigma_weight, obstaclesX) )

        return cost_sample, robot_states, human_sigma_points, human_sigma_weights, human_mus, human_covs
    
    @staticmethod
    @jit
    def human_point_init(i, inputs):
        points_init, weights_init, human_init_state_mu, human_init_state_cov = inputs
        points, weights = generate_sigma_points_gaussian( human_init_state_mu[:,[i]], jnp.diag(jnp.sqrt(human_init_state_cov[:,i])), jnp.zeros((MPPI_FORESEE.human_n,1)), 1.0 )
        points_init = points_init.at[:,i].set( points.T.reshape(-1,1)[:,0] )
        weights_init = weights_init.at[:,i].set( weights.T[:,0] )
        return points_init, weights_init, human_init_state_mu, human_init_state_cov

    @staticmethod
    @jit
    def rollout_states_foresee(subkey, robot_init_state, perturbed_control, previous_control, goal, human_init_state_mu, human_init_state_cov, human_speed, obstaclesX, aware):

        # Human
        human_sigma_points = jnp.zeros( (MPPI_FORESEE.samples,(2*MPPI_FORESEE.human_n+1)*MPPI_FORESEE.human_n, MPPI_FORESEE.num_humans, MPPI_FORESEE.horizon) )
        human_sigma_weights = jnp.zeros( (MPPI_FORESEE.samples,2*MPPI_FORESEE.human_n+1, MPPI_FORESEE.num_humans, MPPI_FORESEE.horizon) )
        human_mus = jnp.zeros( (MPPI_FORESEE.samples,MPPI_FORESEE.human_n, MPPI_FORESEE.num_humans, MPPI_FORESEE.horizon) )
        human_covs = jnp.zeros( (MPPI_FORESEE.samples,MPPI_FORESEE.human_n, MPPI_FORESEE.num_humans, MPPI_FORESEE.horizon) )
        
        ##### Initialize
        points_init = jnp.zeros(((2*MPPI_FORESEE.human_n+1)*MPPI_FORESEE.human_n, MPPI_FORESEE.num_humans))
        weights_init = jnp.zeros((2*MPPI_FORESEE.human_n+1, MPPI_FORESEE.num_humans))
        points_init, weights_init, _, _ = lax.fori_loop(0, MPPI_FORESEE.num_humans, MPPI_FORESEE.human_point_init, (points_init, weights_init, human_init_state_mu, human_init_state_cov))

        human_sigma_points = human_sigma_points.at[:,:,:,0].set( jnp.tile(points_init.reshape((1,-1,MPPI_FORESEE.num_humans)), (MPPI_FORESEE.samples,1,1)) )
        human_sigma_weights = human_sigma_weights.at[:,:,:,0].set( jnp.tile( weights_init.reshape((1,-1,MPPI_FORESEE.num_humans)), (MPPI_FORESEE.samples,1,1) ) )
        human_mus = human_mus.at[:,:,:,0].set( jnp.tile(human_init_state_mu.reshape((1,MPPI_FORESEE.human_n, MPPI_FORESEE.num_humans)), (MPPI_FORESEE.samples,1,1)) )
        human_covs = human_covs.at[:,:,:,0].set( jnp.tile(human_init_state_cov.reshape((1,MPPI_FORESEE.human_n, MPPI_FORESEE.num_humans)), (MPPI_FORESEE.samples,1,1)) )
        
        # Robot
        robot_states = jnp.zeros( (MPPI_FORESEE.samples, MPPI_FORESEE.robot_n, MPPI_FORESEE.horizon) )
        robot_states = robot_states.at[:,:,0].set( jnp.tile( robot_init_state.T, (MPPI_FORESEE.samples,1) ) )

        # Cost
        cost_total = jnp.zeros(MPPI_FORESEE.samples)

        if MPPI_FORESEE.use_gpu:
            @jit
            def body_sample(robot_states_init, perturbed_control_sample, human_sigma_points_init, human_sigma_weights_init, human_mus_init, human_covs_init): #, human_speed, obstaclesX):
                cost_sample, robot_states_sample, human_sigma_points_sample, human_sigma_weights_sample, human_mus_sample, human_covs_sample = MPPI_FORESEE.single_sample_rollout(goal, robot_states_init, perturbed_control_sample.T, human_sigma_points_init, human_sigma_weights_init, human_mus_init, human_covs_init, human_speed, obstaclesX, aware )
                return cost_sample, robot_states_sample, human_sigma_points_sample, human_sigma_weights_sample, human_mus_sample, human_covs_sample
            batched_body_sample = jax.vmap( body_sample, in_axes=0 )
            cost_total, robot_states, human_sigma_points, human_sigma_weights, human_mus, human_covs = batched_body_sample( robot_states[:,:,0], perturbed_control, human_sigma_points[:,:,:,0], human_sigma_weights[:,:,:,0], human_mus[:,:,:,0], human_covs[:,:,:,0])#, human_speed, obstaclesX  )
        else:
            @jit
            def body_samples(i, inputs):
                robot_states, human_sigma_points, human_sigma_weights, cost_total, human_mus, human_covs, human_speed, obstaclesX = inputs     

                # Get cost
                cost_sample, robot_states_sample, human_sigma_points_sample, human_sigma_weights_sample, human_mus_sample, human_covs_sample = MPPI_FORESEE.single_sample_rollout(goal, robot_states[i,:,0], perturbed_control[i,:,:].T, human_sigma_points[i,:,:,0], human_sigma_weights[i,:,:,0], human_mus[i,:,:,0], human_covs[i,:,:,0], human_speed, obstaclesX, aware )
                cost_total = cost_total.at[i].set( cost_sample )
                robot_states = robot_states.at[i,:,:].set( robot_states_sample )
                human_sigma_points= human_sigma_points.at[i,:,:].set( human_sigma_points_sample )
                human_sigma_weights= human_sigma_weights.at[i,:,:].set( human_sigma_weights_sample )
                human_mus = human_mus.at[i,:,:].set( human_mus_sample )
                human_covs = human_covs.at[i,:,:].set( human_covs_sample )
                return robot_states, human_sigma_points, human_sigma_weights, cost_total, human_mus, human_covs, human_speed, obstaclesX  
            robot_states, human_sigma_points, human_sigma_weights, cost_total, human_mus, human_covs, human_speed, obstaclesX = lax.fori_loop( 0, MPPI_FORESEE.samples, body_samples, (robot_states, human_sigma_points, human_sigma_weights, cost_total, human_mus, human_covs, human_speed, obstaclesX) )

        return robot_states, cost_total, human_sigma_points, human_sigma_weights, human_mus, human_covs#, perturbation
        
    @staticmethod
    @jit
    def compute_perturbed_control(subkey, control_mu, control_cov, control_bound, U):
        perturbation = multivariate_normal( subkey, control_mu, control_cov, shape=( MPPI_FORESEE.samples, MPPI_FORESEE.horizon ) ) # K x T x nu
        
        # perturbation = jnp.clip( perturbation, -0.8, 0.8 ) #0.3
        perturbation = jnp.clip( perturbation, -1.0, 1.0 ) #0.3
        perturbed_control = U + perturbation

        perturbed_control = jnp.clip( perturbed_control, -control_bound, control_bound )
        perturbation = perturbed_control - U
        return U, perturbation, perturbed_control
    
    def policy_mppi( self, init_state, goal, human_states_mu, human_states_cov, human_speed, obstaclesX, aware ):

        self.key, subkey = jax.random.split(self.key)
        self.U, perturbation, perturbed_control = MPPI_FORESEE.compute_perturbed_control(subkey, self.control_mu, self.control_cov, self.control_bound, self.U)

        t0 = time.time()
        sampled_robot_states, costs, human_sigma_points, human_sigma_weights, human_mus, human_covs = MPPI_FORESEE.rollout_states_foresee(subkey, init_state, perturbed_control, self.U, goal, human_states_mu, human_states_cov, human_speed, obstaclesX, aware)
        # sampled_robot_states, costs, human_sigma_points, human_sigma_weights, human_mus, human_covs, perturbation = MPPI_FORESEE.rollout_states_foresee(subkey, init_state, self.U, goal, human_states_mu, human_states_cov)
        tf = time.time()
        # print(f"costs: min: {jnp.min(costs)}, max: {jnp.max(costs)}")
        print(f"costs: min {jnp.min(costs)}")
        self.U = MPPI_FORESEE.weighted_sum( self.U, perturbation, costs) #weights )

        states_final = MPPI_FORESEE.rollout_control(init_state, self.U)              
        action = self.U[0,:].reshape(-1,1)
        self.U = jnp.append(self.U[1:,:], self.U[[-1],:], axis=0)

        sampled_robot_states = sampled_robot_states.reshape(( MPPI_FORESEE.robot_n*MPPI_FORESEE.samples, MPPI_FORESEE.horizon ))
        human_mus = human_mus.reshape( (MPPI_FORESEE.human_n*MPPI_FORESEE.samples, MPPI_FORESEE.num_humans, MPPI_FORESEE.horizon) )
        human_covs = human_covs.reshape( (MPPI_FORESEE.human_n*MPPI_FORESEE.samples, MPPI_FORESEE.num_humans, MPPI_FORESEE.horizon) )
   
        return np.asarray(sampled_robot_states), np.asarray(states_final), np.asarray(action), np.asarray(human_mus), np.asarray(human_covs)