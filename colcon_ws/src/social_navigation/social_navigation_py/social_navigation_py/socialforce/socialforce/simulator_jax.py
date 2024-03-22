# coding=utf-8

"""Synthetic pedestrian behavior according to the Social Force model.

See Helbing and Molnár 1998.
"""

# import numpy as np
import jax.numpy as jnp
from jax import jit
import matplotlib.pyplot as plt
# from .socialforce import socialforce
from .potentials_jax import PedPedPotential
from .fieldofview_jax import FieldOfView
from . import stateutils_jax

MAX_SPEED_MULTIPLIER = 1.3  # with respect to initial speed

class Simulator(object):
    """Simulate social force model.

    Main interface is the state. Every pedestrian is an entry in the state and
    represented by a vector (x, y, v_x, v_y, d_x, d_y, [tau]).
    tau is optional in this vector.

    ped_space is an instance of PedSpacePotential.

    delta_t in seconds.
    tau in seconds: either float or numpy array of shape[n_ped].
    """

    V = 0 # PedPed potential
    U = 0 # Ped object potential
    w = 0 # 

    def __init__(self, ped_space=None, delta_t=0.4, tau=0.5, initial_speed=1.0, v0=0.3, sigma=0.3):
        # Simulator.state = initial_state
        # Simulator.initial_speeds = jnp.ones((initial_state.shape[0])) * initial_speed
        # Simulator.max_speeds = MAX_SPEED_MULTIPLIER * Simulator.initial_speeds

        # Simulator.delta_t = delta_t

        # if Simulator.state.shape[1] < 7:
        #     if not hasattr(tau, 'shape'):
        #         tau = tau * jnp.ones(Simulator.state.shape[0])
        #     Simulator.state = jnp.concatenate((Simulator.state, np.expand_dims(tau, -1)), axis=-1)

        # potentials
        Simulator.V = PedPedPotential(v0=v0, sigma=sigma)
        Simulator.U = ped_space
        # field of view
        Simulator.w = FieldOfView(twophi=360.0)

    @staticmethod
    def f_ab(state, delta_t):
        """Compute f_ab."""
        return -1.0 * Simulator.V.grad_r_ab(state, delta_t)

    @staticmethod
    def f_aB(state, delta_t):
        return jnp.zeros((state.shape[0], 0, 2))
        """Compute f_aB."""
        # if Simulator.U is None:
        #     return jnp.zeros((Simulator.state.shape[0], 0, 2))
        return -1.0 * Simulator.U.grad_r_aB(state)

    @staticmethod
    def capped_velocity(desired_velocity, max_speeds):
        """Scale down a desired velocity to its capped speed."""
        desired_speeds = jnp.linalg.norm(desired_velocity, axis=-1)
        factor = jnp.minimum(1.0, max_speeds / desired_speeds)
        return desired_velocity * jnp.expand_dims(factor, -1)
    
    @staticmethod
    @jit
    def step_(state, initial_speeds, max_speeds, delta_t):
        """Do one step in the simulation and update the state in place."""
        # accelerate to desired velocity
        e = stateutils_jax.desired_directions(state)
        vel = state[:, 2:4]
        tau = state[:, 6:7]
        F0 = 1.0 / tau * (jnp.expand_dims(initial_speeds, -1) * e - vel)

        # repulsive terms between pedestrians
        f_ab = Simulator.f_ab(state, delta_t)
        w = jnp.expand_dims(Simulator.w.__call__(e, -f_ab), -1)
        F_ab = w * f_ab * 20

        # repulsive terms between pedestrians and boundaries
        F_aB = Simulator.f_aB(state, delta_t)

        # social force
        F = F0 + jnp.sum(F_ab, axis=1) + jnp.sum(F_aB, axis=1)
        # desired velocity
        w = state[:, 2:4] + delta_t * F
        # velocity
        v = Simulator.capped_velocity(w, max_speeds)

        # update state
        # state[:, 0:2] += v * delta_t
        state = state.at[:,0:2].set( state[:,0:2] + v * delta_t )
        # state[:, 2:4] = v
        state = state.at[:, 2:4].set( v )

        return F, state

    def step(self,state, initial_speeds, max_speeds, delta_t):
        """Do one step in the simulation and update the state in place."""
        # accelerate to desired velocity
        F, state = Simulator.step_(state, initial_speeds, max_speeds, delta_t)
        return F, state
