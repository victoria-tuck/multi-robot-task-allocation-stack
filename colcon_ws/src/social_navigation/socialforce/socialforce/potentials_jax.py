"""Interaction potentials."""

# import numpy as np
import jax.numpy as jnp

from . import stateutils_jax


class PedPedPotential(object):
    """Ped-ped interaction potential.

    v0 is in m^2 / s^2.
    sigma is in m.
    """    
    v0 = 2.1
    sigma = 0.3

    def __init__(self, v0=2.1, sigma=0.3):
        PedPedPotential.v0 = v0
        PedPedPotential.sigma = sigma

    @staticmethod
    def b(r_ab, speeds, desired_directions, delta_t):
        """Calculate b."""
        speeds_b = jnp.expand_dims(speeds, axis=0)
        speeds_b_abc = jnp.expand_dims(speeds_b, axis=2)  # abc = alpha, beta, coordinates
        e_b = jnp.expand_dims(desired_directions, axis=0)

        in_sqrt = (
            jnp.linalg.norm(r_ab, axis=-1) +
            jnp.linalg.norm(r_ab - delta_t * speeds_b_abc * e_b, axis=-1)
        )**2 - (delta_t * speeds_b)**2
        in_sqrt = jnp.fill_diagonal(in_sqrt, 0.0, inplace=False)

        return 0.5 * jnp.sqrt(in_sqrt)

    @staticmethod
    def value_r_ab(r_ab, speeds, desired_directions, delta_t):
        """Value of potential explicitely parametrized with r_ab."""
        return PedPedPotential.v0 * jnp.exp(-PedPedPotential.b(r_ab, speeds, desired_directions, delta_t) / PedPedPotential.sigma)

    @staticmethod
    def r_ab(state):
        """r_ab"""
        r = state[:, 0:2]
        r_a = jnp.expand_dims(r, 1)
        r_b = jnp.expand_dims(r, 0)
        return r_a - r_b

    # @staticmethod
    # def __call__(state, delta_t):
    #     speeds = stateutils.speeds(state)
    #     return PedPedPotential.value_r_ab(PedPedPotential.r_ab(state), speeds, stateutils.desired_directions(state), delta_t)

    @staticmethod
    def grad_r_ab(state, delta_t, delta=1e-3):
        """Compute gradient wrt r_ab using finite difference differentiation."""
        r_ab = PedPedPotential.r_ab(state)
        speeds = stateutils_jax.speeds(state)
        desired_directions = stateutils_jax.desired_directions(state)

        dx = jnp.array([[[delta, 0.0]]])
        dy = jnp.array([[[0.0, delta]]])

        v = PedPedPotential.value_r_ab(r_ab, speeds, desired_directions, delta_t)
        dvdx = (PedPedPotential.value_r_ab(r_ab + dx, speeds, desired_directions, delta_t) - v) / delta
        dvdy = (PedPedPotential.value_r_ab(r_ab + dy, speeds, desired_directions, delta_t) - v) / delta

        # remove gradients from self-intereactions
        dvdx = jnp.fill_diagonal(dvdx, 0.0, inplace=False)
        dvdy = jnp.fill_diagonal(dvdy, 0.0, inplace=False)

        return jnp.stack((dvdx, dvdy), axis=-1)


class PedSpacePotential(object):
    """Pedestrian-space interaction potential.

    space is a list of numpy arrays containing points of boundaries.

    u0 is in m^2 / s^2.
    r is in m
    """
    u0 = 10
    r = 0.2

    def __init__(self, u0=10, r=0.2):
        # self.space = space or []
        PedSpacePotential.u0 = u0
        PedSpacePotential.r = r

    @staticmethod
    def value_r_aB(r_aB):
        """Compute value parametrized with r_aB."""
        return PedSpacePotential.u0 * jnp.exp(-1.0 * jnp.linalg.norm(r_aB, axis=-1) / PedSpacePotential.r)

    @staticmethod
    def r_aB(state, space):
        """r_aB"""
        # if not self.space:
        #     return jnp.zeros((state.shape[0], 0, 2))

        r_a = jnp.expand_dims(state[:, 0:2], 1)
        closest_i = [
            jnp.argmin(jnp.linalg.norm(r_a - jnp.expand_dims(B, 0), axis=-1), axis=1)
            for B in space
        ]
        closest_points = jnp.swapaxes(
            jnp.stack([B[i] for B, i in zip(space, closest_i)]),
            0, 1)  # index order: pedestrian, boundary, coordinates
        return r_a - closest_points

    # @staticmethod
    # def __call__(state):
    #     return PedSpacePotential.value_r_aB(PedSpacePotential.r_aB(state))

    @staticmethod
    def grad_r_aB(state, delta=1e-3):
        """Compute gradient wrt r_aB using finite difference differentiation."""
        r_aB = PedSpacePotential.r_aB(state)

        dx = jnp.array([[[delta, 0.0]]])
        dy = jnp.array([[[0.0, delta]]])

        v = PedSpacePotential.value_r_aB(r_aB)
        dvdx = (PedSpacePotential.value_r_aB(r_aB + dx) - v) / delta
        dvdy = (PedSpacePotential.value_r_aB(r_aB + dy) - v) / delta

        return jnp.stack((dvdx, dvdy), axis=-1)
