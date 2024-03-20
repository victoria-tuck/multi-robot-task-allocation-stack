"""Utility functions to process state."""

import jax.numpy as jnp
from jax import jit

@jit
def desired_directions(state):
    """Given the current state and destination, compute desired direction."""
    destination_vectors = state[:, 4:6] - state[:, 0:2]
    norm_factors = jnp.maximum(jnp.linalg.norm(destination_vectors, axis=-1),0.01*jnp.ones(destination_vectors.shape[0]))
    return destination_vectors / jnp.expand_dims(norm_factors, -1)


@jit
def speeds(state):
    """Return the speeds corresponding to a given state."""
    return jnp.linalg.norm(state[:, 2:4], axis=-1)
