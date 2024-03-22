"""Field of view computation."""

import jax.numpy as jnp
from jax import lax

class FieldOfView(object):
    """Compute field of view prefactors.

    The field of view angle twophi is given in degrees.
    out_of_view_factor is C in the paper.
    """
    cosphi = 0
    out_of_view_factor = 0
    def __init__(self, twophi=200.0, out_of_view_factor=0.5):
        FieldOfView.cosphi = jnp.cos(twophi / 2.0 / 180.0 * jnp.pi)
        FieldOfView.out_of_view_factor = out_of_view_factor

    @staticmethod
    def __call__(e, f):
        """Weighting factor for field of view.

        e is rank 2 and normalized in the last index.
        f is a rank 3 tensor.
        """
        in_sight = jnp.einsum('aj,abj->ab', e, f) > jnp.linalg.norm(f, axis=-1) * FieldOfView.cosphi
        out = FieldOfView.out_of_view_factor * jnp.ones_like(in_sight)
        # out = out.at[in_sight].set( 1.0 )

        def true_func(out, i, j):
            out = out.at[i,j].set(1.0)
            return out
        def false_func(out, i, j):
            return out
        def body_i(i, inputs_i):
            out, in_sight = inputs_i
            def body_j(j, inputs_j):
                i, out, in_sight = inputs_j
                out = lax.cond( in_sight[i,j], true_func, false_func, out, i, j )
                return i, out, in_sight
            _, out, in_sight = lax.fori_loop( 0, in_sight.shape[1], body_j, (i, out, in_sight) )
            return out, in_sight
        out, _ = lax.fori_loop( 0, in_sight.shape[0], body_i, (out, in_sight) )

        # out[in_sight] = 1.0
        out = jnp.fill_diagonal(out, 0.0, inplace=False)
        return out
