import numpy as np
import jax
from jax import jit, grad, value_and_grad
import jax.numpy as jnp
from cvxpylayers.jax import CvxpyLayer
import polytope as pt
import cvxpy as cp

def compute_polytope_from_points(points):
    hull = pt.qhull(points)
    return hull, hull.A, hull.b

def polytope_union( polytope_1, polytope_2 ):
    nonconvex = polytope_1.union(polytope_2)  # construct union of convex polytopes
    return nonconvex

def polytope_intersect( polytope_1, polytope_2 ):
    nonconvex = polytope_1.intersect(polytope_2)  # construct union of convex polytopes
    return nonconvex

def plot_polytope_lines(ax, hull, u_bound):
    xs = np.linspace( -u_bound, u_bound, 3 )
    A, b = hull.A, hull.b
    alpha = 0.1
    for i in range(A.shape[0]):
        if np.abs(A[i,1])>0.001:
            ax.plot( xs, (b[i] - A[i,0]*xs)/A[i,1], color='k', linestyle='--', alpha = alpha )
        else:
            if np.abs(A[i,0])>0.001:
                ax.axvline( b[i]/A[i,0], color='k', linestyle='--', alpha = alpha )
            else:
                ax.vline( 0.0, color='k', linestyle='--', alpha = alpha )