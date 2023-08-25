import numpy as np
import jax.numpy as jnp
from jax import jit
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle, Polygon
import polytope as pt
# from utils import wrap_angle

def wrap_angle(theta):
        return np.arctan2( np.sin(theta), np.cos(theta) )

class dynamic_unicycle:
    
    def __init__(self, ax, pos = np.array([0,0,0]), dt = 0.01, color = 'k', alpha_nominal = 0.3, alpha_nominal_humans = 0.3, alpha_nominal_obstacles = 0.3, plot_label = [], plot_polytope=True):
        '''
        X0: iniytial state
        dt: simulation time step
        ax: plot axis handle
        id: robot id
        '''
        
        self.type = 'SingleIntegrator2D'        
        
        self.X0 = pos.reshape(-1,1)
        self.X = np.copy(self.X0)
        self.U = np.array([0,0]).reshape(-1,1)
        self.dt = dt
        self.ax = ax
        
        self.width = 0.3#0.4
        self.height = 0.3#0.4
        self.A, self.b = self.base_polytopic_location()
        
        # Plot handles
        self.body = ax.scatter([],[],c='g',alpha=1.0,s=70)
        self.plot_polytope = plot_polytope
        self.radii = 0.25
        self.axis = ax.plot([self.X[0,0],self.X[0,0]+self.radii*np.cos(self.X[2,0])],[self.X[1,0],self.X[1,0]+self.radii*np.sin(self.X[2,0])])
        if plot_polytope:
            points = np.array( [ [-self.width/2,-self.height/2], [self.width/2,-self.height/2], [self.width/2,self.height/2], [-self.width/2,self.height/2] ] )  
            self.patch = Polygon( points, linewidth = 1, edgecolor='k',facecolor=color, label=plot_label )      
            ax.add_patch(self.patch)
        self.render_plot()
        self.Xs = np.copy(self.X)
        self.Us = np.copy(self.U)

        # trust based alpha adaptation
        self.alpha_nominal = alpha_nominal
        self.alpha_nominal_humans = alpha_nominal_humans
        self.alpha_nominal_obstacles = alpha_nominal_obstacles
        
    def set_state(self, state):
        self.X = state.reshape(-1,1)
        # self.X[0,0] = state[0,0]
        # self.X[1,0] = state[1,0]
        # self.X[2,0] = state[2,0]
        
    def f(self):
        return np.array([self.X[3,0]*np.cos(self.X[2,0]),
                         self.X[3,0]*np.sin(self.X[2,0]),
                         0,0]).reshape(-1,1)
    
    def f_jax(self,X):
        return jnp.array([X[3,0]*jnp.cos(X[2,0]),
                          X[3,0]*jnp.sin(X[2,0]),
                         0,0]).reshape(-1,1)
        
    def df_dx(self):
        return np.array([  
                         [0, 0, -self.X[3,0]*np.sin(self.X[2,0]), np.cos(self.X[2,0])],
                         [0, 0,  self.X[3,0]*np.cos(self.X[2,0]), np.sin(self.X[2,0])],
                         [0, 0, 0, 0],
                         [0, 0, 0, 0]
                         ])
    
    def df_dx_jax(self, X):
        return jnp.array([  
                         [0, 0, -X[3,0]*jnp.sin(X[2,0]), jnp.cos(X[2,0])],
                         [0, 0,  X[3,0]*jnp.cos(X[2,0]), jnp.sin(X[2,0])],
                         [0, 0, 0, 0],
                         [0, 0, 0, 0]
                         ])

    def f_casadi(self,X):
        return cd.vcat( [
            X[3,0] * cd.cos(X[2,0]),
            X[3,0] * cd.sin(X[2,0]), 
            0.0, 
            0.0
        ] ) 
    
    def g(self):
        return np.array([ [0, 0],[0, 0], [0, 1], [1, 0] ])

    def g_jax(self, X):
        return jnp.array([ [0, 0],[0, 0], [0, 1.0], [1.0, 0] ])
    
    # @jit
    def xdot_jax(self,X, U):
        return self.f_jax(X) + self.g_jax(X) @ U
    
    def f_xddot_casadi(self,X):
        return np.array([0,0,0,0]).reshape(-1,1)
        
    def g_xddot_casadi(self,X):
        return cd.vcat([
            cd.hcat([ cd.cos(X[2,0]), -X[3,0]*cd.sin(X[2,0]) ]),
            cd.hcat([ cd.sin(X[2,0]),  X[3,0]*cd.cos(X[2,0]) ]),
            cd.hcat([0, 0]),
            cd.hcat([0, 0])
        ])

    def g_casadi(self, X):
        return np.array([ [0, 0],[0, 0], [0, 1], [1, 0] ])
        
    def step(self,U, dt): #Just holonomic X,T acceleration

        self.U = U.reshape(-1,1)
        self.X = self.X + ( self.f() + self.g() @ self.U )*dt
        self.X[2,0] = wrap_angle(self.X[2,0])
        self.render_plot()
        self.Xs = np.append(self.Xs,self.X,axis=1)
        self.Us = np.append(self.Us,self.U,axis=1)
        return self.X
    
    def rot_mat(self,theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
            ])

    def render_plot(self):
        x = np.array([self.X[0,0],self.X[1,0]])
        theta = self.X[2,0]
        self.body.set_offsets([x[0],x[1]])
        self.axis[0].set_ydata([self.X[1,0],self.X[1,0]+self.radii*np.sin(self.X[2,0])])
        self.axis[0].set_xdata( [self.X[0,0],self.X[0,0]+self.radii*np.cos(self.X[2,0])] )
        if self.plot_polytope:
            points = np.array( [ [-self.width/2,-self.height/2], [self.width/2,-self.height/2], [self.width/2,self.height/2], [-self.width/2,self.height/2] ] )
            R = self.rot_mat(theta)
            points = (R @ points.T).T  + x     
            self.patch.set_xy( points )
            
    def base_polytopic_location(self):
        x = np.array([0,0])
        points = np.array( [ [x[0]-self.width/2,x[1]-self.height/2], [x[0]+self.width/2,x[1]-self.height/2], [x[0]+self.width/2,x[1]+self.height/2], [x[0]-self.width/2,x[1]+self.height/2] ] )
        hull = pt.qhull(points)
        return hull.A, hull.b.reshape(-1,1)
    
    def polytopic_location(self):
        theta = self.X[2,0]
        Rot = self.rot_mat(theta)
        # return self.A @ Rot.T, self.A @ Rot.T @ self.X[0:2].reshape(-1,1)+self.b
        return self.A @ Rot, self.A @ Rot @ self.X[0:2].reshape(-1,1)+self.b
    
    def polytopic_location_next_state(self):
        Rot = np.array([
            [1.0, 0.0],
            [0.0, 1.0]
            ])
        Rot_dot = np.array([
            [0.0, 0.0],
            [0.0, 0.0]
            ])
    
        A, b = self.polytopic_location()
        
        b_f = np.copy(b)
        b_g = A @ Rot.T # to be multiplied with control input
        
        return A, b_f, b_g*self.dt

    def nominal_controller(self, targetX, k_omega = 1.0, k_v = 1.0, k_x = 1.0):
        # k_omega = 3.0#2.0 
        # k_v = 1.0#3.0#0.3#0.15##5.0#0.15
        # k_x = k_v
        distance = np.linalg.norm( self.X[0:2]-targetX[0:2] )
        # print(f"distance: {distance}")
        if (distance>0.1):
            desired_heading = np.arctan2( targetX[1,0]-self.X[1,0], targetX[0,0]-self.X[0,0] )
            error_heading = wrap_angle( desired_heading - self.X[2,0] )
            speed = min(k_x * distance * np.cos(error_heading), 0.7)
        else:
            error_heading = 0
            speed = 0

        omega = k_omega * error_heading #* np.tanh( distance )
        
        # print(f"CBF nominal speed: {speed}, omega:{omega}")
        u_r = k_v * ( speed - self.X[3,0] )
        # print(f"CBF nominal acc: {u_r}, omega:{omega}")
        return np.array([u_r, omega]).reshape(-1,1)
    
    # @jit
    def nominal_controller_jax(self, X, targetX, k_omega = 3.0, k_v = 1.0, k_x = 1.0):
        # k_omega = 3.0#2.0 
        # k_v = 1.0#3.0#0.3#0.15##5.0#0.15
        # k_x = k_v
        distance = jnp.max( jnp.array([jnp.linalg.norm( X[0:2]-targetX[0:2] ), 0.1]) )
        desired_heading = jnp.arctan2( targetX[1,0]-X[1,0], targetX[0,0]-X[0,0] )
        error_heading = desired_heading - X[2,0]
        error_heading = jnp.arctan2( jnp.sin(error_heading), jnp.cos(error_heading) )

        omega = k_omega * error_heading * jnp.tanh( distance )
        speed = k_x * distance * jnp.cos(error_heading)
        u_r = 1.0 * k_v * ( speed - X[3,0] )
        return jnp.array([u_r, omega]).reshape(-1,1)
    
    def barrier(self, target, d_min = 0.5, alpha1 = 1.0):
 
        h = (self.X[0:2] - target.X[0:2]).T @ (self.X[0:2] - target.X[0:2]) - d_min**2
        # assert(h >= 0.0)
        # print(f"h :{h}")
        dh_dx1 = np.append( 2*(self.X[0:2] - target.X[0:2]).T, np.array([[0, 0]]), axis = 1 )
        dh_dx2 = - 2*(self.X[0:2] - target.X[0:2]).T
        
        h_dot = 2 * (self.X[0:2] - target.X[0:2]).T @ ( self.f() )[0:2]
        df_dx = self.df_dx()
        dh_dot_dx1 = np.append( ( self.f() )[0:2].T, np.array([[0,0]]), axis = 1 ) + 2 * ( self.X[0:2] - target.X[0:2] ).T @ df_dx[0:2,:]
        dh_dot_dx2 = - 2 * self.f()[0:2].T
      
        h1 = h_dot + alpha1 * h 
        dh1_dx1 = dh_dot_dx1 + alpha1 * dh_dx1
        dh1_dx2 = dh_dot_dx2 + alpha1 * dh_dx2
        
        return h1, dh1_dx1, dh1_dx2
    
    def barrier_jax(self, X, targetX, d_min = 0.5, alpha1 = 1.0):
 
        h = (X[0:2] - targetX[0:2]).T @ (X[0:2] - targetX[0:2]) - d_min**2
        # assert(h >= 0.0)
        # print(f"h :{h}")
        dh_dx1 = jnp.append( 2*(X[0:2] - targetX[0:2]).T, jnp.array([[0, 0]]), axis = 1 )
        dh_dx2 = - 2*(X[0:2] - targetX[0:2]).T
        
        h_dot = 2 * (X[0:2] - targetX[0:2]).T @ ( self.f_jax(X) )[0:2]
        df_dx = self.df_dx_jax(X)
        dh_dot_dx1 = jnp.append( ( self.f_jax(X) )[0:2].T, jnp.array([[0,0]]), axis = 1 ) + 2 * ( self.X[0:2] - targetX[0:2] ).T @ df_dx[0:2,:]
        dh_dot_dx2 = - 2 * self.f_jax(X)[0:2].T
      
        h1 = h_dot + alpha1 * h 
        dh1_dx1 = dh_dot_dx1 + alpha1 * dh_dx1
        dh1_dx2 = dh_dot_dx2 + alpha1 * dh_dx2
        
        return h1, dh1_dx1, dh1_dx2
    
    def barrier_alpha_jax(self, X, targetX, d_min = 0.5):
        h = (X[0:2] - targetX[0:2]).T @ (X[0:2] - targetX[0:2]) - d_min**2
        h_dot = 2 * (X[0:2] - targetX[0:2]).T @ ( self.f_jax(X)[0:2]  )
        df_dx = self.df_dx_jax(X)
        dh_dot_dx1 = jnp.append( ( self.f_jax(X)[0:2] ).T, jnp.array([[0,0]]), axis = 1 ) + 2 * ( X[0:2] - targetX[0:2] ).T @ df_dx[0:2,:]
        dh_dot_dx2 = - 2 * ( self.f_jax(X)[0:2].T )
        return dh_dot_dx1, dh_dot_dx2, h_dot, h
    
    def barrier_humans_jax(self, X, targetX, targetU, d_min = 0.5, alpha1 = 1.0):
 
        h = (X[0:2] - targetX[0:2]).T @ (X[0:2] - targetX[0:2]) - d_min**2
        # assert(h >= -0.05)
        # print(f"h :{h}")
        dh_dx1 = jnp.append( 2*(X[0:2] - targetX[0:2]).T, jnp.array([[0, 0]]), axis = 1 )
        dh_dx2 = - 2*(X[0:2] - targetX[0:2]).T
        
        h_dot = 2 * (X[0:2] - targetX[0:2]).T @ ( self.f_jax(X)[0:2] - targetU[0:2] )
        df_dx = self.df_dx_jax(X)
        dh_dot_dx1 = jnp.append( ( self.f_jax(X)[0:2] - targetU[0:2] ).T, jnp.array([[0,0]]), axis = 1 ) + 2 * ( X[0:2] - targetX[0:2] ).T @ df_dx[0:2,:]
        dh_dot_dx2 = - 2 * ( self.f()[0:2].T -targetU[0:2] )
        
        h1 = h_dot + alpha1 * h 
        dh1_dx1 = dh_dot_dx1 + alpha1 * dh_dx1
        dh1_dx2 = dh_dot_dx2 + alpha1 * dh_dx2
        
        return h1, dh1_dx1, dh1_dx2
    
    def barrier_humans(self, targetX, targetU, d_min = 0.5, alpha1 = 1.0):
 
        h = (self.X[0:2] - targetX[0:2]).T @ (self.X[0:2] - targetX[0:2]) - d_min**2
        assert(h >= -0.05)
        # print(f"h :{h}")
        dh_dx1 = np.append( 2*(self.X[0:2] - targetX[0:2]).T, np.array([[0, 0]]), axis = 1 )
        dh_dx2 = - 2*(self.X[0:2] - targetX[0:2]).T
        
        h_dot = 2 * (self.X[0:2] - targetX[0:2]).T @ ( self.f()[0:2] - targetU[0:2] )
        df_dx = self.df_dx()
        dh_dot_dx1 = np.append( ( self.f()[0:2] - targetU[0:2] ).T, np.array([[0,0]]), axis = 1 ) + 2 * ( self.X[0:2] - targetX[0:2] ).T @ df_dx[0:2,:]
        dh_dot_dx2 = - 2 * ( self.f()[0:2].T -targetU[0:2].T )
      
        h1 = h_dot + alpha1 * h 
        dh1_dx1 = dh_dot_dx1 + alpha1 * dh_dx1
        dh1_dx2 = dh_dot_dx2 + alpha1 * dh_dx2
        
        return h1, dh1_dx1, dh1_dx2
    
    def barrier_humans_alpha_jax(self, X, targetX, targetU, d_min = 0.5):
        h = (X[0:2] - targetX[0:2]).T @ (X[0:2] - targetX[0:2]) - d_min**2
        h_dot = 2 * (X[0:2] - targetX[0:2]).T @ ( self.f_jax(X)[0:2] - targetU[0:2] )
        df_dx = self.df_dx_jax(X)
        dh_dot_dx1 = jnp.append( ( self.f_jax(X)[0:2] - targetU[0:2] ).T, jnp.array([[0,0]]), axis = 1 ) + 2 * ( X[0:2] - targetX[0:2] ).T @ df_dx[0:2,:]
        dh_dot_dx2 = - 2 * ( self.f_jax(X)[0:2].T -targetU[0:2].T )
        return dh_dot_dx1, dh_dot_dx2, h_dot, h
    
    def barrier_humans_alpha(self, targetX, targetU, d_min = 0.5):
 
        h = (self.X[0:2] - targetX[0:2]).T @ (self.X[0:2] - targetX[0:2]) - d_min**2
        assert(h >= -0.05)
        # print(f"h :{h}")
        dh_dx1 = np.append( 2*(self.X[0:2] - targetX[0:2]).T, np.array([[0, 0]]), axis = 1 )
        dh_dx2 = - 2*(self.X[0:2] - targetX[0:2]).T
        
        h_dot = 2 * (self.X[0:2] - targetX[0:2]).T @ ( self.f()[0:2] - targetU[0:2] )
        df_dx = self.df_dx()
        dh_dot_dx1 = np.append( ( self.f()[0:2] - targetU[0:2] ).T, np.array([[0,0]]), axis = 1 ) + 2 * ( self.X[0:2] - targetX[0:2] ).T @ df_dx[0:2,:]
        dh_dot_dx2 = - 2 * ( self.f()[0:2].T -targetU[0:2].T )
        
        return dh_dot_dx1, dh_dot_dx2, h_dot, h
        

        
        
        
    