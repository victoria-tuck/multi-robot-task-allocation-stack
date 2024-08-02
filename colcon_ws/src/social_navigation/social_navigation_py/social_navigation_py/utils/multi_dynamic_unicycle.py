import numpy as np
import jax.numpy as jnp
from jax import jit
from jax import lax
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle, Polygon
import polytope as pt
# from utils import wrap_angle

def wrap_angle(theta):
        return np.arctan2( np.sin(theta), np.cos(theta) )

class multi_dynamic_unicycle:
    
    def __init__(self, ax, pos = np.array([0,0,0]), dt = 0.01, color = 'k', alpha_nominal = 0.3, alpha_nominal_humans = 0.3, alpha_nominal_obstacles = 0.3, plot_label = [], plot_polytope=True, num_agents=1):
        '''
        X0: initial state
        dt: simulation time step
        ax: plot axis handle
        id: robot id
        '''
        
        self.type = 'MultiAgentDynamicUnicycle' 
        self.num_agents = num_agents
        print(f"Number of agents: {self.num_agents}")    
        self.s = 0.287
        
        self.X0 = pos.reshape(-1,1)
        self.X = np.copy(self.X0)
        self.U = np.array([0,0] * self.num_agents).reshape(-1,1)
        self.dt = dt
        self.ax = ax
        
        self.width = 0.3#0.4
        self.height = 0.3#0.4
        # self.A, self.b = self.base_polytopic_location()
        
        # Plot handles
        # self.body = ax.scatter([],[],c='g',alpha=1.0,s=70)
        # self.plot_polytope = plot_polytope
        # self.radii = 0.25
        # self.axis = ax.plot([self.X[0,0],self.X[0,0]+self.radii*np.cos(self.X[2,0])],[self.X[1,0],self.X[1,0]+self.radii*np.sin(self.X[2,0])])
        # if plot_polytope:
        #     points = np.array( [ [-self.width/2,-self.height/2], [self.width/2,-self.height/2], [self.width/2,self.height/2], [-self.width/2,self.height/2] ] )  
        #     self.patch = Polygon( points, linewidth = 1, edgecolor='k',facecolor=color, label=plot_label )      
        #     ax.add_patch(self.patch)
        # self.render_plot()
        self.Xs = np.copy(self.X)
        self.Us = np.copy(self.U)

        # trust based alpha adaptation
        # self.alpha_nominal = alpha_nominal
        # self.alpha_nominal_humans = alpha_nominal_humans
        # self.alpha_nominal_obstacles = alpha_nominal_obstacles
        
    def set_state(self, state):
        self.X = state.reshape(-1,1)
        # self.X[0,0] = state[0,0]
        # self.X[1,0] = state[1,0]
        # self.X[2,0] = state[2,0]

    def f_i(self):
        return np.array([self.X[3,0]*np.cos(self.X[2,0]),
                         self.X[3,0]*np.sin(self.X[2,0]),
                         0,0]).reshape(-1,1)
    
    def f(self):
        num_agents = int(len(self.X)/4)
        return np.tile(self.f_i(), (num_agents, 1))
    
    def f_jax_i(self,X):
        return jnp.array([X[3,0]*jnp.cos(X[2,0]),
                          X[3,0]*jnp.sin(X[2,0]),
                         0,0]).reshape(-1,1)
    
    def f_jax(self,X):
        dynamics_list = jnp.zeros((self.num_agents, 4, 1))
        def body(i, carry):
            dynamics_list = carry
            sub_X = lax.dynamic_slice(X, (4*i, 0), (4, 1))
            result = self.f_jax_i(sub_X).reshape((1,4,1))
            dynamics_list = lax.dynamic_update_slice(dynamics_list, result, (i, 0, 0))
            return dynamics_list
        dynamics_list = lax.fori_loop(0, self.num_agents, body, (dynamics_list))
        return jnp.concatenate(dynamics_list, axis=0)
    
    def df_dx_jax_i(self, X):
        return jnp.array([  
                         [0, 0, -X[3,0]*jnp.sin(X[2,0]), jnp.cos(X[2,0])],
                         [0, 0,  X[3,0]*jnp.cos(X[2,0]), jnp.sin(X[2,0])],
                         [0, 0, 0, 0],
                         [0, 0, 0, 0]
                         ])
    
    def df_dx_jax(self, X):
        dynamics_list = jnp.zeros((self.num_agents, 4, 4*self.num_agents))
        def body(i, carry):
            dynamics_list = carry
            sub_X = lax.dynamic_slice(X, (4*i, 0), (4, 1))
            result = self.df_dx_jax_i(sub_X).reshape((1,4,4))
            dynamics_list = lax.dynamic_update_slice(dynamics_list, result, (i, 4*i, 0))
            return dynamics_list
        dynamics_list = lax.fori_loop(0, self.num_agents, body, (dynamics_list))
        return jnp.concatenate(dynamics_list, axis=0)

    def g_i(self):
        return np.array([ [0, 0],[0, 0], [0, 1], [1, 0] ])# @ self.vel_to_wheel_g()
    
    def g(self, X):
        num_agents = int(len(X)/4)
        print(f"Number of agents in g function: {num_agents}")
        array = np.zeros((4*num_agents, 2*num_agents))
        for i in range(num_agents):
            subarray = self.g_i()
            array[4*i:4*(i+1), 2*i:2*(i+1)] = subarray
        return array

    def g_jax_i(self, X):
        return jnp.array([ [0, 0],[0, 0], [0, 1.0], [1.0, 0] ])# @ self.vel_to_wheel_g_jax()
    
    def g_jax(self, X):
        return jnp.array([ [0, 0],[0, 0], [0, 1.0], [1.0, 0] ] * self.num_agents)# @ self.vel_to_wheel_g_jax()
    
    def xdot_jax_i(self,X, U):
        return self.f_jax_i(X) + self.g_jax_i(X) @ U

    # @jit
    def xdot_jax(self,X, U):
        return self.f_jax(X) + self.g_jax(X) @ U
        
    def step(self,U, dt): #Just holonomic X,T acceleration

        self.U = U.reshape(-1,1)

        print(f"g vec: {self.g(self.X)}")
        self.X = self.X + ( self.f() + self.g(self.X) @ self.U )*dt
        # print(self.X)
        for i in range(self.num_agents):
            self.X[4*i + 2,0] = wrap_angle(self.X[4*i + 2,0])
        # self.render_plot()
        self.Xs = np.append(self.Xs,self.X,axis=1)
        self.Us = np.append(self.Us,self.U,axis=1)
        return self.X
    
    def rot_mat(self,theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
            ])

    # def render_plot(self):
    #     x = np.array([self.X[0,0],self.X[1,0]])
    #     theta = self.X[2,0]
    #     self.body.set_offsets([x[0],x[1]])
    #     self.axis[0].set_ydata([self.X[1,0],self.X[1,0]+self.radii*np.sin(self.X[2,0])])
    #     self.axis[0].set_xdata( [self.X[0,0],self.X[0,0]+self.radii*np.cos(self.X[2,0])] )
    #     if self.plot_polytope:
    #         points = np.array( [ [-self.width/2,-self.height/2], [self.width/2,-self.height/2], [self.width/2,self.height/2], [-self.width/2,self.height/2] ] )
    #         R = self.rot_mat(theta)
    #         points = (R @ points.T).T  + x     
    #         self.patch.set_xy( points )
            
    # def base_polytopic_location_i(self):
    #     x = np.array([0,0])
    #     points = np.array( [ [x[0]-self.width/2,x[1]-self.height/2], [x[0]+self.width/2,x[1]-self.height/2], [x[0]+self.width/2,x[1]+self.height/2], [x[0]-self.width/2,x[1]+self.height/2] ] )
    #     hull = pt.qhull(points)
    #     return hull.A, hull.b.reshape(-1,1)
    
    # def base_polytopic_location(self):
    #     ## TODO ##
    
    #v1-v3, v6: 1.5, v4-5: 1.0
    #1.0 with base CBF initial bad
    def nominal_controller_i(self, targetX, k_omega = 1.5, k_v = 1.0, k_x = 1.0):
        # k_omega = 3.0#2.0 
        # k_v = 1.0#3.0#0.3#0.15##5.0#0.15
        # k_x = k_v
        print(f"Shape of X: {self.X.shape}. Shape of goal: {targetX.shape}")
        distance = np.linalg.norm( self.X[0:2]-targetX[0:2] )
        # print(f"distance: {distance}")
        if (distance>0.1):
            desired_heading = np.arctan2( targetX[1,0]-self.X[1,0], targetX[0,0]-self.X[0,0] )
            error_heading = wrap_angle( desired_heading - self.X[2,0] )
            if np.abs(error_heading) < np.pi/2:
                speed = min(k_x * distance * np.cos(error_heading), 1.5)
            else:
                speed = 0
            # min(k_x * distance * np.cos(error_heading), 1.5)
        else:
            error_heading = 0
            speed = 0

        omega = k_omega * error_heading #* np.tanh( distance )
        
        # print(f"CBF nominal speed: {speed}, omega:{omega}")
        u_r = k_v * ( speed - self.X[3,0] )
        # print(f"CBF nominal acc: {u_r}, omega:{omega}")
        return np.array([u_r, omega]).reshape(-1,1)
    
    def nominal_controller(self, targetX, other_robot_states, k_omega = 1.5, k_v = 1.0, k_x = 1.0):
        nominal_controls_list = []
        agent_nominal_control = self.nominal_controller_i(targetX, k_omega = k_omega, k_x = k_x, k_v = k_v)
        nominal_controls_list.append(agent_nominal_control)
        num_other_agents = int(len(other_robot_states)/4)
        for i in range(num_other_agents):
            nominal_controls_list.append(np.array([0,0]).reshape(-1,1))
        print(f"Nominal control: {nominal_controls_list} for {num_other_agents+1} agents")
        return np.concatenate(nominal_controls_list, axis=0)
    
    # def nominal_controller_jax(self, targetX, k_omega = 1.5, k_v = 1.0, k_x = 1.0):
    #     distance = jnp.linalg.norm( self.X[0:2]-targetX[0:2] )
    #     if (distance>0.1):
    #         desired_heading = jnp.arctan2( targetX[1,0]-self.X[1,0], targetX[0,0]-self.X[0,0] )
    #         error_heading = wrap_angle( desired_heading - self.X[2,0] )
    #         if jnp.abs(error_heading) < jnp.pi/2:
    #             speed = min(k_x * distance * jnp.cos(error_heading), 1.5)
    #         else:
    #             speed = 0
    #         # speed = min(k_x * distance * jnp.cos(error_heading), 1.5)
    #     else:
    #         error_heading = 0
    #         speed = 0

    #     omega = k_omega * error_heading #* np.tanh( distance )
    #     u_r = k_v * ( speed - self.X[3,0] )
    #     return jnp.array([u_r, omega]).reshape(-1,1)
    
    def barrier_alpha_jax(self, X, otherX, avoidX, d_min = 0.5):
        h = (X[0:2] - avoidX[0:2]).T @ (X[0:2] - avoidX[0:2]) - d_min**2
        h_dot = 2 * (X[0:2] - avoidX[0:2]).T @ ( self.f_jax_i(X)[0:2]  )
        df_dx_i = self.df_dx_jax_i(X)
        dh_dot_dx1 = jnp.append( ( self.f_jax_i(X)[0:2] ).T, jnp.array([[0,0]]), axis = 1 ) + 2 * ( X[0:2] - avoidX[0:2] ).T @ df_dx_i[0:2,:]
        dh_dot_dx2 = - 2 * ( self.f_jax_i(X)[0:2].T )
        return dh_dot_dx1, dh_dot_dx2, h_dot, h
    
    def barrier_humans_alpha_jax(self, X, otherX, targetX, targetU, d_min = 0.5):
        h = (X[0:2] - targetX[0:2]).T @ (X[0:2] - targetX[0:2]) - d_min**2
        h_dot = 2 * (X[0:2] - targetX[0:2]).T @ ( self.f_jax_i(X)[0:2] - targetU[0:2] )
        df_dx_i = self.df_dx_jax_i(X)
        dh_dot_dx1 = jnp.append( ( self.f_jax_i(X)[0:2] - targetU[0:2] ).T, jnp.array([[0,0]]), axis = 1 ) + 2 * ( X[0:2] - targetX[0:2] ).T @ df_dx_i[0:2,:]
        dh_dot_dx2 = - 2 * ( self.f_jax_i(X)[0:2].T -targetU[0:2].T )
        return dh_dot_dx1, dh_dot_dx2, h_dot, h
    
    def barrier_agent_agent_jax_ij(self, X_i, X_j, d_min = 0.5):
        X = jnp.concatenate((X_i, X_j))
        def h(X):
            return (X[0:2] - X[4:6]).T @ (X[0:2] - X[4:6]) - d_min**2
        dh_dx = jax.grad(h, argnums=(0))
        def hdot_func(X):
            hdot = dh_dx(X) @ self.f_jax(X)
            return hdot
        dhdot_dx = jax.grad(hdot_func, argnums=(0))
        dhdot_dx1 = dhdot_dx[0:4]
        dhdot_dx2 = dhdot_dx[4:]
        return dhdot_dx1, dhdot_dx2, hdot_func(X), h(X)


def main():
    robot_init_state = np.array([3.0, 4.0, 0.0, 0.1, 4.0, 5.0, 0.2, 0.1]).reshape(-1,1)
    robots_init = []
    num_agents = 2
    for i in range(num_agents):
        robots_init.append(np.array([robot_init_state[4*i,0], robot_init_state[4*i+1,0], robot_init_state[4*i+2,0], 0.0]).reshape(-1,1))
    robots_init_state = np.hstack(robots_init)
    robot = multi_dynamic_unicycle( None, pos = robots_init_state, plot_polytope=False, num_agents=num_agents)
    print(robot.g(robot.X))

    
if __name__ == '__main__':
    main()
