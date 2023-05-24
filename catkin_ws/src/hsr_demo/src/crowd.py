import numpy as np
import matplotlib.pyplot as plt
# from obstacles import rectangle, circle
from matplotlib.animation import FFMpegWriter
import casadi as cd
import time

class crowd:
    
    def __init__(self, ax, crowd_center = np.array([0,0]), num_people = 10, dt = 0.01, horizon = 100, paths_file = []):
        '''
        X0: iniytial state
        dt: simulation time step
        ax: plot axis handle
        id: robot id
        '''
        
        self.type = 'SingleIntegrator2D'        
        self.num_people = num_people
        self.X0 = 40*( np.random.rand(2,num_people) - np.array([[0.5],[0.5]]) ) + crowd_center.reshape(-1,1)
        for i in range(num_people):
            if self.X0[0,i] < 0:
                self.X0[0,i] = np.clip( self.X0[0,i], -20, -5 )
            else:
                self.X0[0,i] = np.clip( self.X0[0,i], 5, 20 )
        self.X = np.copy(self.X0)
        self.U = np.zeros((2,num_people))
        self.dt = dt
        self.horizon = horizon
        self.ax = ax
        self.t = 0
        self.counter = 0
        
        # Set goals for each human
        self.goals = -np.copy(self.X0)
        
        if paths_file != []:
            with open(paths_file,'rb') as f:
                self.paths = np.load(f)
            # self.X = self.paths[0,0: self.num_people], self.paths[1,0: self.num_people] + 1.0 # 1.0
            # print(f"paths: {self.paths[0:2,1]}, {self.paths[0:2,1+self.num_people]}, {self.paths[0:2,1+2*self.num_people]}, {self.paths[0:2,1+3*self.num_people]}")
            # exit()
            self.X = self.paths[0:2,0: self.num_people] #+ np.array([0,1]).reshape(-1,1) # 1.0
            self.X_init = np.copy(self.X)
            self.U_init = (self.paths[0:2,self.num_people: 2*self.num_people] - self.X_init)/dt
            
            # Animate trajectories
            self.body = self.ax.scatter(self.paths[0,0: self.num_people], self.paths[1,0: self.num_people],c='g',alpha=0.5,s=50)#50
            self.plot_counter = 1
            
        self.distance_to_polytope = cd.Opti()
        self.A = self.distance_to_polytope.parameter(4,2)
        self.b = self.distance_to_polytope.parameter(4,1)
        self.y = self.distance_to_polytope.variable(2,1)
        self.curr_x = self.distance_to_polytope.parameter(2,1)
        dist = self.y - self.curr_x
        self.distance_to_polytope.minimize( cd.mtimes(dist.T, dist) )
        self.distance_to_polytope.subject_to( cd.mtimes( self.A, self.y ) <= self.b )
        option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
        self.distance_to_polytope.solver("ipopt", option)
        
        
        
    # def render_plot(self):
    #     self.body.set_offsets(self.paths[:,self.plot_counter*self.num_people: (self.plot_counter+1)*self.num_people].T)
    #     self.plot_counter = self.plot_counter + 1
        
    def get_future_states(self,t,dt,mpc_horizon):
        states = np.copy(self.X)
        
        t_temp = t
        counter_temp = self.counter
        while t_temp < (t + dt*mpc_horizon):
            if t_temp<=(counter_temp*self.dt):
                if t_temp>=counter_temp*self.dt:
                    counter_temp += 1
            counter = counter_temp - 1
            t_temp += dt
            U = (self.paths[:,(counter+1)*self.num_people: (counter+2)*self.num_people] - self.paths[:,counter*self.num_people: (counter+1)*self.num_people])/self.dt
            states = np.append(states, states[:,-self.num_people:] + U * dt, axis=1)
            
        return states[:,0:self.num_people*(mpc_horizon+1)]
    
    def current_position(self, t, dt):
        if t<=(self.horizon*self.dt):
            if t>=self.counter*self.dt:
                self.counter += 1
        counter = self.counter - 1
        # counter = int(1.001*t/self.dt)
        U = (self.paths[:,(counter+1)*self.num_people: (counter+2)*self.num_people] - self.paths[:,counter*self.num_people: (counter+1)*self.num_people])/self.dt
        self.X = self.X + U * dt
        # current_pos = self.paths[:,counter*self.num_people: (counter+1)*self.num_people] + U * (t-counter*self.dt)
        # print(f"t:{t}, counter:{counter}, change in time: {(t-counter*self.dt)}")
        # return current_pos
        return np.copy(self.X)
    
    def reset_pose(self):
        self.X = np.copy(self.X_init)
        self.counter = 0
        
    def render_plot(self, current_pos):# t, dt):
        # counter = int(1.001*t/self.dt)
        # U = self.paths[:,(counter+1)*self.num_people: (counter+2)*self.num_people] - self.paths[:,counter*self.num_people: (counter+1)*self.num_people]
        # # U = self.paths[:,(counter+1)*self.num_people: (counter+2)*self.num_people] - self.paths[:,counter*self.num_people: (counter+1)*self.num_people]
        # current_pos = self.paths[:,counter*self.num_people: (counter+1)*self.num_people] + U * (t-counter*self.dt)
        self.body.set_offsets(current_pos.T)
        # self.plot_counter = self.plot_counter + 1
        
    
    def plan_paths(self, obstacles):
        # Use MPC to plan paths for all humans in centralized manner
        opti = cd.Opti()
        
        X = opti.variable(2,self.num_people*(self.horizon+1))
        U = opti.variable(2,self.num_people*self.horizon)
        
        # Initial state
        initial_state_error = X[:,0:self.num_people] -  self.X0
        
        for i in range(self.horizon):
            
            # Dynamics
            opti.subject_to( X[:,(i+1)*self.num_people:(i+2)*self.num_people] == X[:,i*self.num_people:(i+1)*self.num_people] + U[:,i*self.num_people:(i+1)*self.num_people]*self.dt )
            
            for j in range(self.num_people):
                # Input constraints
                control_input = U[:,i*self.num_people+j]
                opti.subject_to( cd.mtimes( control_input.T, control_input ) <= 2 )
                
                # Collision avoidance with other people
                for k in range(j+1,self.num_people):
                    dist = X[:,i*self.num_people+j] - X[:,i*self.num_people+k]
                    opti.subject_to( cd.mtimes(dist.T, dist) >= 0.3 )
                    
        # Goal location
        final_state_error = X[:,(self.horizon)*self.num_people:(self.horizon+1)*self.num_people] - self.goals
        
        cost = 100*cd.norm_fro(initial_state_error)**2 + cd.norm_fro(final_state_error)**2
        opti.minimize(cost)
        option = {"verbose": True, "ipopt.print_level": 0, "print_time": 0}
        opti.solver("ipopt", option)
        sol = opti.solve();
        return sol.value(X)
    
    def attractive_potential(self, x1, x2):
        k = 10
        potential = np.linalg.norm(x1-x2)**2
        force = - 2 * (x1 - x2)
        return k * force
    
    def repulsive_potential(self,x1, x2):
        k = 20
        min_dist = 0.5
        
        dist_thr = 3.0
        dist = np.linalg.norm(x1-x2) - min_dist
        if dist < dist_thr:
            potential = ( 1/dist - 1/dist_thr )**2
            force =  ( 1/dist - 1/dist_thr ) / dist**2 * 2* (x1 - x2)
        else:
            potential = 0;
            force = 0
        
        if (dist < min_dist):
            print(f"Violated, dist:{dist}")
            # exit()
        return k * force        
    
    def repulsive_potential_obstacle_circle(self,x1, x2, radius):
        k = 20
        min_dist = 0.2 + radius
        dist_thr = 5.0
        dist = np.linalg.norm(x1-x2) - min_dist
        if dist < dist_thr:
            potential = ( 1/dist - 1/dist_thr )**2
            force = ( 1/dist - 1/min_dist ) / dist**2 * 2* (x1 - x2)
        else:
            potential = 0
            force = 0

        if (dist <= 0.01):
            print(f"Violated, dist:{dist}")

        return k * force        
    
    def repulsive_potential_obstacle_rectangle(self, x1, A, b):
        self.distance_to_polytope.set_value( self.A, A )
        self.distance_to_polytope.set_value( self.b, b )
        self.distance_to_polytope.set_value( self.curr_x, x1 )
        self.distance_to_polytope.set_value( self.A, A )
        opt_sol = self.distance_to_polytope.solve()
        return self.repulsive_potential(x1, opt_sol.value(self.y).reshape(-1,1))
    
    def plan_paths_potential_field(self, obstacles):
        t = 0
        paths = np.copy(self.X)
        while (t < self.horizon):
            for i in range( num_people ):
                force = 0
                # attractive potential
                force += self.attractive_potential( self.X[:,i].reshape(-1,1), self.goals[:,i].reshape(-1,1) )
                
                # Collision avoidance with other people
                for j in range(num_people):    
                    if j==i:
                        continue
                    # print("human")           
                    force += self.repulsive_potential( self.X[:,i].reshape(-1,1), self.X[:,j].reshape(-1,1) )
                for j in range(len(obstacles)):
                    # print("obs")
                    if obstacles[j].type == 'rectangle':
                        force += self.repulsive_potential_obstacle_rectangle( self.X[:,i].reshape(-1,1), obstacles[j].A, obstacles[j].b )
                    elif obstacles[j].type == 'circle':
                        force += self.repulsive_potential_obstacle_circle( self.X[:,i].reshape(-1,1), obstacles[j].X, obstacles[j].radius )
                self.U[:,i] = (self.U[:,i] + np.clip( force, -1.0, 1.0  )[:,0])/2
            self.X = self.X  + self.U * self.dt
            paths = np.append(paths, self.X, axis=1)        
            t = t + 1
        return paths
if 0:    
    # Set Figure
    plt.ion()
    fig = plt.figure()
    ax = plt.axes(xlim=(-20,20), ylim=(-20,20))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    dt = 0.2
    tf = 40.0#20.0
    horizon = int(tf/dt)
    num_people = 20
    
    obstacles = []
    obstacles.append( rectangle( ax, pos = np.array([0,0.0]), width = 5.0, height = 5.0 ) )
    obstacles.append( circle( ax, pos = np.array([1.0,-1.0]), radius = 3.0 ) )
    # obstacles.append( rectangle( ax, pos = np.array([1,-1.5]) ) )
    
    # obstacles.append( rectangle( ax, pos = np.array([0,0.5]), width = 2.5 ) )        
    # obstacles.append( rectangle( ax, pos = np.array([-2.5,-5.0]), width = 10.0 ) )
    # obstacles.append( rectangle( ax, pos = np.array([-1.28,2.0]), height = 4.0 ) )
    # obstacles.append( rectangle( ax, pos = np.array([-7.0,1.0]), height = 12.0 ) )
    # plt.show()
    humans = crowd(ax, crowd_center = np.array([0,0]), num_people = num_people, dt = dt, horizon = horizon)
    
    # paths = humans.plan_paths(obstacles)
    paths = humans.plan_paths_potential_field(obstacles)
    
    # Save Data
    # with open('paths_n20_tf40_v1_test.npy','wb') as f:
    #     np.save(f, paths)
    
    # Plot trajectory
    # for i in range(num_people):
    #     plt.plot(paths[0,i::num_people], paths[1,i::num_people])

    # Animate trajectories
    body = ax.scatter(paths[0,0: num_people], paths[1,0: num_people],c='g',alpha=0.5,s=70)
    goals = ax.scatter(humans.goals[0,0: num_people], humans.goals[1,0: num_people],c='r',alpha=0.1,s=70)
    
    movie_name = 'Videos/crowd_sim_potential_fields_obstacles.mp4'
    metadata = dict(title='Movie Test', artist='Matplotlib',comment='Movie support!')
    writer = FFMpegWriter(fps=15, metadata=metadata)
    with writer.saving(fig, movie_name, 100): 
        for i in range(horizon-1):
            body.set_offsets(paths[:,i*num_people: (i+1)*num_people].T)
            time.sleep(0.1)
            writer.grab_frame()
            fig.canvas.draw()
            fig.canvas.flush_events()
    plt.ioff()       
        
    plt.show()
    
# paths.npy
# dt = 0.5
# tf = 10.0
# horizon = int(tf/dt)
# num_people = 10

# paths2.npy


# potential fields:
# 1.how to avoid perfectly symmetric situations