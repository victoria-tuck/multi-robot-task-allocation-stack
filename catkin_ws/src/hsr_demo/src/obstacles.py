import numpy as np
import matplotlib.pyplot as plt
import polytope as pt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle

class rectangle:

    def __init__(self, ax, pos = np.array([0,0]), width = 1.0, height = 1.0):
        self.X = pos.reshape(-1,1)
        self.width = width
        self.height = height
        self.A, self.b = self.initial_polytopic_location()
        self.id = id
        self.ax = ax
        self.type = 'rectangle'
        
        self.rect = Rectangle((self.X[0,0]-self.width/2,self.X[1,0]-self.height/2),self.width,self.height,linewidth = 1, edgecolor='k',facecolor='k')
        self.ax.add_patch(self.rect)
        
        self.render()

    def render(self):
        self.rect.set_xy((self.X[0,0]-self.width/2,self.X[1,0]-self.height/2))      
        
    
        
    def initial_polytopic_location(self):
        x = np.array([self.X[0,0],self.X[1,0]])
        points = np.array( [ [x[0]-self.width/2,x[1]-self.height/2], [x[0]+self.width/2,x[1]-self.height/2], [x[0]+self.width/2,x[1]+self.height/2], [x[0]-self.width/2,x[1]+self.height/2] ] )
        hull = pt.qhull(points)
        return hull.A, hull.b.reshape(-1,1)
    
    def polytopic_location(self):
        return self.A, self.b


class circle:

    def __init__(self, ax, pos = np.array([0,0]),radius = 1.0):
        self.X = pos.reshape(-1,1)
        self.radius = radius
        self.id = id
        self.type = 'circle'

        self.render(ax)

    def render(self,ax):
        circ = plt.Circle((self.X[0],self.X[1]),self.radius,linewidth = 1, edgecolor='k',facecolor='k')
        ax.add_patch(circ)

