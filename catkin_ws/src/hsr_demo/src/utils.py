import numpy as np

def wrap_angle(theta):
    return np.arctan2( np.sin(theta), np.cos(theta) )