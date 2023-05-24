# Ros libraries
import rospy
import roslib
import functools # for making copies of subscriber function
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist

# gazebo specific
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

# Other libraries
import numpy as np
import casadi as cd
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from crowd import crowd




