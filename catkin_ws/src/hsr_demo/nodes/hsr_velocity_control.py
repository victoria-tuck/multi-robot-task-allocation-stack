#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import controller_manager_msgs.srv
import geometry_msgs.msg
import rospy
import numpy as np

from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState

def wrap_angle(angle):
    return np.arctan2( np.sin(angle), np.cos(angle) ) 

rospy.init_node('test')


 #get robot Pose
rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
model = GetModelStateRequest()
model.model_name = 'hsrb'
result = get_model_srv(model)
pose = np.zeros((3,1))
pose[0,0] = result.pose.position.x
pose[1,0] = result.pose.position.y
pose[2,0] = wrap_angle(2.0*np.arctan2(result.pose.orientation.z, result.pose.orientation.w))
print(f"pose: {pose[0,0]}, {pose[1,0]}, {pose[2,0]*180.0/np.pi}")
# initialize ROS publisher
pub = rospy.Publisher(
    '/hsrb/command_velocity',
    geometry_msgs.msg.Twist, queue_size=10)

# wait to establish connection between the controller
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)

# make sure the controller is running
rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
list_controllers = rospy.ServiceProxy(
    '/hsrb/controller_manager/list_controllers',
    controller_manager_msgs.srv.ListControllers)
running = False
while running is False:
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        if c.name == 'omni_base_controller' and c.state == 'running':
            running = True

# fill ROS message
tw = geometry_msgs.msg.Twist()
tw.linear.x = 0.0#1.0

# publish ROS message
pub.publish(tw)