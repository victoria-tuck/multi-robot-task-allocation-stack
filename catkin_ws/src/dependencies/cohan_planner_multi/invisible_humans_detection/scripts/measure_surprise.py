#!/usr/bin/env python
# Brief: Node for suprise metric
# Author: Phani Teja Singamaneni

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from utils import norm2D

class Metrics(object):
  def __init__(self):
    rospy.init_node('metrics_node')
    rospy.Subscriber("/move_base/HATebLocalPlannerROS/hateb_log", String, self.logCB)
    rospy.Subscriber('/invisible_humans_corners', PoseArray, self.invCB)

    self.log = {}
    self.poses = PoseArray()

    rospy.spin()


  def logCB(self, msg):
    self.log = {}
    tmp = msg.data.split(';')
    # print(msg.data)
    for i in range(0, len(tmp)):
      data = tmp[i].split()
      if(len(data) > 1):
        self.log[data[0][:-1]] = [float(data[1]), float(data[2])]

    name = 'corner'
    for i in range(0,len(self.poses.poses)):
      self.log[name+str(i)] = [self.poses.poses[i].position.x, self.poses.poses[i].position.y]
    # print(self.log)
    self.meaure_surprise()

  def invCB(self, msg):
    self.poses = msg

  def meaure_surprise(self):
    name='corner'
    dist = []
    surprise = 0.0
    for i in range(0,len(self.poses.poses)):
      dist.append((i, norm2D(self.log[name+str(i)],self.log['position'])))

    if(len(dist)>0):
      dist.sort(key=self.takeSecond)
      # print(dist)
      surprise = np.linalg.norm([self.log['velocity'][0],self.log['velocity'][1]])/dist[0][1]
    print(surprise)

  def takeSecond(self, elem):
    return elem[1]



if __name__ == '__main__':
  met = Metrics()
