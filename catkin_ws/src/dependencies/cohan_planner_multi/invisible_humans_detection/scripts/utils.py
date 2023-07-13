#!/usr/bin/env python
# Brief: Some utility functions
# Author: Phani Teja Singamaneni

import numpy as np

def getLeftPoint(P1,P2,P3, dist=1):
  ## Get a point at on the left a distance of 'dist' perpedicular to P1P2 and passing through P3
  X = P2[0]-P1[0]
  Y = P2[1]-P1[1]
  px = P3[0] - (dist*Y/np.linalg.norm([X,Y]))
  py = P3[1] + (dist*X/np.linalg.norm([X,Y]))
  return [px, py]

def getRightPoint(P1,P2,P3, dist=1):
  ## Get a point on the right at a distance of 'dist', perpedicular to P1P2 and passing through P3
  X = P2[0]-P1[0]
  Y = P2[1]-P1[1]
  px = P3[0] + (dist*Y/np.linalg.norm([X,Y]))
  py = P3[1] - (dist*X/np.linalg.norm([X,Y]))
  return [px, py]

def checkPoint(O,P1,P2):
  ## Returns +1 if the point P2 is on left of OP1 or -1 if its on the right, 0 if collinear
  o = np.array(O)
  p1 = np.array(P1)
  p2 = np.array(P2)
  return np.sign(np.cross(p1-o,p2-o))

def get2Points(P1,P2,radius):
  # P3 = [(P1[0]+P2[0])/2, (P1[1]+P2[1])/2]
  P3 = P2
  lP = getLeftPoint(P1,P2,P3,radius)
  rP = getRightPoint(P1,P2,P3,radius)
  return lP, rP

def worldTomap(wx, wy, info):
  resolution = info.resolution
  origin_x = info.origin.position.x
  origin_y = info.origin.position.y
  mx = int(((wx - origin_x) / resolution))
  my = int(((wy - origin_y) / resolution))
  return [mx, my]

def getIndex(mx, my, info):
  size_x = info.width
  return my * size_x + mx;

def norm2D(a, b):
  return np.linalg.norm([a[0]-b[0],a[1]-b[1]])
