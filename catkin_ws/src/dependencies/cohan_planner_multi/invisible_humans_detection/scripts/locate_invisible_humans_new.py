#!/usr/bin/env python
# Brief: Node for detecting and publishing invisible humans
# Author: Phani Teja Singamaneni

import rospy
import numpy as np
import tf2_ros
import math
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point32, Quaternion, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from transformations import quaternion_from_euler
from utils import *
#import matplotlib.pyplot as plt


class InvisibleHumans(object):
  def __init__(self):
    rospy.init_node('invisible_humans_node')

    self.x = []
    self.y = []
    self.x_vis = []
    self.y_vis = []
    self.corners = [[],[]]
    self.rays = [[],[]]
    self.centers = [[],[]]
    self.info = []
    self.map = []
    self.scan = []
    self.mid_scan = 0.0

    rospy.Subscriber('/map_scanner/map_scan', LaserScan, self.laserCB)
    rospy.Subscriber("/map", OccupancyGrid, self.mapCB)
    self.pub_invis_human_viz = rospy.Publisher('invisible_humans_markers', MarkerArray, queue_size = 1)
    self.pub_invis_human = rospy.Publisher('/move_base/HATebLocalPlannerROS/invisible_humans', ObstacleArrayMsg, queue_size = 1)
    self.pub_invis_human_corners = rospy.Publisher('invisible_humans_corners', PoseArray, queue_size = 1)
    self.pub_invis_humans_pos_ = rospy.Publisher('invisible_humans', PoseArray, queue_size = 1)


    #Intialize tf2 transform listener
    self.tf = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tf)

    # rospy.Timer(rospy.Duration(0.01), self.locate_humans)

    # Keep the node alive
    rospy.spin()

  def laserCB(self, scan):
    self.saved = False
    prev_point = [0.0,0.0]
    self.scan = scan
    self.x = []
    self.y = []
    self.x_vis = []
    self.y_vis = []
    self.opp_ang = []
    self.corners = [[],[]]
    self.rays = [[],[]]
    self.hum_dir = []
    self.mid_scan = scan.ranges[len(scan.ranges)/2]
    for i in range(0,len(scan.ranges)):
      angle = scan.angle_min + (i * scan.angle_increment)
      if abs(angle) < 1.57:
        self.x_vis[len(self.x_vis):] = [scan.ranges[i]*np.cos(angle)]
        self.y_vis[len(self.y_vis):] = [scan.ranges[i]*np.sin(angle)]
        idx = len(self.x_vis) - 1
        if(idx>0):
          dist = np.linalg.norm([prev_point[0]-self.x_vis[idx], prev_point[1]-self.y_vis[idx]])
          check = (abs(prev_point[0]-self.x_vis[idx]) >= 0.5 or abs(prev_point[1]-self.y_vis[idx]) >= 0.5)
          under_rad = min(np.linalg.norm([self.x_vis[idx], self.y_vis[idx]]), np.linalg.norm([prev_point[0], prev_point[1]]))
          if(dist>0.15 and under_rad<=5.0 and check):
            if(np.linalg.norm([prev_point[0], prev_point[1]]) < np.linalg.norm([self.x_vis[idx], self.y_vis[idx]])):
              self.corners[0][len(self.corners[0]):] = [prev_point[0]]
              self.corners[1][len(self.corners[1]):] = [prev_point[1]]
              self.rays[0][len(self.rays[0]):] = [self.x_vis[idx]]
              self.rays[1][len(self.rays[1]):] = [self.y_vis[idx]]
              self.hum_dir[len(self.hum_dir):] = 'p'
              # self.opp_ang[len(self.opp_ang):] = [-ang]
            else:
              self.corners[0][len(self.corners[0]):] = [self.x_vis[idx]]
              self.corners[1][len(self.corners[1]):] = [self.y_vis[idx]]
              self.rays[0][len(self.rays[0]):] = [prev_point[0]]
              self.rays[1][len(self.rays[1]):] = [prev_point[1]]
              self.hum_dir[len(self.hum_dir):] = 'n'
            self.opp_ang[len(self.opp_ang):] = [i]

        prev_point = [self.x_vis[idx], self.y_vis[idx]]

      self.x[len(self.x):] = [scan.ranges[i]*np.cos(angle)]
      self.y[len(self.y):] = [scan.ranges[i]*np.sin(angle)]
    self.saved = True
    self.locate_humans()


  def mapCB(self, map):
    if len(self.map) < 2:
      self.info = map.info
      self.map = map.data

  def publish_to_cohan_obstacles(self, humans):

    obstacle_msg = ObstacleArrayMsg()
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map

    for i in range(0,len(humans)):
      # Add point obstacle
      # print(humans[i][0])
      obstacle_msg.obstacles.append(ObstacleMsg())
      obstacle_msg.obstacles[i].radius = 0.07
      obstacle_msg.obstacles[i].id = i
      obstacle_msg.obstacles[i].polygon.points = [Point32()]
      obstacle_msg.obstacles[i].polygon.points[0].x = humans[i][0]
      obstacle_msg.obstacles[i].polygon.points[0].y = humans[i][1]
      obstacle_msg.obstacles[i].polygon.points[0].z = self.mid_scan

      yaw = math.atan2(humans[i][3], humans[i][2])
      q = quaternion_from_euler(0,0,yaw)
      obstacle_msg.obstacles[i].orientation = Quaternion(*q)

      obstacle_msg.obstacles[i].velocities.twist.linear.x = humans[i][2]
      obstacle_msg.obstacles[i].velocities.twist.linear.y = humans[i][3]
      obstacle_msg.obstacles[i].velocities.twist.linear.z = self.scan.ranges[humans[i][4]]
      obstacle_msg.obstacles[i].velocities.twist.angular.x = 0
      obstacle_msg.obstacles[i].velocities.twist.angular.y = 0
      obstacle_msg.obstacles[i].velocities.twist.angular.z = 0
    self.pub_invis_human.publish(obstacle_msg)

  # def add_to_memory(self):

  def locate_humans(self):
    # print("locating")
    # The robot position is same irrespective of the map position as the laser is with respect to laser frame
    # Hence you can use the fixed cordinates for the robot
    #   position:
    #   x: -0.275
    #   y: -0.055
    #   z: -0.1
    # orientation:
    #   x: 0.0
    #   y: 0.0
    #   z: 0.0
    #   w: 1.0
    self.centers = [[],[]]
    corner_array = PoseArray()
    corner_array.header.stamp = rospy.Time.now()
    corner_array.header.frame_id = "map"
    inv_array = PoseArray()
    inv_array.header.stamp = rospy.Time.now()
    inv_array.header.frame_id = "map"
    marker_array = MarkerArray()
    inv_humans = []
    m_id = 0
    point_transform = self.tf.lookup_transform("map", "base_footprint", rospy.Time(0),rospy.Duration(0.3))

    if len(self.map) > 1 and len(self.corners[0])>0:
      for i in range(0,len(self.corners[0])):
        x = self.corners[0][i]
        y = self.corners[1][i]
        x1 = self.rays[0][i]
        y1 = self.rays[1][i]

        if math.isnan(x):
          x = 7.0
        if math.isnan(y):
          y = 7.0
        if math.isnan(x1):
          x1 = 7.0
        if math.isnan(y1):
          y1 = 7.0

        l_or_r = checkPoint([0.0,0.0], [1.0,0.0],[x,y])

        v_mag = np.linalg.norm([x1-x, y1-y])
        ux = (x1-x)/v_mag
        uy = (y1-y)/v_mag
        alp = 0.2
        center = [0,0]
        hum_rad = 0.3
        remove_detection = False
        xt = x + hum_rad*ux
        yt = y + hum_rad*uy

        while(True):
          if self.hum_dir[i] == 'p':
            pt = getRightPoint([x,y],[x1,y1],[xt,yt],dist = hum_rad)
          elif self.hum_dir[i] == 'n':
            pt = getLeftPoint([x,y],[x1,y1],[xt,yt],dist = hum_rad)

          angle = math.atan2(pt[1], pt[0])
          ang_idx = int((angle-self.scan.angle_min)/self.scan.angle_increment)

          if self.scan.ranges[ang_idx] > np.linalg.norm(pt):
            xt = xt + alp*ux
            yt = yt + alp*uy
            continue

          center = [(xt+pt[0])/2, (yt+pt[1])/2]
          # center = [pt[0], pt[1]]
          overlap = False
          n_div = 10
          for ri in range(0,n_div):
            l_pt, r_pt = get2Points([xt, yt],pt, radius = ((ri+1)/n_div)*(1.5*hum_rad)) # Add 0.1 clearance for the detection
            in_pose_l = PoseStamped()
            in_pose_r = PoseStamped()
            in_pose_temp = PoseStamped()

            in_pose_l.pose.position.x = l_pt[0]
            in_pose_l.pose.position.y = l_pt[1]
            in_pose_l.pose.orientation.w = 1
            in_pose_r.pose.position.x = r_pt[0]
            in_pose_r.pose.position.y = r_pt[1]
            in_pose_r.pose.orientation.w = 1
            in_pose_temp.pose.position.x = center[0]
            in_pose_temp.pose.position.y = center[1]
            in_pose_temp.pose.orientation.w = 1

            robot_pose = PoseStamped()
            robot_pose.pose.position.x = 0.0
            robot_pose.pose.position.y = 0.0
            robot_pose.pose.orientation.w = 1

            p1 = tf2_geometry_msgs.do_transform_pose(in_pose_l, point_transform)
            p2 = tf2_geometry_msgs.do_transform_pose(in_pose_r, point_transform)
            p3 = tf2_geometry_msgs.do_transform_pose(in_pose_temp, point_transform)
            p4 = tf2_geometry_msgs.do_transform_pose(robot_pose, point_transform)
            mx_l, my_l = worldTomap(p1.pose.position.x,p1.pose.position.y, self.info)
            mx_r, my_r = worldTomap(p2.pose.position.x,p2.pose.position.y, self.info)
            mx_c, my_c = worldTomap(p3.pose.position.x,p3.pose.position.y, self.info)

            m_idx_l = getIndex(mx_l, my_l, self.info)
            m_idx_r = getIndex(mx_r, my_r, self.info)
            m_idx_c = getIndex(mx_c, my_c, self.info)


            if m_idx_c > (len(self.map) - 1) or m_idx_l > (len(self.map) - 1) or m_idx_r > (len(self.map) - 1):
              remove_detection = True
              break

            if (self.map[m_idx_l] == 0 and self.map[m_idx_r] == 0 and self.map[m_idx_c] == 0):
              continue
            else:
              overlap =  True
              break


          xt = xt + alp*ux
          yt = yt + alp*uy

          if (np.linalg.norm([xt-x,yt-y])>=np.linalg.norm([x1-x,y1-y])) or (np.linalg.norm([xt,yt])>=7.0):
            remove_detection = True
            break

          if overlap:
            continue
          else:
            break

        # Remove false detections
        if remove_detection:
          continue


        self.centers[0][len(self.centers[0]):] = [pt[0]]
        self.centers[1][len(self.centers[1]):] = [pt[1]]


        _c_pose = PoseStamped()
        _c_pose.header.frame_id = "base_footprint"
        _c_pose.pose.position.x = x
        _c_pose.pose.position.y = y
        _c_pose.pose.orientation.w = 1
        corner_pos = tf2_geometry_msgs.do_transform_pose(_c_pose, point_transform)
        corner_array.poses.append(corner_pos.pose)
        # Filter the detections based on the 2D map using ray tracing
        # n_div = 50
        # Dx = (p4.pose.position.x - p3.pose.position.x)/n_div
        # Dy = (p4.pose.position.y - p3.pose.position.y)/n_div
        # ray_pos_x = p3.pose.position.x
        # ray_pos_y = p3.pose.position.y
        # for j in range(0,n_div):
        #   ray_mx, ray_my = worldTomap(ray_pos_x,ray_pos_y, self.info)
        #   if (self.map[getIndex(ray_mx, ray_my, self.info)] > 0 and self.map[getIndex(ray_mx, ray_my, self.info)] < 50) or self.map[getIndex(ray_mx, ray_my, self.info)] == -1:
        #     remove_detection = True
        #     break
        #   ray_pos_x += Dx
        #   ray_pos_y += Dy
        #
        # # Remove false detections
        # if remove_detection:
        #   continue

        vel_ux = p4.pose.position.x - p3.pose.position.x
        vel_uy = p4.pose.position.y - p3.pose.position.y
        vec_ang = math.atan2(vel_uy, vel_ux)

        inv_humans.append([p3.pose.position.x, p3.pose.position.y, 1.5*math.cos(vec_ang), 1.5*math.sin(vec_ang), self.opp_ang[i]])

        yaw = math.atan2(1.5*math.sin(vec_ang), 1.5*math.cos(vec_ang))
        q = quaternion_from_euler(0,0,yaw)
        _inv_pose = PoseStamped()

        arrow = Marker()
        arrow.header.frame_id = "map"
        arrow.id = m_id
        arrow.type = arrow.ARROW
        arrow.action = arrow.ADD
        arrow.pose.orientation = Quaternion(*q)
        _inv_pose.pose.orientation = Quaternion(*q)
        arrow.pose.position.x = p3.pose.position.x
        arrow.pose.position.y = p3.pose.position.y
        _inv_pose.pose.position.x = p3.pose.position.x
        _inv_pose.pose.position.y = p3.pose.position.y
        _inv_pose.pose.position.z = self.mid_scan
        _inv_pose.pose.orientation.z = self.scan.ranges[self.opp_ang[i]]
        arrow.pose.position.z = 0.0
        t = rospy.Duration(0.1)
        arrow.lifetime = t
        arrow.scale.x = 0.6
        arrow.scale.y = 0.1
        arrow.scale.z = 0.1
        arrow.color.a = 1.0
        arrow.color.b = 1.0

        m_id += 1

        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = m_id
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        marker.pose.position.x = p3.pose.position.x #(p1.pose.position.x + p2.pose.position.x)/2
        marker.pose.position.y = p3.pose.position.y #(p1.pose.position.y + p2.pose.position.y)/2 + 0.055
        marker.pose.position.z = 0.6
        t = rospy.Duration(0.1)
        marker.lifetime = t
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 1.2
        marker.color.a = 1.0
        marker.color.r = 1.0

        m_id += 1

        marker_array.markers.append(marker)
        marker_array.markers.append(arrow)
        inv_array.poses.append(_inv_pose.pose)
      self.pub_invis_human_viz.publish(marker_array)
      self.publish_to_cohan_obstacles(inv_humans)
      # self.pub_invis_human_corners.publish(corner_array)
      self.pub_invis_humans_pos_.publish(inv_array)
      # self.save_contours()
      # self.pub_invis_human.publish(inv_humans)
'''
  def save_contours(self):
    self.drawing = True
    # print(self.corners)
    if(self.saved and len(self.x)>0):
      if (len(self.x) == len(self.y)) and (len(self.corners[0])==len(self.corners[1])):
        plt.plot(self.x, self.y)
        plt.plot(self.rays[0], self.rays[1],'k.')
        plt.plot(self.x_vis, self.y_vis, 'r')
        plt.plot(self.corners[0],self.corners[1], 'yo')
        plt.plot(self.centers[0],self.centers[1], 'go')
        plt.arrow(-0.275,-0.55,1.0,0.0, length_includes_head=True,
            head_width=0.2, head_length=0.2)
        plt.plot(-0.275,-0.55,'ro')
        # plt.axis('scaled')
        plt.title("Laser Contour")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.axis([-7.5, 7.5, -7.5, 7.5])
        plt.draw()
        plt.pause(0.001)
        # plt.savefig("contour"+str(self.data_counter)+".png")
        # self.data_counter += 1
        plt.clf()
    self.drawing = False
'''


if __name__ == '__main__':
  cont = InvisibleHumans()
