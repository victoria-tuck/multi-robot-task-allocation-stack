#!/usr/bin/env python

# Brief: This node filters the /tracked_agents from the laser data and publishes the filtered laser scan used for hateb
# Author: Phani Teja Singamaneni

import rospy
import sys
import tf2_ros
import numpy as np
from sensor_msgs.msg import LaserScan
from cohan_msgs.msg import TrackedAgents, TrackedSegmentType
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped

class AgentFilter(object):
    def __init__(self, ns, sim):
        rospy.init_node('agent_filter')
        self.ns_ = ns
        self.rate = rospy.Rate(50.0)
        self.filtered_scan = LaserScan()
        self.segment_type = TrackedSegmentType.TORSO
        self.agents = []
        self.laser_transform = TransformStamped()
        self.got_scan = False

        #Intialize tf2 transform listener
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf)

        if sim=="stage":
            base = "/base_scan"
        elif sim=="morse":
            base = "/scan"

        if(self.ns_ is not ""):
            base = "/"+self.ns_+base

        rospy.Subscriber(base, LaserScan, self.laserCB)
        rospy.Subscriber("tracked_agents", TrackedAgents, self.agentsCB)
        self.laser_pub = rospy.Publisher("base_scan_filtered", LaserScan, queue_size=10)

        rospy.Timer(rospy.Duration(0.02), self.publishScan)

        # Keep the node alive
        rospy.spin()

    def laserCB(self, scan):
        filtered_scan = scan
        filtered_scan.ranges = list(scan.ranges)
        filtered_scan.header.stamp = rospy.Time.now()

        try:
            base_link = "base_laser_link"
            if(self.ns_ is not ""):
                base_link = self.ns_ +"/" + base_link
            self.laser_transform = self.tf.lookup_transform("map", base_link, rospy.Time(),rospy.Duration(5.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        if(self.laser_transform.header.frame_id is not ''):
            laser_pose = self.laser_transform.transform.translation

            rot = self.laser_transform.transform.rotation
            r,p,y = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            base_laser_dir = [np.cos(y), np.sin(y)]


            # Filtering agents from the scan
            for pose_type in self.agents:
                rh_vec = [pose_type[0].position.x - laser_pose.x, pose_type[0].position.y - laser_pose.y]
                sign = base_laser_dir[0]*-rh_vec[1] + base_laser_dir[1]*rh_vec[0]
                sign = sign/abs(sign)
                t_angle = scan.angle_max - scan.angle_min
                mid_angle = t_angle/2 - sign*np.arccos((base_laser_dir[0]*rh_vec[0]+base_laser_dir[1]*rh_vec[1])/(np.linalg.norm(rh_vec)))

                mid_idx = int((mid_angle)/scan.angle_increment)
                if(mid_idx>=len(scan.ranges)):
                    continue

                if pose_type[1] is 0:
                    r = 0.6
                else:
                    r = 0.4
                d = np.linalg.norm(rh_vec)
                mr = scan.ranges[mid_idx]

                if(mr<=(d-r)):
                    continue

                if(r<=d):
                    beta = np.arcsin(r/d)
                else:
                    beta = np.pi/2

                min_idx = int(np.floor((mid_angle-beta)/scan.angle_increment))
                max_idx = int(np.ceil((mid_angle+beta)/scan.angle_increment))

                for i in range(min_idx, max_idx):
                    if(i<len(scan.ranges)):
                        filtered_scan.ranges[i] = float('NaN')

        #print (filtered_scan.ranges)
        self.filtered_scan = filtered_scan
        self.got_scan = True

    def agentsCB(self,msg):
        for agent in msg.agents:
            # if agent.type == 0:
            #     continue
            for segment in agent.segments:
                if(segment.type == self.segment_type):
                    if(len(self.agents)<agent.track_id):
                        self.agents.append([segment.pose.pose,agent.type])
                    else:
                        self.agents[agent.track_id-1] = [segment.pose.pose,agent.type]

    def publishScan(self, event):
        if(self.got_scan):
            self.filtered_scan.header.stamp = rospy.Time.now()
            self.laser_pub.publish(self.filtered_scan)


if __name__ == '__main__':
    sim = sys.argv[1]
    if(len(sys.argv)<5):
        ns = ""
    else:
        ns = sys.argv[2]
    hfilter = AgentFilter(ns = ns, sim=sim)
