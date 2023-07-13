#!/usr/bin/env python
# Brief: Publish TF frames for emulating a fake robot
# Author: Phani Teja Singamaneni
import rospy
import tf2_msgs.msg
import geometry_msgs.msg #PoseArray
import random
import math
import yaml
import os
import numpy as np
from nav_msgs.msg import MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from transformations import quaternion_from_euler
import PySimpleGUI as sg
from map_server.srv import LoadMap
# random.seed(3407)

class FakeTFBroadcaster(object):

    def __init__(self, map_path, map_name):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_passage = rospy.Publisher("/passage_detection", Marker, queue_size=1)
        rospy.Subscriber("map_metadata", MapMetaData, self.update_map_limits)
        rospy.Subscriber("invisible_humans", geometry_msgs.msg.PoseArray, self.invHumansCB)
        with open(map_path+map_name+'.yaml','r') as file:
          self.map = yaml.safe_load(file)
        with open(map_path+'passages.yaml','r') as file:
          dat = yaml.safe_load(file)
          self.psg_info = dat[map_name]

        self.x_lim = 0
        self.y_lim = 0
        self.inv_humans_info = []
        self.inv_humans = geometry_msgs.msg.PoseArray()
        self.last_time = rospy.Time.now()

        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.frame_id = "map"
        self.t.header.stamp = rospy.Time.now()
        self.t.child_frame_id = "base_footprint"
        self.t.transform.translation.x = 5.5
        self.t.transform.translation.y = 5.5
        self.t.transform.translation.z = 0.0

        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = 0.0
        self.t.transform.rotation.w = 1.0

    def update_pose(self, center = None, rotate = None, passage = None):
      if not center and not rotate:
        c, r, theta, index = self.pick_random_pose(passage)
        self.t.transform.translation.x = c[0]+r*math.cos(theta)
        self.t.transform.translation.y = c[1]+r*math.sin(theta)
      elif center:
        self.t.transform.translation.x = self.x_lim/2
        self.t.transform.translation.y = self.y_lim/2

      # print(c[0]+r*math.cos(theta), c[1]+r*math.sin(theta))
      if passage:
        q = quaternion_from_euler(0,0,math.radians(int(self.psg_info['ang'][index])))
      else:
        q = quaternion_from_euler(0,0,random.uniform(-math.pi, math.pi))
      # print(q)
      self.t.transform.rotation.x = q[0]
      self.t.transform.rotation.y = q[1]
      self.t.transform.rotation.z = q[2]
      self.t.transform.rotation.w = q[3]

    def pick_random_pose(self, passage = None):
      index = -1
      if not passage:
        n = len(self.map['radii']) # Not radius but diameter
        idx = random.randrange(0,n)
        center = self.map['centers'][idx]
        radius = math.sqrt(random.random())*(self.map['radii'][idx]/2) #Divided by 2 because diameter
        theta = random.uniform(0, 2*math.pi)
        index = idx
      else:
        n = len(self.psg_info['idx'])
        i = random.randrange(0,n)
        center = self.map['centers'][self.psg_info['idx'][i]]
        radius = math.sqrt(random.random())*(self.map['radii'][self.psg_info['idx'][i]]/2) #Divided by 2 because diameter
        # theta = math.radians(self.psg_info['ang'][i])
        theta = random.uniform(0, 2*math.pi)
        index = i

      return center, radius, theta, index

    def update_map_limits(self, msg):
      self.x_lim = msg.width* msg.resolution
      self.y_lim = msg.height* msg.resolution

    def update_pose_within_limits(self):
      self.t.transform.translation.x = random.uniform(0, self.x_lim)
      self.t.transform.translation.y = random.uniform(0, self.y_lim)
      q = quaternion_from_euler(0,0,random.uniform(-math.pi, math.pi))
      self.t.transform.rotation.x = q[0]
      self.t.transform.rotation.y = q[1]
      self.t.transform.rotation.z = q[2]
      self.t.transform.rotation.w = q[3]

    def invHumansCB(self, msg):
      self.inv_humans_info = []
      x = self.t.transform.translation.x
      y = self.t.transform.translation.y
      self.inv_humans = msg
      i = 0
      for pose in msg.poses:
        dist = np.linalg.norm([pose.position.x-x,pose.position.y-y])
        self.inv_humans_info.append([dist,i])
        i = i+1
      # print("prev",self.inv_humans_info)
      self.inv_humans_info.sort(key = lambda x: x[0])
      self.last_time = rospy.Time.now()
      self.check_passages()
      # print("new",self.inv_humans_info)

    def check_passages(self):
      # print(len(self.inv_humans_info))
      found = -1
      dist1 = -999
      dist2 = -777
      if len(self.inv_humans_info)>0:
        # print(self.inv_humans.poses[self.inv_humans_info[0][1]].orientation.z)
        dist1 = self.inv_humans_info[0][0]
        if len(self.inv_humans_info)>1:
          i1= self.inv_humans_info[0][1]
          i2= self.inv_humans_info[1][1]
          sep_dist = np.linalg.norm([self.inv_humans.poses[i1].position.x-self.inv_humans.poses[i2].position.x,
                                    self.inv_humans.poses[i1].position.y-self.inv_humans.poses[i2].position.y])
          if self.inv_humans_info[0][0] < 2.0 and abs(self.inv_humans_info[0][0] - self.inv_humans_info[1][0]) \
            and sep_dist <3.0:
            detect_pose = [(self.inv_humans.poses[i1].position.x + self.inv_humans.poses[i2].position.x)/2,
                          (self.inv_humans.poses[i1].position.y + self.inv_humans.poses[i2].position.y)/2]
            if self.inv_humans.poses[i1].position.z > 1.33:
              # print("It's a door/passage")
              found = 0
            else:
              found = 1
              # print("It's a  pillar")
        dist2 = self.inv_humans.poses[self.inv_humans_info[0][1]].orientation.z
        if found == -1 and self.inv_humans_info[0][0] < 2.0 and 0.5 <= self.inv_humans.poses[self.inv_humans_info[0][1]].orientation.z <=7.0:
          # print("It's a wall passage")
          found = 2
          detect_pose = [self.inv_humans.poses[self.inv_humans_info[0][1]].position.x, self.inv_humans.poses[self.inv_humans_info[0][1]].position.y]
      if found != -1:
        marker = Marker()
        m_id = 0
        marker.header.frame_id = "map"
        marker.id = m_id
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = detect_pose[0]
        marker.pose.position.y = detect_pose[1]
        marker.pose.position.z = 0.0
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.2)
        # marker.color.r = 1.0
        if found == 0:
          marker.text =  "Door/Passage " #+ str(dist1)+ " "+ str(dist2)
        elif found == 1:
          marker.text = "Pillar "#+ str(dist1)+ " "+ str(dist2)
        elif found == 2:
          marker.text = "Wall on the other side "#+ str(dist1)+ " "+ str(dist2)
        else:
          marker.text = "No info "+ str(dist1)+ " "+ str(dist2)
        self.pub_passage.publish(marker)
      else:
        marker = Marker()
        m_id = 0
        marker.header.frame_id = "map"
        marker.id = m_id
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 2
        marker.pose.position.y = 2
        marker.pose.position.z = 0.0
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.2)
        marker.text = "Dist = "+str(dist1) + " "+ str(dist2)
        self.pub_passage.publish(marker)


def generate_random_map(path):
    idx = random.randint(0,3)
    if idx == 0:
      os.system(path+"/mazegenerator/src/mazegen -t 1 -s 2 -m 2") #Honeycomb
    elif idx == 1:
      os.system(path+"/mazegenerator/src/mazegen -t 1 -s 3 -m 1") #Hexagonal
    elif idx == 2:
      os.system(path+"/mazegenerator/src/mazegen -t 1 -w 5 -h 5 -m 0") #Rectangular
    else:
      os.system(path+"/mazegenerator/src/mazegen -t 1 -s 3 -m 4") #Hexagonal

def update_map(path):
    rospy.wait_for_service('change_map')
    try:
        change_map_ = rospy.ServiceProxy('change_map', LoadMap)
        resp = change_map_(path+"/maze.yaml")
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    map_name = 'bremen'
    passage_detect = False

    sg.theme('DarkAmber')   # Add a touch of color
    # All the stuff inside your window.
    if not passage_detect:
      layout = [  [sg.Text('False Positives'), sg.InputText(), sg.Text('Overlapping'), sg.InputText()],
                [sg.Text('Total'), sg.InputText()],
                [sg.Button('AddData'),sg.Text('0', size=(50,1), key='-mytext-')],
                [sg.Button('NextPose'), sg.Button('CenterPose'), sg.Button('Rotate')],
                [sg.Button('NextMap'), sg.Button('Save'), sg.Button('Exit')]
             ]
    else:
      layout = [[sg.Text('True Detection'), sg.InputText(), sg.Text('False Detection'), sg.InputText()],
                [sg.Text('No Detect (Range Limit)'), sg.InputText(), sg.Text('No Detect (Within Range)'), sg.InputText()],
                [sg.Button('AddData'),sg.Text('0', size=(50,1), key='-mytext-')],
                [sg.Button('NextPose'), sg.Button('CenterPose'), sg.Button('Rotate')],
                [sg.Button('NextMap'), sg.Button('Save'), sg.Button('Exit')]
             ]
    # Create the Window
    window = sg.Window('Get Data', layout)

    rospy.init_node('fake_robot_broadcaster')
    path = os.path.abspath(os.path.dirname(__file__))
    tfb = FakeTFBroadcaster(os.path.join(path, '../maps/areas/'), map_name)
    robot_marker_pub = rospy.Publisher("/robot_marker", MarkerArray, queue_size=1)
    robot_marker = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "map"
    marker.id = 0
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.pose.orientation = tfb.t.transform.rotation
    marker.pose.position.x = tfb.t.transform.translation.x
    marker.pose.position.y = tfb.t.transform.translation.y
    marker.pose.position.z = 0.0
    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = 0.4
    marker.color.a = 1.0
    marker.color.g = 1.0
    arrow = Marker()
    arrow.header.frame_id = "map"
    arrow.id = 1
    arrow.type = 0
    arrow.action = arrow.ADD
    arrow.pose.orientation=tfb.t.transform.rotation
    arrow.pose.position.x = tfb.t.transform.translation.x
    arrow.pose.position.y = tfb.t.transform.translation.y
    arrow.pose.position.z = 0.0
    arrow.scale.x = 0.6
    arrow.scale.y = 0.1
    arrow.scale.z = 0.1
    arrow.color.a = 1.0

    map_updated = False
    data = dict()
    if not passage_detect:
      data['false_postives'] = []
      data['overlap'] = []
      data['total'] = []
    else:
      data['t_detect'] = []
      data['f_detect'] = []
      data['no_detect_lim'] = []
      data['no_detect'] = []
    while not rospy.is_shutdown():
      tfb.t.header.stamp = rospy.Time.now()
      tfm = tf2_msgs.msg.TFMessage([tfb.t])
      tfb.pub_tf.publish(tfm)
      robot_marker = MarkerArray()
      marker.pose.orientation = tfb.t.transform.rotation
      marker.pose.position.x = tfb.t.transform.translation.x
      marker.pose.position.y = tfb.t.transform.translation.y
      arrow.pose.orientation=tfb.t.transform.rotation
      arrow.pose.position.x = tfb.t.transform.translation.x
      arrow.pose.position.y = tfb.t.transform.translation.y
      robot_marker.markers.append(marker)
      robot_marker.markers.append(arrow)
      robot_marker_pub.publish(robot_marker)

      event, values = window.read(50)
      if event == sg.WIN_CLOSED or event == 'Exit': # if user closes window or clicks cancel
          break
      if event == 'AddData':
        if not passage_detect:
          data['false_postives'].append(float(values[0]))
          data['overlap'].append(float(values[1]))
          data['total'].append(float(values[2]))
          window['-mytext-'].update(str(len(data['total'])))
        else:
          data['t_detect'].append(float(values[0]))
          data['f_detect'].append(float(values[1]))
          data['no_detect_lim'].append(float(values[2]))
          data['no_detect'].append(float(values[3]))
          window['-mytext-'].update(str(len(data['t_detect'])))
      if event == 'NextPose':
        if not map_updated:
          tfb.update_pose(passage=passage_detect)
        else:
          tfb.update_pose_within_limits()

      if event == 'CenterPose':
        tfb.update_pose(center = True)

      if event == 'Rotate':
        tfb.update_pose(rotate = True)

      if event == 'NextMap':
        generate_random_map(path)
        update_map(path)
        map_updated = True

      if event == 'Save':
        with open('data.yaml', 'w') as file:
            yaml.dump(data, file, default_flow_style=False)

      if((tfb.last_time - rospy.Time.now()).to_sec()) > 2.0:
        tfb.inv_humans_info = []
