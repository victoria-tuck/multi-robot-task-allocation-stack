#!/usr/bin/env python3
"""
Created on November 1, 2018

@author: Pepper robotics

Purpose: Tilt head for navigation obstacle avoidance. 

Modified by Juan D. Hernandez Vega (juandhv@rice.edu)
"""

# Copyright (c) 2015 Pepper Robotics Inc.
# Copyright (c) 2013-2014 Unbounded Robotics Inc.
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Unbounded Robotics Inc. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNBOUNDED ROBOTICS INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from threading import Lock

import rospy
import actionlib

from tf.listener import TransformListener
from tf.transformations import quaternion_matrix

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist


class NavHeadController:
    """
    Navigation Controller class
    """

    def __init__(self):
        """
        Constructor
        """

        # =======================================================================
        # Initial values
        # =======================================================================
        self.controller_active_ = False

        # pose and lock
        self.last_time_cmd_vel_ = None
        self.x_ = 1.0
        self.y_ = 0.0
        self.mutex_ = Lock()

        self.tf_listener_ = TransformListener()

        self.head_controller_client_ = actionlib.SimpleActionClient(
            "head_controller/point_head", PointHeadAction
        )
        self.head_controller_client_.wait_for_server()

        # self.plan_sub = rospy.Subscriber("move_base/TrajectoryPlannerROS/local_plan", Path, self.planCallback)
        # self.stat_sub = rospy.Subscriber("move_base/status", GoalStatusArray, self.statCallback)
        self.cmd_vel_sub_ = rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)

        # =======================================================================
        # Timer
        # =======================================================================
        rospy.Timer(rospy.Duration(1.0), self.checkStatus)
        return

    def checkStatus(self, event):
        """
        Check controller status (active)
        """
        if self.last_time_cmd_vel_ == None:
            self.controller_active_ = False
        else:
            print "time: ", (rospy.Time.now() - self.last_time_cmd_vel_).to_sec()
            if (rospy.Time.now() - self.last_time_cmd_vel_).to_sec() < 1.0:
                self.controller_active_ = True
            else:
                self.controller_active_ = False
        return

    def cmdVelCallback(self, cmd_vel_msg):
        """
        Callback to base velocity commands
        """
        self.last_time_cmd_vel_ = rospy.Time.now()
        return

    def statCallback(self, msg):
        """ """
        goal = False
        for status in msg.status_list:
            if status.status == GoalStatus.ACTIVE:
                goal = True
                break
        self.controller_active_ = goal

    def planCallback(self, msg):
        # get the goal
        pose_stamped = msg.poses[-1]
        pose = pose_stamped.pose

        # look ahead one meter
        R = quaternion_matrix(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        point = [1, 0, 0, 1]
        M = R * point

        p = PointStamped()
        p.header.frame_id = pose_stamped.header.frame_id
        p.header.stamp = rospy.Time(0)
        p.point.x += pose_stamped.pose.position.x + M[0, 0]
        p.point.y += pose_stamped.pose.position.y + M[1, 0]
        p.point.z += pose_stamped.pose.position.z + M[2, 0]

        # transform to base_link
        p = self.tf_listener_.transformPoint("base_link", p)

        # update
        with self.mutex_:
            if p.point.x < 0.65:
                self.x_ = 0.65
            else:
                self.x_ = p.point.x
            if p.point.y > 0.5:
                self.y_ = 0.5
            elif p.point.y < -0.5:
                self.y_ = -0.5
            else:
                self.y_ = p.point.y

    def loop(self):
        """
        Controller loop
        """
        while not rospy.is_shutdown():
            if self.controller_active_:
                goal = PointHeadGoal()
                goal.target.header.stamp = rospy.Time.now()
                goal.target.header.frame_id = "base_link"
                with self.mutex_:
                    goal.target.point.x = self.x_
                    goal.target.point.y = self.y_
                    self.x_ = 1.0
                    self.y_ = 0.0
                goal.target.point.z = 0.2
                goal.min_duration = rospy.Duration(1.0)

                self.head_controller_client_.send_goal(goal)
                self.head_controller_client_.wait_for_result()

                with self.mutex_:
                    goal.target.point.x = self.x_
                    goal.target.point.y = self.y_
                    self.x_ = 1.0
                    self.y_ = 0.0
                goal.target.point.z = 0.75

                self.head_controller_client_.send_goal(goal)
                self.head_controller_client_.wait_for_result()
            else:
                rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node("tilt_head_node")
    h = NavHeadController()
    h.loop()
