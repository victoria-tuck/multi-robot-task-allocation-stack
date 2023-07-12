#!/usr/bin/env python3
"""
Created on November 1, 2018

@author: juandhv (Juan David Hernandez Vega, juandhv@rice.edu)

Purpose: Alternative esc base controller 
"""

# ROS imports
import roslib

roslib.load_manifest("esc_move_base_control")
import rospy
import tf
import tf2_ros
import math
import numpy

# ROS messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from esc_move_base_msgs.msg import Path2D

# Numpy
import numpy as np


class Controller(object):
    """
    Controller class
    """

    def __init__(self):
        """
        Constructor
        """

        # =======================================================================
        # Initial values
        # =======================================================================
        self.current_position_ = np.zeros(2)
        self.current_orientation_ = 0.0

        self.desired_position_ = np.zeros(2)
        self.desired_orientation_ = 0.0

        self.solution_path_wps_ = []

        # =======================================================================
        # Path trimmer variables
        # =======================================================================
        self.yaw = 0

        # =======================================================================
        # TF listener
        # =======================================================================
        self.tf_listener_ = tf.TransformListener()
        rospy.sleep(1.0)

        # =======================================================================
        # Get parameters
        # =======================================================================
        self.max_vel_ = rospy.get_param("~max_vel", 0.1)
        self.min_vel_ = rospy.get_param("~min_vel", 0.05)
        self.max_turn_rate_ = rospy.get_param("~max_turn_rate", 0.5)
        self.min_turn_rate_ = rospy.get_param("~min_turn_rate", 0.5)
        self.drift_turning_vel_ = rospy.get_param("~drift_turning_vel", 0.0)
        self.controller_hz_ = rospy.get_param("~controller_hz", 100)
        self.odometry_topic_ = rospy.get_param("~odometry_topic", "odometry_topic")
        self.control_path_topic_ = rospy.get_param(
            "~control_path_topic", "control_path_topic"
        )
        self.control_output_topic_ = rospy.get_param(
            "~control_output_topic", "control_output_topic"
        )
        self.control_active_topic_ = rospy.get_param(
            "~control_active_topic", "control_active_topic"
        )
        self.xy_goal_tolerance_ = rospy.get_param(
            "/esc_move_base_planner/xy_goal_tolerance", 0.2
        )
        self.yaw_goal_tolerance_ = rospy.get_param(
            "/esc_move_base_planner/yaw_goal_tolerance", 0.2
        )

        # =======================================================================
        # Subscribers
        # =======================================================================
        # Navigation data (feedback)
        self.odometry_sub_ = rospy.Subscriber(
            self.odometry_topic_, Odometry, self.odomCallback, queue_size=1
        )
        self.control_path_sub_ = rospy.Subscriber(
            self.control_path_topic_,
            Path2D,
            self.receiveControlPathCallback,
            queue_size=1,
        )

        # =======================================================================
        # Publishers
        # =======================================================================
        self.control_output_pub_ = rospy.Publisher(
            self.control_output_topic_, Twist, queue_size=1
        )
        self.control_active_pub_ = rospy.Publisher(
            self.control_active_topic_, Bool, queue_size=1
        )

    def odomCallback(self, odometry_msg):
        """
        Callback to robot's odometry data
        """
        self.current_position_[0] = odometry_msg.pose.pose.position.x
        self.current_position_[1] = odometry_msg.pose.pose.position.y
        (r, p, self.yaw) = tf.transformations.euler_from_quaternion(
            [
                odometry_msg.pose.pose.orientation.x,
                odometry_msg.pose.pose.orientation.y,
                odometry_msg.pose.pose.orientation.z,
                odometry_msg.pose.pose.orientation.w,
            ]
        )
        self.current_orientation_ = wrapAngle(self.yaw)
        return

    def receiveControlPathCallback(self, path_2d_msg):
        """Callback to receive path (list of waypoints)"""
        self.solution_path_wps_ = []

        waypoint_distances = np.array([])
        min_list_index = []
        actual_next_waypoint_index = None

        waypoints_list = path_2d_msg.waypoints

        if len(waypoints_list) <= 2:
            self.solution_path_wps_.append(
                [waypoints_list[-1].x, waypoints_list[-1].y, waypoints_list[-1].theta]
            )
            return

        # print(waypoints_list)
        for waypoint in waypoints_list:
            waypoint_distances = np.append(
                waypoint_distances,
                [
                    euclidian_distance(
                        self.current_position_[0],
                        self.current_position_[1],
                        waypoint.x,
                        waypoint.y,
                    )
                ],
            )
        # print(len(waypoint_distances))
        for i in range(0, 3):
            min_list_index.append(
                numpy.where(waypoint_distances == numpy.amin(waypoint_distances))[0][0]
            )
            waypoint_distances[min_list_index[i]] = float("inf")
        print(min_list_index)
        current_waypoint_angle = float("inf")
        for i in min_list_index:
            new_waypoint_angle = self.robotAngleToPoint(
                path_2d_msg.waypoints[i].x, path_2d_msg.waypoints[i].y
            )
            if new_waypoint_angle < current_waypoint_angle:
                actual_next_waypoint_index = i
                current_waypoint_angle = new_waypoint_angle

        if actual_next_waypoint_index is not None:
            waypoints_list = waypoints_list[actual_next_waypoint_index + 1 :]

        for i in range(1, len(waypoints_list)):
            waypoint = waypoints_list[i]
            distance_to_wp = math.sqrt(
                math.pow(waypoint.x - self.current_position_[0], 2.0)
                + math.pow(waypoint.y - self.current_position_[1], 2.0)
            )
            if distance_to_wp > self.xy_goal_tolerance_ or i == (
                len(waypoints_list) - 1
            ):
                self.solution_path_wps_.append([waypoint.x, waypoint.y, waypoint.theta])
        return

    def robotAngleToPoint(self, x, y):

        tetha_robot_waypoint = np.arctan2(
            y - self.current_position_[1], x - self.current_position_[0]
        )

        if tetha_robot_waypoint < 0:
            tetha_robot_waypoint = 2 * math.pi + tetha_robot_waypoint

        robotAngle = self.yaw

        if robotAngle < 0:
            robotAngle = 2 * math.pi + robotAngle

        if tetha_robot_waypoint > (robotAngle + math.pi):
            tetha_robot_waypoint = abs(robotAngle + 2 * math.pi - tetha_robot_waypoint)
        elif robotAngle > (tetha_robot_waypoint + math.pi):
            tetha_robot_waypoint = abs(tetha_robot_waypoint + 2 * math.pi - robotAngle)
        else:
            tetha_robot_waypoint = abs(tetha_robot_waypoint - robotAngle)

        return tetha_robot_waypoint

    def controlBaseEsc(self):
        """Control loop"""
        loop_rate = rospy.Rate(self.controller_hz_)
        controller_state = 0
        while not rospy.is_shutdown():
            if len(self.solution_path_wps_) > 0:
                # print(self.solution_path_wps_)
                self.desired_position_[0] = self.solution_path_wps_[0][0]
                self.desired_position_[1] = self.solution_path_wps_[0][1]
                inc_x = self.desired_position_[0] - self.current_position_[0]
                inc_y = self.desired_position_[1] - self.current_position_[1]
                distance_to_goal = math.sqrt(
                    math.pow(inc_x, 2.0) + math.pow(inc_y, 2.0)
                )
                control_input = Twist()
                control_input.angular.x = 0.0
                control_input.angular.y = 0.0
                control_input.angular.z = 0.0
                control_input.linear.x = 0.0
                control_input.linear.y = 0.0
                control_input.linear.z = 0.0

                if distance_to_goal >= 0.4:
                    controller_state = 0
                    self.desired_orientation_ = wrapAngle(math.atan2(inc_y, inc_x))
                    yaw_error = wrapAngle(
                        self.desired_orientation_ - self.current_orientation_
                    )

                    if abs(yaw_error) > 0.4:
                        rospy.logdebug(
                            "%s: orienting towards the next waypoint: %s",
                            rospy.get_name(),
                            yaw_error,
                        )
                        control_input.angular.z = yaw_error * self.max_turn_rate_
                    else:
                        rospy.logdebug(
                            "%s: moving towards the next waypoint: %s",
                            rospy.get_name(),
                            yaw_error,
                        )
                        control_input.angular.z = yaw_error * self.max_turn_rate_

                        liner_speed = abs(distance_to_goal) * 0.5
                        if liner_speed < self.min_vel_:
                            control_input.linear.x = self.min_vel_
                        elif liner_speed > self.max_vel_:
                            control_input.linear.x = self.max_vel_
                        else:
                            control_input.linear.x = liner_speed

                    if control_input.angular.z < -self.max_turn_rate_:
                        control_input.angular.z = -self.max_turn_rate_
                    elif control_input.angular.z > self.max_turn_rate_:
                        control_input.angular.z = self.max_turn_rate_
                    self.control_output_pub_.publish(control_input)

                else:
                    if len(self.solution_path_wps_) > 1:
                        del self.solution_path_wps_[0]
                    else:
                        if (
                            distance_to_goal >= self.xy_goal_tolerance_
                            and controller_state != 2
                        ):
                            controller_state = 1
                            self.desired_orientation_ = wrapAngle(
                                math.atan2(inc_y, inc_x)
                            )
                            yaw_error = wrapAngle(
                                self.desired_orientation_ - self.current_orientation_
                            )
                            if abs(yaw_error) > 0.2 and abs(yaw_error) < 1.57:
                                rospy.logdebug(
                                    "%s: final approach: orienting towards a waypoint (fordward)",
                                    rospy.get_name(),
                                )
                                if yaw_error > 0.0:
                                    control_input.angular.z = self.min_turn_rate_
                                else:
                                    control_input.angular.z = -self.min_turn_rate_
                                control_input.linear.x = self.drift_turning_vel_
                            elif abs(yaw_error) > 1.57 and abs(yaw_error) < 2.94:
                                rospy.logdebug(
                                    "%s: final approach: orienting towards a waypoint (backward)",
                                    rospy.get_name(),
                                )
                                if yaw_error > 0.0:
                                    control_input.angular.z = -self.min_turn_rate_
                                else:
                                    control_input.angular.z = self.min_turn_rate_

                                control_input.linear.x = -self.drift_turning_vel_
                            elif abs(yaw_error) < 0.2:
                                rospy.logdebug(
                                    "%s: final approach: moving towards a waypoint (fordward)",
                                    rospy.get_name(),
                                )

                                control_input.linear.x = 0.05
                            else:
                                rospy.logdebug(
                                    "%s: final approach: moving towards a waypoint (backward)",
                                    rospy.get_name(),
                                )

                                control_input.linear.x = -0.05

                            self.control_output_pub_.publish(control_input)
                        else:
                            controller_state = 2
                            rospy.logdebug(
                                "%s: final approach, final orientation",
                                rospy.get_name(),
                            )
                            yaw_error = wrapAngle(
                                self.solution_path_wps_[0][2]
                                - self.current_orientation_
                            )

                            if abs(yaw_error) < self.yaw_goal_tolerance_:
                                if len(self.solution_path_wps_) > 0:
                                    del self.solution_path_wps_[0]
                            else:
                                control_input.angular.z = (
                                    yaw_error * self.max_turn_rate_
                                )
                                if yaw_error < 0.0:
                                    control_input.linear.x = self.drift_turning_vel_
                                    control_input.angular.z = -self.min_turn_rate_
                                else:
                                    control_input.linear.x = -self.drift_turning_vel_
                                    control_input.angular.z = self.min_turn_rate_
                                self.control_output_pub_.publish(control_input)

                # 				rospy.logdebug("%s: yaw_error: %f", rospy.get_name(), yaw_error)
                # 				rospy.logdebug("%s: distance_to_goal: %f", rospy.get_name(), distance_to_goal)
                # 				rospy.logdebug("%s: control_input.linear.x %f", rospy.get_name(), control_input.linear.x)
                # 				rospy.logdebug("%s: control_input.angular.z %f\n", rospy.get_name(), control_input.angular.z)
                self.control_active_pub_.publish(Bool(True))
            else:
                self.control_active_pub_.publish(Bool(False))
            loop_rate.sleep()
        return


def euclidian_distance(x1, y1, x2, y2):
    return np.sqrt(np.power(x2 - x1, 2) + np.power(y2 - y1, 2))


def wrapAngle(angle):
    """wrapAngle
    Calculates angles values between 0 and 2pi"""
    return angle + (2.0 * math.pi * math.floor((math.pi - angle) / (2.0 * math.pi)))


if __name__ == "__main__":
    rospy.init_node("esc_move_base_control", log_level=rospy.INFO)
    rospy.loginfo("%s: starting esc_move_base controller", rospy.get_name())

    controller = Controller()
    controller.controlBaseEsc()
    rospy.spin()
