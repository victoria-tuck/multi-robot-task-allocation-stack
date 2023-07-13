/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 LAAS/CNRS
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
*********************************************************************/
#ifndef MAP_SCANNER_H
#define MAP_SCANNER_H
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/utils.h>

namespace invisible_humans_detection{
class MapScanner
{
public:
  MapScanner();
  ~MapScanner();
  void initialize();
private:
  void publishScan(const ros::TimerEvent& event);
  void getMap(const nav_msgs::OccupancyGrid &grid);
  bool worldToMap(double wx, double wy, int& mx, int& my) const;
  unsigned int getIndex(unsigned int mx, unsigned int my) const;

  ros::Timer get_robot_pose;
  geometry_msgs::PoseStamped robot_pose_;
  tf2_ros::Buffer tf_;
  ros::Subscriber map_sub_;
  ros::Publisher scan_pub_;
  nav_msgs::OccupancyGrid map_;
  sensor_msgs::LaserScan map_scan_;
  int samples, size_x_, size_y_;
  double origin_x_, origin_y_, resolution_;

};

}// namespace invisible_humans_detection

 #endif  // MAP_SCANNER_H
