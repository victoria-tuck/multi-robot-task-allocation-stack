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
 #include <invisible_humans_detection/map_scanner.h>
 //#include <angles/angles.h>


namespace invisible_humans_detection
{
  MapScanner::MapScanner(){
    initialize();
  }

  MapScanner::~MapScanner(){}

  void MapScanner::initialize(){
    ros::NodeHandle nh("~/");
    tf2_ros::TransformListener tfListener(tf_);
    get_robot_pose = nh.createTimer(ros::Duration(0.01), &MapScanner::publishScan, this);
    scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("map_scan",1);
    map_sub_ = nh.subscribe("/map", 1, &MapScanner::getMap, this);
    ros::spin();

  }

  void MapScanner::publishScan(const ros::TimerEvent& event){

    // Get Robot Pose
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tf_.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    robot_pose_.header = transformStamped.header;
    robot_pose_.pose.position.x = transformStamped.transform.translation.x;
    robot_pose_.pose.position.y = transformStamped.transform.translation.y;
    robot_pose_.pose.position.z = transformStamped.transform.translation.z;
    robot_pose_.pose.orientation = transformStamped.transform.rotation;
    auto theta = tf2::getYaw(robot_pose_.pose.orientation);

    samples = 1081;
    std::vector<float> vect(samples, 0.0);
    map_scan_.header.frame_id = "base_footprint";
    map_scan_.header.stamp = ros::Time::now();
    map_scan_.angle_min = -2.358;
    map_scan_.angle_max = 2.358;
    map_scan_.scan_time = 0.0;
    map_scan_.time_increment = 0.0;
    map_scan_.angle_increment = (map_scan_.angle_max-map_scan_.angle_min)/samples;
    map_scan_.range_min = 0.05;
    map_scan_.range_max = 7.0;
    map_scan_.ranges = vect;

    int scan_resolution = 700;

    auto ang = map_scan_.angle_min;
    double increment_ = map_scan_.range_max/scan_resolution;

    Eigen::Vector2d robot_vec{cos(theta),sin(theta)};

    for(int i=0;i<samples;i++){
      if(map_.data.empty()){
      continue;
      }
      double ray_ = map_scan_.range_min;
      Eigen::Vector2d r_dir{robot_vec.x()*cos(ang)-robot_vec.y()*sin(ang),
                            +robot_vec.x()*sin(ang)+robot_vec.y()*cos(ang)};

      map_scan_.ranges[i] = map_scan_.range_max;

      for(int j=0;j<scan_resolution;j++){
        auto rx = robot_pose_.pose.position.x + ray_*r_dir.x();
        auto ry = robot_pose_.pose.position.y + ray_*r_dir.y();
        int mx, my;

        if(!worldToMap(rx,ry,mx,my)){
          continue;
        }
        auto idx = getIndex(mx,my);

        if(int(map_.data[idx]) == 100){
          map_scan_.ranges[i] = ray_;
          break;
        }
        ray_ += increment_;
      }
      ang += map_scan_.angle_increment;
    }
    scan_pub_.publish(map_scan_);
    // for(int i=0;i<1081;i++)
      // std::cout << "map_scan_.ranges " <<map_scan_<< '\n';


  }

  void MapScanner::getMap(const nav_msgs::OccupancyGrid &grid){
    map_ = grid;
    // std::cout << map_.data.size() << "\n";
    origin_x_ = map_.info.origin.position.x;
    origin_y_ = map_.info.origin.position.y;
    resolution_ = map_.info.resolution;
    size_x_ = map_.info.width;
    size_y_ = map_.info.height;
  }

  bool MapScanner::worldToMap(double wx, double wy, int& mx, int& my) const{
    if(wx < origin_x_ || wy < origin_y_)
      return false;

    mx = (int) ((wx - origin_x_) / resolution_);
    my = (int) ((wy - origin_y_) / resolution_);

    if(mx < size_x_ && my < size_y_)
      return true;

    return false;
  }

  unsigned int MapScanner::getIndex(unsigned int mx, unsigned int my) const{
   return my * size_x_ + mx;
 }

}
