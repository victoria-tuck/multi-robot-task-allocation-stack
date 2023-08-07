/**
* Copyright 2014- Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
*/

#ifndef PEDSIM_SENSOR_H
#define PEDSIM_SENSOR_H

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace pedsim_ros {

struct FoV {
  double origin_x;
  double origin_y;

  FoV() = default;
  FoV(const double x, const double y) : origin_x{x}, origin_y{y} {}
  virtual bool inside(const double x, const double y) const = 0;
  virtual void updateViewpoint(const double new_x, const double new_y) = 0;
};

using FoVPtr = std::shared_ptr<FoV>;

struct CircularFov : FoV {
  double radius;

  CircularFov(const double cx, const double cy, const double r)
      : FoV(cx, cy), radius{r} {}

  bool inside(const double x, const double y) const override {
    return std::hypot(x - origin_x, y - origin_y) < radius;
  }
  void updateViewpoint(const double new_x, const double new_y) override {
    origin_x = new_x;
    origin_y = new_y;
  }
};

/// \brief A sensor interface.
class PedsimSensor: public rclcpp::Node {

 public:
   PedsimSensor(std::string node_name)//, const double rate, const FoVPtr& fov)
      : Node(node_name)//, rate_{rate}
       {

    // if (fov == nullptr) {
    //   ROS_FATAL_STREAM("Sensor FoV cannot be null.");
    // }

    this->declare_parameter<double>("pose_initial_x", 0.0);
    this->declare_parameter<double>("pose_initial_y", 0.0);
    this->declare_parameter<double>("fov_range", 15.);
    this->get_parameter("pose_initial_x",init_x);
    this->get_parameter("pose_initial_y",init_y);
    this->get_parameter("fov_range",fov_range);

    pedsim_ros::FoVPtr circle_fov;
    circle_fov.reset(new pedsim_ros::CircularFov(init_x, init_y, fov_range));

    double sensor_rate = 0.0;
    this->declare_parameter<double>("rate", 25.0);
    this->get_parameter("rate", sensor_rate);
    rate_ = sensor_rate;

    fov_ = circle_fov;
    // Set up robot odometry subscriber.
    sub_robot_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/pedsim_simulator/robot_position", 1,
                                    std::bind(&PedsimSensor::callbackRobotOdom, this, _1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // transform_listener_ = boost::make_shared<tf::TransformListener>();
  }
  virtual ~PedsimSensor() = default;
  virtual void broadcast() = 0;
  void callbackRobotOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_odom_ = *msg;
    // update sensor anchor.
    fov_->updateViewpoint(robot_odom_.pose.pose.position.x,
                          robot_odom_.pose.pose.position.y);
  }

 protected:
  double rate_ = 25.;
  double init_x = 0.0; 
  double init_y = 0.0;
  double fov_range = 0.0;
  FoVPtr fov_ = nullptr;
  nav_msgs::msg::Odometry robot_odom_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_signals_local_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_signals_global_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_robot_odom_;

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

// tf2::Pose transformPoint(const tf::StampedTransform& T_r_o,
//                         const tf::Vector3& point) {
//   tf2::Pose source;
//   source.setOrigin(point);

//   tf2::Matrix3x3 identity;
//   identity.setIdentity();
//   source.setBasis(identity);

//   return T_r_o * source;
// }

}  // namespace pedsim_ros

#endif
