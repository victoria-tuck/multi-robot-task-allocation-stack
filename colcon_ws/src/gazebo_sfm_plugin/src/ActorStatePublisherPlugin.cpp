// Copyright 2013 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ignition/math/Rand.hh>
#include "gazebo_ros/node.hpp"
#include "gazebo_ros/utils.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <gazebo_sfm_plugin/ActorStatePublisherPlugin.hpp>

#ifdef NO_ERROR
// NO_ERROR is a macro defined in Windows that's used as an enum in tf2
#undef NO_ERROR
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <string>
#include <memory>

namespace gazebo_plugins
{

class ActorStatePublisherPluginPrivate
{
public:
  /// Callback to be called at every simulation iteration
  /// \param[in] info Updated simulation info
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// The link being traked.
  gazebo::physics::LinkPtr link_{nullptr};

  /// The body of the frame to display pose, twist
  gazebo::physics::LinkPtr reference_link_{nullptr};

  /// Pointer to ros node
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  // Actor point
  gazebo::physics::ActorPtr actor;
  gazebo::physics::WorldPtr world;
  ignition::math::Vector3d actorPos_prev;
  bool pos_init = false;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_{nullptr};
  // rclcpp::Subscriber<std_msgs::msg::Bool>::SharedPtr sub_{nullptr};
  // bool run_sim = false;

  /// Odom topic name
  std::string topic_name_{"odom"};

  /// Frame transform name, should match name of reference link, or be world.
  std::string frame_name_{"world"};

  /// Constant xyz and rpy offsets
  ignition::math::Pose3d offset_;

  /// Keep track of the last update time.
  gazebo::common::Time last_time_;

  /// Publish rate in Hz.
  double update_rate_{0.0};

  /// Gaussian noise
  double gaussian_noise_;

  /// Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_{nullptr};
};

ActorStatePublisherPlugin::ActorStatePublisherPlugin()
: impl_(std::make_unique<ActorStatePublisherPluginPrivate>())
{
}

ActorStatePublisherPlugin::~ActorStatePublisherPlugin()
{
}

// Load the controller
void ActorStatePublisherPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Configure the plugin from the SDF file
//   impl_->ros_node_ = gazebo_ros::Node::Get(sdf, model);
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  impl_->actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
  impl_->world = impl_->actor->GetWorld();
//   std::vector<physics::LinkPtr> links = model->GetLinks();
  ignition::math::Vector3d actorPos = impl_->actor->WorldPose().Pos();
//   this->sdf = _sdf;
 
  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  if (!sdf->HasElement("update_rate")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "p3d plugin missing <update_rate>, defaults to 0.0"
      " (as fast as possible)");
  } else {
    impl_->update_rate_ = sdf->GetElement("update_rate")->Get<double>();
  }
  std::cout << " ********  ActorStatePublisherPlugin started for " << impl_->actor->GetName() << " at initial location: " << actorPos << " ******** " << std::endl;

  impl_->pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
    impl_->topic_name_, qos.get_publisher_qos(
      impl_->topic_name_, rclcpp::SensorDataQoS().reliable()));
  // impl_->sub_ = impl_->create_subscriber<std_msgs::msg::bool>
  impl_->topic_name_ = impl_->pub_->get_topic_name();
  RCLCPP_DEBUG(
    impl_->ros_node_->get_logger(), "Publishing on topic [%s]", impl_->topic_name_.c_str());

  impl_->last_time_ = model->GetWorld()->SimTime();

  // Listen to the update event. This event is broadcast every simulation iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ActorStatePublisherPluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

// Update the controller
void ActorStatePublisherPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{

ignition::math::Vector3d actorPos = actor->WorldPose().Pos();
// std::cout << " PLUGIN ********************************** my loc: " << actorPos << std::endl;
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosP3DPrivate::OnUpdate");
#endif
  gazebo::common::Time current_time = info.simTime;

  if (current_time < last_time_) {
    RCLCPP_WARN(ros_node_->get_logger(), "Negative update time difference detected.");
    last_time_ = current_time;
  }

  // Rate control
  if (update_rate_ > 0 &&
    (current_time - last_time_).Double() < (1.0 / update_rate_))
  {
    return;
  }
  // If we don't have any subscribers, don't bother composing and sending the message
  if (ros_node_->count_subscribers(topic_name_) == 0) {
    return;
  }
  // Differentiate to get accelerations
  double tmp_dt = current_time.Double() - last_time_.Double();
  if (tmp_dt == 0) {
    return;
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  nav_msgs::msg::Odometry pose_msg;

  // Copy data into pose message
  pose_msg.header.frame_id = frame_name_;
  pose_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  pose_msg.child_frame_id = actor->GetName();//link_->GetName();

  // Get inertial rates
  ignition::math::Vector3d vpos = actor->WorldLinearVel(); //link_->WorldLinearVel();
  ignition::math::Vector3d veul = actor->WorldAngularVel(); //link_->WorldAngularVel();

  // Get pose/orientation
  auto pose = actor->WorldPose();//.Pos();//link_->WorldPose();

  // Fill out messages
  pose_msg.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
   // Differentiate position
  if (pos_init){
    vpos = (pose.Pos() - actorPos_prev)/tmp_dt;
  }
  // std::cout << "Pose " << pose.Pos() << " Vel: " << vpos << std::endl;
  // ignition::math::Vector3d actorPos2 = pose.Pos();
//   std::cout << " PLUGIN ********************************** my loc: " << actorPos << std::endl;
  ignition::math::Vector3d rpy = this->actor->WorldPose().Rot().Euler();
  double yaw = rpy.Z()-1.57079;
  pose_msg.pose.pose.orientation.x = 0.0;
  pose_msg.pose.pose.orientation.y = 0.0;
  pose_msg.pose.pose.orientation.z = sin(yaw/2.0);
  pose_msg.pose.pose.orientation.w = cos(yaw/2.0);

//   pose_msg.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  pose_msg.twist.twist.linear.x = vpos.X() ;
  pose_msg.twist.twist.linear.y = vpos.Y() ;
  pose_msg.twist.twist.linear.z = vpos.Z() ;
  pose_msg.twist.twist.angular.x = veul.X() ;
  pose_msg.twist.twist.angular.y = veul.Y() ;
  pose_msg.twist.twist.angular.z = veul.Z() ;
    // std::cout << " PLUGIN ********************************** my loc: " << pose_msg.pose.pose.position.x << std::endl;
  // Fill in covariance matrix
  /// @TODO: let user set separate linear and angular covariance values
  double gn2 = 0;//gaussian_noise_ * gaussian_noise_;
  pose_msg.pose.covariance[0] = gn2;
  pose_msg.pose.covariance[7] = gn2;
  pose_msg.pose.covariance[14] = gn2;
  pose_msg.pose.covariance[21] = gn2;
  pose_msg.pose.covariance[28] = gn2;
  pose_msg.pose.covariance[35] = gn2;
  pose_msg.twist.covariance[0] = gn2;
  pose_msg.twist.covariance[7] = gn2;
  pose_msg.twist.covariance[14] = gn2;
  pose_msg.twist.covariance[21] = gn2;
  pose_msg.twist.covariance[28] = gn2;
  pose_msg.twist.covariance[35] = gn2;
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish");
#endif
  // Publish to ROS
  pub_->publish(pose_msg);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  // Save last time stamp
  last_time_ = current_time;
  actorPos_prev = pose.Pos();
  pos_init = true;
}



GZ_REGISTER_MODEL_PLUGIN(ActorStatePublisherPlugin) //GazeboRosP3DTest

}  // namespace gazebo_plugins