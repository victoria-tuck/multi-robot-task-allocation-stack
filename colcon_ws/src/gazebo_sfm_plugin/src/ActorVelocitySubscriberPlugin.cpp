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
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <gazebo_sfm_plugin/ActorVelocitySubscriberPlugin.hpp>

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

#define WALKING_ANIMATION "walking"

#include <string>
#include <memory>

namespace gazebo_plugins
{

class ActorVelocitySubscriberPluginPrivate
{
public:
  /// Callback to be called at every simulation iteration
  /// \param[in] info Updated simulation info
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);
  void get_velocities(double &vx, double &vy, double &vz);

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
//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_{nullptr};

  /// Velocity subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_{nullptr};
  // bool run_sim = false;

  /// Odom topic name
//   std::string topic_name_{"odom"};
  std::string topic_name_{"cmd_vel"};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_w_{0.0};

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

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

  // Read in the animation factor (applied in the OnUpdate function).
  double animationFactor = 1.0;
  std::string animationName;
  gazebo::physics::TrajectoryInfoPtr trajectoryInfo;
  
};

ActorVelocitySubscriberPlugin::ActorVelocitySubscriberPlugin()
: impl_(std::make_unique<ActorVelocitySubscriberPluginPrivate>())
{
}

ActorVelocitySubscriberPlugin::~ActorVelocitySubscriberPlugin()
{
}

// Load the controller
void ActorVelocitySubscriberPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Configure the plugin from the SDF file
//   impl_->ros_node_ = gazebo_ros::Node::Get(sdf, model);
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  impl_->actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
  impl_->world = impl_->actor->GetWorld();
//   std::vector<physics::LinkPtr> links = model->GetLinks();
  ignition::math::Vector3d actorPos = impl_->actor->WorldPose().Pos();
//   this->sdf = sdf;
 
  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  if (!sdf->HasElement("update_rate")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "p3d plugin missing <update_rate>, defaults to 0.0"
      " (as fast as possible)");
  } else {
    impl_->update_rate_ = sdf->GetElement("update_rate")->Get<double>();
  }
  std::cout << " ********  ActorVelocitySubscriberPlugin started for " << impl_->actor->GetName() << " at initial location: " << actorPos << " ******** " << std::endl;

  impl_->sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&ActorVelocitySubscriberPluginPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));


  // impl_->sub_ = impl_->create_subscriber<std_msgs::msg::bool>
  impl_->topic_name_ = impl_->sub_->get_topic_name();
  RCLCPP_DEBUG(
    impl_->ros_node_->get_logger(), "Subscribed to topic [%s]", impl_->topic_name_.c_str());

  impl_->last_time_ = model->GetWorld()->SimTime();

  // Listen to the update event. This event is broadcast every simulation iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ActorVelocitySubscriberPluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

  if (sdf->HasElement("animation_factor"))
    impl_->animationFactor = sdf->Get<double>("animation_factor");
  else
    impl_->animationFactor = 5.1;//4.5;

  if (sdf->HasElement("animation_name")) {
    impl_->animationName = sdf->Get<std::string>("animation_name");
  } else
    impl_->animationName = WALKING_ANIMATION;

  // Reset
  auto skelAnims = impl_->actor->SkeletonAnimations();
  if (skelAnims.find(impl_->animationName) == skelAnims.end()) {
    gzerr << "Skeleton animation " << impl_->animationName << " not found.\n";
  } else {
    // Create custom trajectory
    impl_->trajectoryInfo.reset(new gazebo::physics::TrajectoryInfo());
    impl_->trajectoryInfo->type = impl_->animationName;
    impl_->trajectoryInfo->duration = 1.0;

    impl_->actor->SetCustomTrajectory(impl_->trajectoryInfo);
  }
}

void ActorVelocitySubscriberPluginPrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_x_ = _msg->linear.x;
  target_y_ = _msg->linear.y;
  target_w_ = _msg->angular.z;
}

void ActorVelocitySubscriberPluginPrivate::get_velocities( double &vx, double &vy, double &wz){
  std::lock_guard<std::mutex> scoped_lock(lock_);
  vx = target_x_;
  vy = target_y_;
  wz = target_w_;
}

// Update the controller
void ActorVelocitySubscriberPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
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
//   nav_msgs::msg::Odometry pose_msg;
  geometry_msgs::msg::Twist vel_msg;

  double vx, vy, wz;
  ActorVelocitySubscriberPluginPrivate::get_velocities(vx, vy, wz);

  double dt = (current_time - last_time_).Double();
  ignition::math::Pose3d actorPose = this->actor->WorldPose();
  

  // std::lock_guard<std::mutex> scoped_lock(lock_);

  // ignition::math::Vector3d rpy = actorPose.Rot().Euler();
  // utils::Angle current = utils::Angle::fromRadian(rpy.Z());
  // double diff = (h - current).toRadian();
  // if (std::fabs(diff) > IGN_DTOR(10)) {
  //   current = current + utils::Angle::fromRadian(diff * 0.005);
  //   yaw = current.toRadian();
  // }
  actorPose.Pos().X(actorPose.Pos().X()+vx * dt);
  actorPose.Pos().Y(actorPose.Pos().Y()+vy * dt);
  double yaw = std::atan2( vy, vx ) + 3.14/2;
  actorPose.Rot() =
      ignition::math::Quaterniond(1.5707, 0, yaw); // rpy.Z()+yaw.Radian());
  // actorPose.Rot() =
  //     ignition::math::Quaterniond(0, 0, yaw); // rpy.Z()+yaw.Radian());

  // Get inertial rates
  ignition::math::Vector3d vpos = actor->WorldLinearVel(); //link_->WorldLinearVel();
  ignition::math::Vector3d veul = actor->WorldAngularVel(); //link_->WorldAngularVel();

  // Get pose/orientation
  auto pose = actor->WorldPose();//.Pos();//link_->WorldPose();

   // Differentiate position
  if (pos_init){
    vpos = (pose.Pos() - actorPos_prev)/tmp_dt;
  }
  // std::cout << "Pose " << pose.Pos() << " Vel: " << vpos << std::endl;
  // ignition::math::Vector3d actorPos2 = pose.Pos();

  double distanceTraveled =
      (actorPose.Pos() - this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(actorPose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
                             (distanceTraveled * this->animationFactor));

  // Save last time stamp
  last_time_ = current_time;
  actorPos_prev = pose.Pos();
  pos_init = true;

}



GZ_REGISTER_MODEL_PLUGIN(ActorVelocitySubscriberPlugin) //GazeboRosP3DTest

}  // namespace gazebo_plugins