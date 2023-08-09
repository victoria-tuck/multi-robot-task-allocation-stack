#ifndef GAZEBO_ROS_ACTOR_PLUGIN_INCLUDE_GAZEBO_ROS_ACTOR_COMMAND
#define GAZEBO_ROS_ACTOR_PLUGIN_INCLUDE_GAZEBO_ROS_ACTOR_COMMAND

#include <rclcpp/rclcpp.hpp>
// #include <ros/callback_queue.h>
#include <geometry_msgs/msg/twist.hpp>

#include <string>
#include <queue>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

class GazeboRosActorCommand : public ModelPlugin {
 public:
  /// \brief Constructor
  GazeboRosActorCommand();

  /// \brief Destructor
  ~GazeboRosActorCommand();

  /// \brief Load the actor plugin.
  /// \param[in] _model Pointer to the parent model.
  /// \param[in] _sdf Pointer to the plugin's SDF elements.
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // \brief Reset the plugin.
  virtual void Reset();

 private:
  /// \brief Callback function for receiving velocity commands from a publisher.
  /// \param[in] _model Pointer to the incoming velocity message.
  void VelCallback(const geometry_msgs::Twist::ConstPtr &msg);

  /// \brief Function that is called every update cycle.
  /// \param[in] _info Timing information.
  void OnUpdate(const common::UpdateInfo &_info);

  /// \brief Custom callback queue thread for velocity commands.
  void VelQueueThread();

  /// \brief ROS node handle.
//   ros::NodeHandle *ros_node_;

  /// \brief Subscribers for velocity and path commands.
  rclcpp::Subscription vel_sub_;
  rclcpp::Subscription path_sub_;

//   /// \brief Custom callback queues for velocity and path commands.
//   ros::CallbackQueue vel_queue_;
//   ros::CallbackQueue path_queue_;

//   /// \brief Custom callback queue threads for velocity and path commands.
//   boost::thread velCallbackQueueThread_;
//   boost::thread pathCallbackQueueThread_;

  /// \brief Topic names for velocity and path commands.
  std::string vel_topic_;
//   std::string path_topic_;

  /// \brief Pointer to the parent actor.
  physics::ActorPtr actor_;

  /// \brief Pointer to the world
  physics::WorldPtr world_;

  /// \brief Pointer to the sdf element.
  sdf::ElementPtr sdf_;

  /// \brief Multiplier to base animation speed to adjust
  /// the speed of actor's animation and foot swinging.
  double animation_factor_;

  /// \brief List of connections
  std::vector<event::ConnectionPtr> connections_;

  /// \brief Time of the last update.
  common::Time last_update_;

  /// \brief Custom trajectory info.
  physics::TrajectoryInfoPtr trajectoryInfo_;

//   /// \brief Flag to determine if
//   /// the plugin will follow a path or velocity subscriber
//   std::string follow_mode_;

//   /// \brief Target walking velocity for the actor
//   ignition::math::Pose3d target_vel_;

//   /// \brief Speed at which actor moves along path during path-following
//   double lin_velocity_;

  /// \brief Speed at which actor rotates to achieve desired orientation
  /// during rotational alignment
  double ang_velocity_;

//   /// \brief Current target pose
//   ignition::math::Vector3d target_pose_;

  /// \brief List of target poses
  std::vector<ignition::math::Vector3d> target_poses_;

  /// \brief Index of current target pose
  int idx_;

  /// \brief Maximum allowed distance between actor and target pose
  /// during path-following
  double lin_tolerance_;

  /// \brief Maximum allowable difference in orientation between
  /// actor's current and desired orientation during rotational alignment
  double ang_tolerance_;

  /// \brief Default rotation for an actor
  double default_rotation_;

  /// \brief Helper function to choose a new target pose
  void ChooseNewTarget();

  /// \brief Data structure for saving velocity command
//   std::queue<ignition::math::Vector3d> cmd_queue_;
};
}

#endif // COMMAND_ACTOR_H