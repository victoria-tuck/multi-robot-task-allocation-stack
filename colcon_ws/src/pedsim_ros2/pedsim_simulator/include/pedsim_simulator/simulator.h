/**
* Copyright 2014-2016 Social Robotics Lab, University of Freiburg
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

#ifndef SIMULATOR_H
#define SIMULATOR_H

// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <functional>
#include <memory>

#include <pedsim_msgs/AgentForce.h>
#include <pedsim_msgs/AgentGroup.h>
#include <pedsim_msgs/AgentGroups.h>
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/LineObstacle.h>
#include <pedsim_msgs/LineObstacles.h>
#include <pedsim_msgs/Waypoint.h>
#include <pedsim_msgs/Waypoints.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>

#include <pedsim_simulator/agentstatemachine.h>
#include <pedsim_simulator/config.h>
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentgroup.h>
#include <pedsim_simulator/element/attractionarea.h>
#include <pedsim_simulator/element/obstacle.h>
#include <pedsim_simulator/element/waitingqueue.h>
#include <pedsim_simulator/element/waypoint.h>
#include <pedsim_simulator/scenarioreader.h>
#include <pedsim_simulator/scene.h>

#include <dynamic_reconfigure/server.h>
#include <pedsim_simulator/PedsimSimulatorConfig.h>

using SimConfig = pedsim_simulator::PedsimSimulatorConfig;

/// \class Simulator
/// \brief Simulation wrapper
/// \details ROS interface to the scene object provided by pedsim
class Simulator : public rclcpp::Node {
 
  public:
      explicit Simulator(): Node("pedsim_simulator");
      virtual ~Simulator();

      void runSimulation();

      // callbacks
      bool onPauseSimulation(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response);
      bool onUnpauseSimulation(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response);

      void spawnCallback(const ros::TimerEvent& event);

 protected:
  void reconfigureCB(SimConfig& config, uint32_t level);
  dynamic_reconfigure::Server<SimConfig> server_;

 private:
  void updateRobotPositionFromTF();
  void publishAgents();
  void publishGroups();
  void publishObstacles();
  void publishRobotPosition();
  void publishWaypoints();

 private:
  
  bool paused_;
  ros::Timer spawn_timer_;

  // publishers
  rclcpp::Publisher<pedsim_msgs::LineObstacles>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<pedsim_msgs::AgentStates>::SharedPtr pub_agent_states_;
  rclcpp::Publisher<pedsim_msgs::AgentGroups>::SharedPtr pub_agent_groups_;
  rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr pub_robot_position_;
  rclcpp::Publisher<pedsim_msgs::Waypoints>::SharedPtr pub_waypoints_;

  // provided services
  rclcpp::Service<std_srvs::Empty> srv_pause_simulation_;
  rcpcpp::Service<std_srvs::Empty> srv_unpause_simulation_;

  // frame ids
  std::string frame_id_;
  std::string robot_base_frame_id_;

  // pointers and additional data
  // std::unique_ptr<tf::TransformListener> transform_listener_;
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  geometry_msgs::msg::TransformStamped last_robot_pose_;
  // tf::StampedTransform last_robot_pose_;
  Agent* robot_;
  geometry_msgs::Quaternion last_robot_orientation_;

  inline std::string agentStateToActivity(
      const AgentStateMachine::AgentState& state) const;

  inline std_msgs::Header createMsgHeader() const;
};

#endif
