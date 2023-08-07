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

#ifndef SIM_VISUALIZER_H
#define SIM_VISUALIZER_H

// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <functional>
#include <memory>
#include <queue>

#include <pedsim_msgs/msg/agent_force.hpp>
#include <pedsim_msgs/msg/agent_group.hpp>
#include <pedsim_msgs/msg/agent_groups.hpp>
#include <pedsim_msgs/msg/agent_state.hpp>
#include <pedsim_msgs/msg/agent_states.hpp>
#include <pedsim_msgs/msg/line_obstacle.hpp>
#include <pedsim_msgs/msg/line_obstacles.hpp>
#include <pedsim_msgs/msg/waypoint.hpp>
#include <pedsim_msgs/msg/waypoints.hpp>

#include <pedsim_msgs/msg/social_activities.hpp>
#include <pedsim_msgs/msg/social_activity.hpp>
#include <pedsim_msgs/msg/social_relation.hpp>
#include <pedsim_msgs/msg/social_relations.hpp>
#include <pedsim_msgs/msg/tracked_group.hpp>
#include <pedsim_msgs/msg/tracked_groups.hpp>
#include <pedsim_msgs/msg/tracked_person.hpp>
#include <pedsim_msgs/msg/tracked_persons.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// #include <dynamic_reconfigure/server.h>
// #include <pedsim_visualizer/PedsimVisualizerConfig.h>

namespace pedsim {

class SimVisualizer: public rclcpp::Node {

 public:
  // using VizConfig = pedsim_visualizer::PedsimVisualizerConfig;

  explicit SimVisualizer(std::string node_name);
  ~SimVisualizer();
  SimVisualizer(const SimVisualizer& other) = delete;

  void run();

  // callbacks.
  void agentStatesCallBack(const pedsim_msgs::msg::AgentStates::SharedPtr agents);
  void agentGroupsCallBack(const pedsim_msgs::msg::AgentGroups::SharedPtr groups);
  void obstaclesCallBack(const pedsim_msgs::msg::LineObstacles::SharedPtr obstacles);
  void waypointsCallBack(const pedsim_msgs::msg::Waypoints::SharedPtr waypoints);

 protected:
  /// publishers
  void publishAgentVisuals();
  void publishRelationVisuals();
  void publishActivityVisuals();
  void publishGroupVisuals();
  void publishObstacleVisuals();
  void publishWaypointVisuals();

 private:
  void setupPublishersAndSubscribers();

  double hz_;

  /// publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_obstacles_visuals_;
  rclcpp::Publisher<pedsim_msgs::msg::TrackedPersons>::SharedPtr pub_person_visuals_;
  rclcpp::Publisher<pedsim_msgs::msg::TrackedGroups>::SharedPtr pub_group_visuals_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_forces_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_waypoints_;

  /// Subscribers.
  rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr sub_states_;
  rclcpp::Subscription<pedsim_msgs::msg::AgentGroups>::SharedPtr sub_groups_;
  rclcpp::Subscription<pedsim_msgs::msg::LineObstacles>::SharedPtr sub_obstacles_;
  rclcpp::Subscription<pedsim_msgs::msg::Waypoints>::SharedPtr sub_waypoints_;

  /// Local data queues.
  std::queue<pedsim_msgs::msg::AgentStates::SharedPtr> q_people_;
  std::queue<pedsim_msgs::msg::AgentGroups::SharedPtr> q_groups_;
  std::queue<pedsim_msgs::msg::LineObstacles::SharedPtr> q_obstacles_;
  std::queue<pedsim_msgs::msg::Waypoints::SharedPtr> q_waypoints_;
};
}  // namespace pedsim

#endif
