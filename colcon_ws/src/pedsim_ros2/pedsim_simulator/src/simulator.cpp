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
* \author Sven Wehner <mail@svenwehner.de>
*/

#include <QApplication>
#include <algorithm>

#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/simulator.h>

#include <pedsim_utils/geometry.h>

using namespace pedsim;

Simulator::~Simulator() {
  // shutdown service servers and publishers
  pub_obstacles_.shutdown();
  pub_agent_states_.shutdown();
  pub_agent_groups_.shutdown();
  pub_robot_position_.shutdown();
  pub_waypoints_.shutdown();

  srv_pause_simulation_.shutdown();
  srv_unpause_simulation_.shutdown();

  delete robot_;
  QCoreApplication::exit(0);
}

bool Simulator::initializeSimulation() {
  int queue_size = 0;
  nh_.param<int>("default_queue_size", queue_size, 1);
  RCPCPP_INFO_STREAM(this->get_logger(), "Using default queue size of "
                  << queue_size << " for publisher queues... "
                  << (queue_size == 0
                          ? "NOTE: This means the queues are of infinite size!"
                          : ""));

  // setup ros publishers
  pub_obstacles_ = this->create_publisher<pedsim_msgs::LineObstacles>("simulated_walls", queue_size);
  pub_agent_states_ = this->create_publisher<pedsim_msgs::AgentStates>("simulated_agents", queue_size);
  pub_agent_groups_ = this->create_publisher<pedsim_msgs::AgentGroups>("simulated_groups", queue_size);
  pub_robot_position_ = this->create_publisher<nav_msgs::Odometry>("robot_position", queue_size);
  pub_waypoints_ = this->create_publisher<pedsim_msgs::Waypoints>("simulated_waypoints", queue_size);

  // services
  srv_pause_simulation_ = this->create_service<std_srvs::Empty>("pause_simulation", &Simulator::onPauseSimulation, this);
  srv_unpause_simulation_ = this->create_service<std_srvs::Empty>("unpause_simulation", &Simulator::onUnpauseSimulation, this);

  // setup TF listener and other pointers
  transform_listener_.reset(new tf2::TransformListener());
  robot_ = nullptr;

  // load additional parameters
  std::string scene_file_param;
  nh_.param<std::string>("scene_file", scene_file_param, "");
  if (scene_file_param == "") {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid scene file: " << scene_file_param);
    return false;
  }

  RCLCPP_INFO_STREAM(this->get_logger, "Loading scene [" << scene_file_param << "] for simulation");

  const QString scenefile = QString::fromStdString(scene_file_param);
  ScenarioReader scenario_reader;
  if (scenario_reader.readFromFile(scenefile) == false) {
    RCPCPP_ERROR_STREAM(this->get_logger(),
        "Could not load the scene file, please check the paths and param "
        "names : "
        << scene_file_param);
    return false;
  }

  this->declare_parameter<bool>("enable_groups", CONFIG.groups_enabled, true);
  this->declare_parameter<double>("max_robot_speed", CONFIG.max_robot_speed, 1.5);
  this->declare_parameter<double>("update_rate", CONFIG.updateRate, 25.0);
  this->declare_parameter<double>("simulation_factor", CONFIG.simulationFactor, 1.0);

  int op_mode = 1;
  this->declare_parameter<int>("robot_mode", op_mode, 1);
  CONFIG.robot_mode = static_cast<RobotMode>(op_mode);

  double spawn_period;
  this->declare_parameter<double>("spawn_period", spawn_period, 5.0);
  this->declare_parameter<std::string>("frame_id", frame_id_, "odom");
  this->declare_parameter<std::string>("robot_base_frame_id", robot_base_frame_id_,
      "base_footprint");

  paused_ = false;

  spawn_timer_ = this->create_wall_timer( rclcpp::Duration(spawn_period), std::bind( &Simulator::spawnCallback, this )   )
  return true;
}

void Simulator::run() {
   if (!robot_) {
      // setup the robot
      for (Agent* agent : SCENE.getAgents()) {
        if (agent->getType() == Ped::Tagent::ROBOT) {
          robot_ = agent;
          last_robot_orientation_ =
              poseFrom2DVelocity(robot_->getvx(), robot_->getvy());
        }
      }
    }

    if (!paused_) {
      updateRobotPositionFromTF();
      SCENE.moveAllAgents();

      publishAgents();
      publishGroups();
      publishRobotPosition();
      publishObstacles();
      publishWaypoints();
    }
}

void Simulator::runSimulation() {

  rate_timer = this->create_wall_timer( rclcpp::Duration(1.0/CONFIG.updateRate), std::bind( &Simulator::run, this )   )
  // TODO
  // ros::Rate r(CONFIG.updateRate);
}

bool Simulator::onPauseSimulation(std_srvs::Empty::Request& request,
                                  std_srvs::Empty::Response& response) {
  paused_ = true;
  return true;
}

bool Simulator::onUnpauseSimulation(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response) {
  paused_ = false;
  return true;
}

void Simulator::spawnCallback(const ros::TimerEvent& event) {
  ROS_DEBUG_STREAM("Spawning new agents.");

  for (const auto& sa : SCENE.getSpawnAreas()) {
    AgentCluster* agentCluster = new AgentCluster(sa->x, sa->y, sa->n);
    agentCluster->setDistribution(sa->dx, sa->dy);
    agentCluster->setType(static_cast<Ped::Tagent::AgentType>(0));

    for (const auto& wp_name : sa->waypoints) {
      agentCluster->addWaypoint(SCENE.getWaypointByName(wp_name));
    }

    SCENE.addAgentCluster(agentCluster);
  }
}

void Simulator::updateRobotPositionFromTF() {
  if (!robot_) return;

  if (CONFIG.robot_mode == RobotMode::TELEOPERATION ||
      CONFIG.robot_mode == RobotMode::CONTROLLED) {
    robot_->setTeleop(true);
    robot_->setVmax(2 * CONFIG.max_robot_speed);

    //NEW
    // Get robot position via TF
    geometry_msgs::msg::TransformStamped tfTransform;
    try {
          t = tf_buffer_->lookupTransform(
            frame_id_, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            frame_id_.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }

    const double x = tfTransform.transform.translation.x;
    const double y = tfTransform.transform.translation.y;
    const double dx = x - last_robot_pose_.transform.translation.x,
                 dy = y - last_robot_pose_.transform.translation.y;
    const double dt =
        tfTransform.header.stamp_.toSec() - last_robot_pose_.header.stamp_.toSec();
    double vx = dx / dt, vy = dy / dt;

    if (!std::isfinite(vx)) vx = 0;
    if (!std::isfinite(vy)) vy = 0;

    RCLCPP_DEBUG_STREAM(this->get_logger(), "rx, ry: " << robot_->getx() << ", " << robot_->gety() << " vs: " << x << ", " << y);

    robot_->setX(x);
    robot_->setY(y);
    robot_->setvx(vx);
    robot_->setvy(vy);


    ROS_DEBUG_STREAM("Robot speed: " << std::hypot(vx, vy) << " dt: " << dt);

    last_robot_pose_ = tfTransform;
  }
}

void Simulator::publishRobotPosition() {
  if (robot_ == nullptr) return;

  nav_msgs::Odometry robot_location;
  robot_location.header = createMsgHeader();
  robot_location.child_frame_id = robot_base_frame_id_;

  robot_location.pose.pose.position.x = robot_->getx();
  robot_location.pose.pose.position.y = robot_->gety();
  if (hypot(robot_->getvx(), robot_->getvy()) < 0.05) {
    robot_location.pose.pose.orientation = last_robot_orientation_;
  } else {
    robot_location.pose.pose.orientation =
        poseFrom2DVelocity(robot_->getvx(), robot_->getvy());
    last_robot_orientation_ = robot_location.pose.pose.orientation;
  }

  robot_location.twist.twist.linear.x = robot_->getvx();
  robot_location.twist.twist.linear.y = robot_->getvy();

  pub_robot_position_->publish(robot_location);
}

void Simulator::publishAgents() {
  if (SCENE.getAgents().size() < 2) {
    return;
  }

  pedsim_msgs::AgentStates all_status;
  all_status.header = createMsgHeader();

  auto VecToMsg = [](const Ped::Tvector& v) {
    geometry_msgs::Vector3 gv;
    gv.x = v.x;
    gv.y = v.y;
    gv.z = v.z;
    return gv;
  };

  for (const Agent* a : SCENE.getAgents()) {
    pedsim_msgs::AgentState state;
    state.header = createMsgHeader();

    state.id = a->getId();
    state.type = a->getType();
    state.pose.position.x = a->getx();
    state.pose.position.y = a->gety();
    state.pose.position.z = a->getz();
    auto theta = std::atan2(a->getvy(), a->getvx());
    state.pose.orientation = pedsim::angleToQuaternion(theta);

    state.twist.linear.x = a->getvx();
    state.twist.linear.y = a->getvy();
    state.twist.linear.z = a->getvz();

    AgentStateMachine::AgentState sc = a->getStateMachine()->getCurrentState();
    state.social_state = agentStateToActivity(sc);
    if (a->getType() == Ped::Tagent::ELDER) {
      state.social_state = pedsim_msgs::AgentState::TYPE_STANDING;
    }

    // Skip robot.
    if (a->getType() == Ped::Tagent::ROBOT) {
      continue;
    }

    // Forces.
    pedsim_msgs::AgentForce agent_forces;
    agent_forces.desired_force = VecToMsg(a->getDesiredDirection());
    agent_forces.obstacle_force = VecToMsg(a->getObstacleForce());
    agent_forces.social_force = VecToMsg(a->getSocialForce());
    // agent_forces.group_coherence_force = a->getSocialForce();
    // agent_forces.group_gaze_force = a->getSocialForce();
    // agent_forces.group_repulsion_force = a->getSocialForce();
    // agent_forces.random_force = a->getSocialForce();

    state.forces = agent_forces;

    all_status.agent_states.push_back(state);
  }

  pub_agent_states_->publish(all_status);
}

void Simulator::publishGroups() {
  if (!CONFIG.groups_enabled) {
    ROS_DEBUG_STREAM("Groups are disabled, no group data published: flag="
                     << CONFIG.groups_enabled);
    return;
  }

  if (SCENE.getGroups().size() < 1) {
    return;
  }

  pedsim_msgs::AgentGroups sim_groups;
  sim_groups.header = createMsgHeader();

  for (const auto& ped_group : SCENE.getGroups()) {
    if (ped_group->memberCount() <= 1) continue;

    pedsim_msgs::AgentGroup group;
    group.group_id = ped_group->getId();
    group.age = 10;
    const Ped::Tvector com = ped_group->getCenterOfMass();
    group.center_of_mass.position.x = com.x;
    group.center_of_mass.position.y = com.y;

    for (const auto& member : ped_group->getMembers()) {
      group.members.emplace_back(member->getId());
    }
    sim_groups.groups.emplace_back(group);
  }
  pub_agent_groups_->publish(sim_groups);
}

void Simulator::publishObstacles() {
  pedsim_msgs::LineObstacles sim_obstacles;
  sim_obstacles.header = createMsgHeader();
  for (const auto& obstacle : SCENE.getObstacles()) {
    pedsim_msgs::LineObstacle line_obstacle;
    line_obstacle.start.x = obstacle->getax();
    line_obstacle.start.y = obstacle->getay();
    line_obstacle.start.z = 0.0;
    line_obstacle.end.x = obstacle->getbx();
    line_obstacle.end.y = obstacle->getby();
    line_obstacle.end.z = 0.0;
    sim_obstacles.obstacles.push_back(line_obstacle);
  }
  pub_obstacles_->publish(sim_obstacles);
}

void Simulator::publishWaypoints() {
  pedsim_msgs::Waypoints sim_waypoints;
  sim_waypoints.header = createMsgHeader();
  for (const auto& waypoint : SCENE.getWaypoints()) {
    pedsim_msgs::Waypoint wp;
    wp.name = waypoint->getName().toStdString();
    wp.behavior = waypoint->getBehavior();
    wp.radius = waypoint->getRadius();
    wp.position.x = waypoint->getPosition().x;
    wp.position.y = waypoint->getPosition().y;
    sim_waypoints.waypoints.push_back(wp);
  }
  pub_waypoints_.publish(sim_waypoints);
}

std::string Simulator::agentStateToActivity(
    const AgentStateMachine::AgentState& state) const {
  std::string activity = "Unknown";
  switch (state) {
    case AgentStateMachine::AgentState::StateWalking:
      activity = pedsim_msgs::AgentState::TYPE_INDIVIDUAL_MOVING;
      break;
    case AgentStateMachine::AgentState::StateGroupWalking:
      activity = pedsim_msgs::AgentState::TYPE_GROUP_MOVING;
      break;
    case AgentStateMachine::AgentState::StateQueueing:
      activity = pedsim_msgs::AgentState::TYPE_WAITING_IN_QUEUE;
      break;
    case AgentStateMachine::AgentState::StateShopping:
      break;
    case AgentStateMachine::AgentState::StateNone:
      break;
    case AgentStateMachine::AgentState::StateWaiting:
      break;
  }
  return activity;
}

std_msgs::Header Simulator::createMsgHeader() const {
  std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  msg_header.frame_id = frame_id_;
  return msg_header;
}
