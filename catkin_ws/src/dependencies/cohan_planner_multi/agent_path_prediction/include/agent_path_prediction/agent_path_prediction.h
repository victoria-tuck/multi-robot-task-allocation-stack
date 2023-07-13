/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020 LAAS/CNRS
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
 *  Authors: Harmish Khambhaita, Phani Teja Singamaneni
 *********************************************************************/

#ifndef AGENT_PATH_PREDICTION_H_
#define AGENT_PATH_PREDICTION_H_

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <agent_path_prediction/AgentPathPredictionConfig.h>

#include <cohan_msgs/TrackedAgents.h>
#include <cohan_msgs/TrackedSegmentType.h>
#include <cohan_msgs/AgentPathArray.h>
#include <cohan_msgs/AgentTrajectory.h>
#include <cohan_msgs/AgentTrajectoryArray.h>
#include <agent_path_prediction/AgentPosePredict.h>
#include <agent_path_prediction/PredictedGoal.h>
#include <agent_path_prediction/AgentGoal.h>
#include <agent_path_prediction/AgentPose.h>
#include <tf/transform_listener.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>

#include <visualization_msgs/MarkerArray.h>

namespace agent_path_prediction {
class AgentPathPrediction {
public:
  AgentPathPrediction();
  ~AgentPathPrediction();

  void initialize();

  void setParams(std::vector<double> velscale_scales, double velscale_angle,
                 double velscale_mul, double velobs_mul, double velobs_min_rad,
                 double velobs_max_rad, double velobs_max_rad_time,
                 bool velobs_use_ang);

private:
  // ros subscribers and publishers
  ros::Subscriber tracked_agents_sub_, external_paths_sub_, external_trajs_sub_,robot_pos_sub_, predicted_goal_sub_;
  ros::Publisher predicted_agents_pub_;
  bool done_cfg;

  // ros services
  ros::ServiceServer predict_agents_server_, reset_ext_paths_server_,
      publish_markers_srv_, set_goal_srv_, set_goal_call_srv_;
  ros::ServiceClient get_plan_client_,goal_change_srv_;

  // dynamic reconfigure variables
  dynamic_reconfigure::Server<AgentPathPredictionConfig> *dsrv_;
  void reconfigureCB(AgentPathPredictionConfig &config, uint32_t level);

  // subscriber callbacks
  void trackedAgentsCB(const cohan_msgs::TrackedAgents &tracked_agents);
  void
  externalPathsCB(const cohan_msgs::AgentPathArray::ConstPtr &external_paths);

  tf::TransformListener tf_;

  std::string tracked_agents_sub_topic_, external_paths_sub_topic_,
      reset_ext_paths_service_name_, predict_service_name_,
      predicted_agents_markers_pub_topic_, publish_markers_srv_name_,
      get_plan_srv_name_;
  int default_agent_part_;
  bool publish_markers_, showing_markers_, got_new_agent_paths_, got_new_goal, got_external_goal;
  std::string robot_frame_id_, map_frame_id_;
  double agent_dist_behind_robot_, agent_angle_behind_robot_;

  struct AgentPathVel {
    uint64_t id;
    nav_msgs::Path path;
    geometry_msgs::TwistWithCovariance start_vel;
  };

  struct AgentTrajVel {
    uint64_t id;
    cohan_msgs::TrajectoryPoint traj;
    geometry_msgs::TwistWithCovariance start_vel;
  };

  struct AgentStartPoseVel {
    uint64_t id;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistWithCovariance vel;
  };

  cohan_msgs::TrackedAgents tracked_agents_;
  cohan_msgs::AgentPathArray::ConstPtr external_paths_;
  cohan_msgs::AgentPathArray external_paths2_;
  cohan_msgs::AgentTrajectoryArrayConstPtr external_trajs_;
  agent_path_prediction::PredictedGoal::ConstPtr predicted_goal_;
  std::vector<agent_path_prediction::AgentPose> external_goals_;

  std::vector<AgentPathVel> behind_path_vels_;
  std::vector<int> behind_path_vels_pos;
  std::vector<agent_path_prediction::PredictedPoses> last_predicted_poses_;
  std::map<uint64_t, size_t> last_prune_indices_;
  std::vector<double> velscale_scales_;
  std::map<uint64_t, int> last_markers_size_map;
  double velscale_angle_, velscale_mul_, velobs_mul_, velobs_min_rad_,
      velobs_max_rad_, velobs_max_rad_time_;
  bool velobs_use_ang_;
  visualization_msgs::MarkerArray predicted_agents_markers_;
  geometry_msgs::Transform behind_pose;
  bool check_path;

  bool predictAgents(agent_path_prediction::AgentPosePredict::Request &req,
                     agent_path_prediction::AgentPosePredict::Response &res);
  bool predictAgentsVelScale(agent_path_prediction::AgentPosePredict::Request &req,
                             agent_path_prediction::AgentPosePredict::Response &res);
  bool predictAgentsVelObs(agent_path_prediction::AgentPosePredict::Request &req,
                           agent_path_prediction::AgentPosePredict::Response &res);
  bool predictAgentsExternal(agent_path_prediction::AgentPosePredict::Request &req,
                             agent_path_prediction::AgentPosePredict::Response &res);
  bool predictAgentsBehind(agent_path_prediction::AgentPosePredict::Request &req,
                           agent_path_prediction::AgentPosePredict::Response &res);
  bool predictAgentsGoal(agent_path_prediction::AgentPosePredict::Request &req,
			   agent_path_prediction::AgentPosePredict::Response &res);
  bool predictAgentsFromPaths(agent_path_prediction::AgentPosePredict::Request &req,
                              agent_path_prediction::AgentPosePredict::Response &res,
                              const std::vector<AgentPathVel> &path_vels);
  bool setPublishMarkers(std_srvs::SetBool::Request &req,
                         std_srvs::SetBool::Response &res);
  bool setExternalGoal(agent_path_prediction::AgentGoal::Request &req, agent_path_prediction::AgentGoal::Response &res);
  bool checkExternalGoal(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool transformPoseTwist(const cohan_msgs::TrackedAgents &tracked_agents,
                          const uint64_t &agent_id, const std::string &to_frame,
                          geometry_msgs::PoseStamped &pose,
                          geometry_msgs::TwistStamped &twist);
  void externalTrajsCB(const cohan_msgs::AgentTrajectoryArrayConstPtr &traj_array);

  void predictedGoalCB(const agent_path_prediction::PredictedGoal::ConstPtr& predicted_goal);

  double checkdist(geometry_msgs::Pose agent, geometry_msgs::Pose robot);


  size_t
  prunePath(size_t begin_index, const geometry_msgs::Pose &pose,
            const std::vector<geometry_msgs::PoseWithCovarianceStamped> &path);

  bool resetExtPaths(std_srvs::Empty::Request &req,
                     std_srvs::Empty::Response &res);
};
}

#endif // AGENT_PATH_PREDICTION_H_
