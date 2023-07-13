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
 * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
 *********************************************************************/
#include <hateb_local_planner/optimized_vel.h>
#define GET_PLAN_SRV "/global_planner/planner/make_plan"
// #define GET_PLAN_SRV "/move_base/GlobalPlanner/make_plan"
#define OPTIMIZE_SRV "/local_planner/hateb_local_planner_test/optimize"
#define AGENTS_SUB "/tracked_agents"
// #define ROBOT_GOAL_SUB "robot_goal"
#define DEFAULT_AGENT_PART cohan_msgs::TrackedSegmentType::TORSO
#define EPS 1e-20

namespace hateb_local_planner{
OptimizedVel::OptimizedVel(tf2_ros::Buffer &tf2_) : initialized_(false), predict_behind_robot_(true), got_robot_plan(false),
                                                     got_agent_plan(false), tf_(tf2_), tfListener_(tf_)
{
}

OptimizedVel::~OptimizedVel()
{
}

void OptimizedVel::initialize()
{
  if(!initialized_){
    ros::NodeHandle nh("~");

    std::string get_plan_srv_name = std::string(GET_PLAN_SRV);
    std::string optimize_srv_name = std::string(OPTIMIZE_SRV);

    getPlan_client  = nh.serviceClient<nav_msgs::GetPlan>(get_plan_srv_name, true);
    optimize_client = nh.serviceClient<hateb_local_planner::Optimize>(optimize_srv_name, true);
    agents_sub_ = nh.subscribe(AGENTS_SUB, 1, &OptimizedVel::UpdateStartPoses, this);
    get_vel_srv_ = nh.advertiseService("get_vel", &OptimizedVel::get_vel_srv, this);
    initialized_ = true;
    ROS_INFO("Everything is ready !");
  }
}

void OptimizedVel::UpdateStartPoses(const cohan_msgs::TrackedAgents &tracked_agents){
  agents_start_poses.clear();
  tracked_agents_ = tracked_agents;
  for(auto &agent: tracked_agents_.agents){
    for(auto &segment : agent.segments){
      if(segment.type == DEFAULT_AGENT_PART){
        geometry_msgs::PoseStamped hum_pose;
        hum_pose.pose = segment.pose.pose;
        hum_pose.header.frame_id ="map";
        hum_pose.header.stamp = ros::Time::now();
        agents_start_poses.push_back(hum_pose);
      }
    }
  }

  try{
    std::string base = "base_footprint";
    robot_to_map_tf = tf_.lookupTransform("map", base, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  robot_start_pose.header = robot_to_map_tf.header;
  robot_start_pose.pose.position.x =  robot_to_map_tf.transform.translation.x;
  robot_start_pose.pose.position.y =  robot_to_map_tf.transform.translation.y;
  robot_start_pose.pose.position.z =  robot_to_map_tf.transform.translation.z;
  robot_start_pose.pose.orientation.x =  robot_to_map_tf.transform.rotation.x;
  robot_start_pose.pose.orientation.y =  robot_to_map_tf.transform.rotation.y;
  robot_start_pose.pose.orientation.z =  robot_to_map_tf.transform.rotation.z;
  robot_start_pose.pose.orientation.w =  robot_to_map_tf.transform.rotation.w;
}

void OptimizedVel::correctPose(geometry_msgs::Pose &behind_pose){
  behind_pose.position.x = (fabs(behind_pose.position.x) > EPS) ? behind_pose.position.x : 0.0;
  behind_pose.position.y = (fabs(behind_pose.position.y) > EPS) ? behind_pose.position.y : 0.0;
  behind_pose.position.z = (fabs(behind_pose.position.z) > EPS) ? behind_pose.position.z : 0.0;
  behind_pose.orientation.x = 0.0;
  behind_pose.orientation.y = 0.0;
  behind_pose.orientation.z = (fabs(behind_pose.orientation.z) > EPS) ? behind_pose.orientation.z : 0.0;
  behind_pose.orientation.w = (fabs(behind_pose.orientation.w) < 1.0) ? behind_pose.orientation.w : 0.0;
}


bool OptimizedVel::checkGoal(geometry_msgs::PoseStamped goal){
  if(goal.header.frame_id == "")
    return false;
  else if(goal.pose.orientation.w == 0.0)
    return false;
  else
    return true;
}

geometry_msgs::Twist OptimizedVel::OptimizeAndgetVel(const geometry_msgs::PoseStamped &robot_goal){
  robot_goal_= robot_goal;
  geometry_msgs::Twist cmd_vel_;

  if(!checkGoal(robot_goal_)){
    cmd_vel_.linear.z =  -100;
    return cmd_vel_;
  }

  auto now = ros::Time::now();
  nav_msgs::GetPlan agent_plan_srv, robot_plan_srv;
  cohan_msgs::AgentPathArray hum_path_arr;
  hum_path_arr.header.frame_id = "map";
  hum_path_arr.header.stamp = now;

  //get global robot_plan
  robot_plan_srv.request.start = robot_start_pose;
  robot_plan_srv.request.goal = robot_goal_;
  if(getPlan_client.call(robot_plan_srv)){
    if(robot_plan_srv.response.plan.poses.size()>0)
      got_robot_plan = true;
    else
      got_robot_plan = false;
  }
  else{
    ROS_ERROR_NAMED("Optimed_Vel", "Cannot subscribe to the service %s", GET_PLAN_SRV);
    cmd_vel_.linear.z =  -100;
    return cmd_vel_;
  }


  int idx = 0;
  for(auto &agent : tracked_agents_.agents){
    if(agent.track_id == 1){
      if(predict_behind_robot_){
        tf2::Transform behind_tr, robot_to_map_tf_;
        behind_tr.setOrigin(tf2::Vector3(-0.5, 0.0, 0.0));
        tf2::fromMsg(robot_to_map_tf.transform,robot_to_map_tf_);
        behind_tr = robot_to_map_tf_ * behind_tr;
        geometry_msgs::Pose behind_pose;
        tf2::toMsg(behind_tr, behind_pose);
        correctPose(behind_pose);

        geometry_msgs::PoseStamped agent_goal;
        agent_goal.header.frame_id = "map";
        agent_goal.header.stamp = now;
        agent_goal.pose = behind_pose;
        agents_goals_.push_back(agent_goal);

        agent_plan_srv.request.start = agents_start_poses[idx];
        agent_plan_srv.request.goal = agent_goal;

        if(getPlan_client.call(agent_plan_srv)){
          if(agent_plan_srv.response.plan.poses.size()>0)
            got_agent_plan = true;
          else
            got_agent_plan = false;
        }

        cohan_msgs::AgentPath temp;
        temp.header = agent_goal.header;
        temp.id = agent.track_id;
        temp.path = agent_plan_srv.response.plan;
        hum_path_arr.paths.push_back(temp);

      }
    }
    else{
      agents_goals_.push_back(agents_start_poses[idx]);
    }
    idx++;
  }

  agents_plans = hum_path_arr;
  robot_plan = robot_plan_srv.response.plan;

  if(got_agent_plan && got_robot_plan){
    hateb_local_planner::Optimize optim_srv;

    optim_srv.request.robot_plan = robot_plan_srv.response.plan;
    optim_srv.request.agent_path_array = hum_path_arr;

    if(optimize_client.call(optim_srv)){
      if(optim_srv.response.success){
        // ROS_INFO("Optimization success");
        cmd_vel_ = optim_srv.response.cmd_vel;
      }
      else
        ROS_INFO("Optimization failed !!");
      }
  }

  return cmd_vel_;
}

bool OptimizedVel::get_vel_srv(hateb_local_planner::getOptimVel::Request &req, hateb_local_planner::getOptimVel::Response &res){

  auto cmd_vel_ = OptimizeAndgetVel(req.robot_goal);

  if(cmd_vel_.linear.z == 0){
    res.success = true;
    res.message = "Got optim vel";
    res.cmd_vel = cmd_vel_;
  }
  else{
    res.success = false;
    res.message = "Failed to get velocity !";
  }

  return true;
}

}// namespace hateb_local_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "optim_vel");

  tf2_ros::Buffer tf2;

  hateb_local_planner::OptimizedVel getvel(tf2);
  getvel.initialize();

  ros::spin();

  return 0;
}
