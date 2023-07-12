/*! \file planning_framework_main.cpp
 * \brief Framework for planning collision-free paths.
 *
 * \date March 5, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
 *
 * \details Framework for planning collision-free paths online. Iterative planning
 * uses last known solution path in order to guarantee that new solution paths are always
 * at least as good as the previous one.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#include <iostream>
#include <vector>

#include <boost/bind.hpp>

// standard OMPL
// #include <ompl/control/SpaceInformation.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <planner/RRTstarMod.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
// ROS services
#include <std_srvs/Empty.h>
// ROS markers rviz
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
// ROS tf
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
// action server
#include <actionlib/server/simple_action_server.h>

// Planner
#include <new_state_sampler.h>
#include <state_cost_objective.h>
#include <state_validity_checker_grid_map_R2.h>

// Esc base controller
#include <esc_move_base_msgs/Path2D.h>
#include <esc_move_base_msgs/Goto2DAction.h>

// pedsim msgs
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef actionlib::SimpleActionServer<esc_move_base_msgs::Goto2DAction> EscBaseGoToActionServer;

//!  OnlinePlannFramework class.
/*!
 * Online Planning Framework.
 * Setup a sampling-based planner for online computation of collision-free paths.
 * C-Space: R2
 * Workspace is represented with Octomap and GridMap
 */
class OnlinePlannFramework
{
public:
    //! Constructor
    OnlinePlannFramework();
    //! Planner setup
    void planWithSimpleSetup();
    //! Periodic callback to solve the query.
    void planningTimerCallback();
    //! Callback for getting current vehicle odometry
    void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
    //! Callback for getting the 2D navigation goal
    void queryGoalCallback(const geometry_msgs::PoseStampedConstPtr &nav_goal_msg);
    //! Callback for getting the 2D navigation goal
    void goToActionCallback(const esc_move_base_msgs::Goto2DGoalConstPtr &goto_req);
    //! Procedure to visualize the resulting path
    void visualizeRRT(og::PathGeometric &geopath);
    //! Callback for getting the state of the Esc base controller.
    void controlActiveCallback(const std_msgs::BoolConstPtr &control_active_msg);

private:
    // ROS
    ros::NodeHandle nh_, local_nh_;
    ros::Timer timer_;
    ros::Subscriber odom_sub_, nav_goal_sub_, control_active_sub_;
    ros::Publisher solution_path_rviz_pub_, solution_path_control_pub_, query_goal_pose_rviz_pub_,
        query_goal_radius_rviz_pub_, num_nodes_pub_;

    // ROS action server
    EscBaseGoToActionServer *goto_action_server_;
    std::string goto_action_;
    esc_move_base_msgs::Goto2DAction goto_action_feedback_;
    esc_move_base_msgs::Goto2DAction goto_action_result_;

    // ROS TF
    tf::Pose last_robot_pose_;
    tf::TransformListener tf_listener_;

    // OMPL, online planner
    og::SimpleSetupPtr simple_setup_;
    double timer_period_, solving_time_, xy_goal_tolerance_, yaw_goal_tolerance_, robot_base_radius;
    bool opport_collision_check_, reuse_last_best_solution_, motion_cost_interpolation_, odom_available_,
        goal_available_, dynamic_bounds_, visualize_tree_,
        control_active_;
    std::vector<double> planning_bounds_x_, planning_bounds_y_, start_state_, goal_map_frame_,
        goal_odom_frame_;
    double goal_radius_;
    std::string planner_name_, optimization_objective_, odometry_topic_, query_goal_topic_, world_frame_, control_active_topic_;
    std::vector<const ob::State *> solution_path_states_;
};

//!  Constructor.
/*!
 * Load planner parameters from configuration file.
 * Publishers to visualize the resulting path.
 */
OnlinePlannFramework::OnlinePlannFramework()
    : local_nh_("~"), dynamic_bounds_(false), goto_action_server_(NULL), control_active_(false)
{
    //=======================================================================
    // Get parameters
    //=======================================================================
    planning_bounds_x_.resize(2);
    planning_bounds_y_.resize(2);
    start_state_.resize(2);
    goal_map_frame_.resize(3);
    goal_odom_frame_.resize(3);

    local_nh_.param("world_frame", world_frame_, world_frame_);
    local_nh_.param("planning_bounds_x", planning_bounds_x_, planning_bounds_x_);
    local_nh_.param("planning_bounds_y", planning_bounds_y_, planning_bounds_y_);
    local_nh_.param("start_state", start_state_, start_state_);
    local_nh_.param("goal_state", goal_map_frame_, goal_map_frame_);
    local_nh_.param("timer_period", timer_period_, timer_period_);
    local_nh_.param("solving_time", solving_time_, solving_time_);
    local_nh_.param("opport_collision_check", opport_collision_check_, opport_collision_check_);
    local_nh_.param("planner_name", planner_name_, planner_name_);
    local_nh_.param("reuse_last_best_solution", reuse_last_best_solution_, reuse_last_best_solution_);
    local_nh_.param("optimization_objective", optimization_objective_, optimization_objective_);
    local_nh_.param("motion_cost_interpolation", motion_cost_interpolation_, motion_cost_interpolation_);
    local_nh_.param("odometry_topic", odometry_topic_, odometry_topic_);
    local_nh_.param("query_goal_topic", query_goal_topic_, query_goal_topic_);
    local_nh_.param("goto_action", goto_action_, goto_action_);
    local_nh_.param("control_active_topic", control_active_topic_, control_active_topic_);
    local_nh_.param("dynamic_bounds", dynamic_bounds_, dynamic_bounds_);
    local_nh_.param("xy_goal_tolerance", xy_goal_tolerance_, 0.2);
    local_nh_.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.1);
    local_nh_.param("visualize_tree", visualize_tree_, false);
    local_nh_.param("robot_base_radius", robot_base_radius, robot_base_radius);

    goal_radius_ = xy_goal_tolerance_;
    goal_available_ = false;

    //=======================================================================
    // Subscribers
    //=======================================================================
    // Odometry data
    odom_sub_ = nh_.subscribe(odometry_topic_, 1, &OnlinePlannFramework::odomCallback, this);
    odom_available_ = false;

    // 2D Nav Goal
    nav_goal_sub_ = local_nh_.subscribe(query_goal_topic_, 1, &OnlinePlannFramework::queryGoalCallback, this);

    // Controller active flag
    control_active_sub_ =
        local_nh_.subscribe(control_active_topic_, 1, &OnlinePlannFramework::controlActiveCallback, this);

    //=======================================================================
    // Publishers
    //=======================================================================
    solution_path_rviz_pub_ = local_nh_.advertise<visualization_msgs::Marker>("solution_path", 1, true);
    solution_path_control_pub_ =
        local_nh_.advertise<esc_move_base_msgs::Path2D>("esc_move_base_solution_path", 1, true);
    query_goal_pose_rviz_pub_ =
        local_nh_.advertise<geometry_msgs::PoseStamped>("query_goal_pose_rviz", 1, true);
    query_goal_radius_rviz_pub_ =
        local_nh_.advertise<visualization_msgs::Marker>("query_goal_radius_rviz", 1, true);

    num_nodes_pub_ = local_nh_.advertise<std_msgs::Int32>("esc_num_nodes", 1, true);

    //=======================================================================
    // Action server
    //=======================================================================
    goto_action_server_ = new EscBaseGoToActionServer(
        ros::NodeHandle(), goto_action_, boost::bind(&OnlinePlannFramework::goToActionCallback, this, _1),
        false);
    //=======================================================================
    // Waiting for odometry
    //=======================================================================
    ros::Rate loop_rate(10);
    while (ros::ok() && !odom_available_)
    {
        ros::spinOnce();
        loop_rate.sleep();
        ROS_WARN("%s:\n\tWaiting for vehicle's odometry\n", ros::this_node::getName().c_str());
    }
    ROS_WARN("%s:\n\tOdometry received\n", ros::this_node::getName().c_str());

    goto_action_server_->start();
}

//! Goto action callback.
/*!
 * Callback for getting the 2D navigation goal
 */
void OnlinePlannFramework::goToActionCallback(const esc_move_base_msgs::Goto2DGoalConstPtr &goto_req)
{
    goal_map_frame_[0] = goto_req->goal.x;
    goal_map_frame_[1] = goto_req->goal.y;
    goal_map_frame_[2] = goto_req->goal.theta;

    double useless_pitch, useless_roll, yaw;

    //=======================================================================
    // Publish RViz Maker
    //=======================================================================
    geometry_msgs::PoseStamped query_goal_msg;
    query_goal_msg.header.frame_id = "map";
    query_goal_msg.header.stamp = ros::Time::now();
    query_goal_msg.pose.position.x = goto_req->goal.x;
    query_goal_msg.pose.position.y = goto_req->goal.y;
    query_goal_msg.pose.position.z = 0.0;
    query_goal_msg.pose.orientation.x = 0.0;
    query_goal_msg.pose.orientation.y = 0.0;
    query_goal_msg.pose.orientation.z = sin(goto_req->goal.theta / 2.0);
    query_goal_msg.pose.orientation.w = cos(goto_req->goal.theta / 2.0);
    query_goal_pose_rviz_pub_.publish(query_goal_msg);

    visualization_msgs::Marker radius_msg;
    radius_msg.header.frame_id = "map";
    radius_msg.header.stamp = ros::Time::now();
    radius_msg.ns = "goal_radius";
    radius_msg.action = visualization_msgs::Marker::ADD;
    radius_msg.pose.orientation.w = 1.0;
    radius_msg.id = 0;
    radius_msg.type = visualization_msgs::Marker::CYLINDER;
    radius_msg.scale.x = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.y = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.z = 0.02;
    radius_msg.color.r = 1.0;
    radius_msg.color.a = 0.5;
    radius_msg.pose.position.x = goto_req->goal.x;
    radius_msg.pose.position.y = goto_req->goal.y;
    radius_msg.pose.position.z = 0.0;
    query_goal_radius_rviz_pub_.publish(radius_msg);

    //=======================================================================
    // Transform from map to odom
    //=======================================================================
    ros::Time t;
    std::string err = "";
    tf::StampedTransform tf_map_to_fixed;
    tf_listener_.getLatestCommonTime("map", "odom", t, &err);
    tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);
    tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

    tf::Point goal_point_odom_frame(goal_map_frame_[0], goal_map_frame_[1], 0.0);
    goal_point_odom_frame = tf_map_to_fixed.inverse() * goal_point_odom_frame;
    goal_odom_frame_[0] = goal_point_odom_frame.getX();
    goal_odom_frame_[1] = goal_point_odom_frame.getY();
    goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

    goal_radius_ = xy_goal_tolerance_;

    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;

    solution_path_states_.clear();
    goal_available_ = true;

    ros::Rate loop_rate(10);
    while (ros::ok() && (goal_available_ || control_active_))
        loop_rate.sleep();

    esc_move_base_msgs::Goto2DResult result;
    result.success = true;

    goto_action_server_->setSucceeded(result);
}

//! Odometry callback.
/*!
 * Callback for getting updated vehicle odometry
 */
void OnlinePlannFramework::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    if (!odom_available_)
        odom_available_ = true;
    tf::poseMsgToTF(odom_msg->pose.pose, last_robot_pose_);

    double useless_pitch, useless_roll, yaw;
    last_robot_pose_.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

    if ((goal_available_) &&
        sqrt(pow(goal_odom_frame_[0] - last_robot_pose_.getOrigin().getX(), 2.0) +
             pow(goal_odom_frame_[1] - last_robot_pose_.getOrigin().getY(), 2.0)) < (goal_radius_ + 0.3))
    {
        goal_available_ = false;
    }
}

//! Control active callback.
/*!
 * Callback for getting the state of the Esc base controller
 */
void OnlinePlannFramework::controlActiveCallback(const std_msgs::BoolConstPtr &control_active_msg)
{
    control_active_ = control_active_msg->data;
}

//! Navigation goal callback.
/*!
 * Callback for getting the 2D navigation goal
 */
void OnlinePlannFramework::queryGoalCallback(const geometry_msgs::PoseStampedConstPtr &query_goal_msg)
{
    double useless_pitch, useless_roll, yaw;
    yaw = tf::getYaw(tf::Quaternion(query_goal_msg->pose.orientation.x, query_goal_msg->pose.orientation.y,
                                    query_goal_msg->pose.orientation.z, query_goal_msg->pose.orientation.w));

    goal_map_frame_[0] = query_goal_msg->pose.position.x; // x
    goal_map_frame_[1] = query_goal_msg->pose.position.y; // y
    goal_map_frame_[2] = yaw;

    //=======================================================================
    // Transform from map to odom
    //=======================================================================
    ros::Time t;
    std::string err = "";
    tf::StampedTransform tf_map_to_fixed;
    tf_listener_.getLatestCommonTime("map", "odom", t, &err);
    tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);
    tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

    tf::Point goal_point_odom_frame(goal_map_frame_[0], goal_map_frame_[1], 0.0);
    goal_point_odom_frame = tf_map_to_fixed.inverse() * goal_point_odom_frame;
    goal_odom_frame_[0] = goal_point_odom_frame.getX();
    goal_odom_frame_[1] = goal_point_odom_frame.getY();
    goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;

    solution_path_states_.clear();
    goal_available_ = true;

    //=======================================================================
    // Publish RViz Maker
    //=======================================================================
    query_goal_pose_rviz_pub_.publish(query_goal_msg);

    visualization_msgs::Marker radius_msg;
    radius_msg.header.frame_id = "map";
    radius_msg.header.stamp = ros::Time::now();
    radius_msg.ns = "goal_radius";
    radius_msg.action = visualization_msgs::Marker::ADD;
    radius_msg.pose.orientation.w = 1.0;
    radius_msg.id = 0;
    radius_msg.type = visualization_msgs::Marker::CYLINDER;
    radius_msg.scale.x = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.y = 2.0 * xy_goal_tolerance_;
    radius_msg.scale.z = 0.02;
    radius_msg.color.r = 1.0;
    radius_msg.color.a = 0.5;
    radius_msg.pose.position.x = goal_map_frame_[0];
    radius_msg.pose.position.y = goal_map_frame_[1];
    radius_msg.pose.position.z = 0.0;
    query_goal_radius_rviz_pub_.publish(radius_msg);
}

//!  Planner setup.
/*!
 * Setup a sampling-based planner using OMPL.
 */
void OnlinePlannFramework::planWithSimpleSetup()
{
    //=======================================================================
    // Instantiate the state space
    //=======================================================================
    ob::StateSpacePtr space = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));

    //=======================================================================
    // Set the bounds for the state space
    //=======================================================================
    ob::RealVectorBounds bounds(2);

    bounds.setLow(0, planning_bounds_x_[0]);
    bounds.setHigh(0, planning_bounds_x_[1]);
    bounds.setLow(1, planning_bounds_y_[0]);
    bounds.setHigh(1, planning_bounds_y_[1]);

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    //=======================================================================
    // Define a simple setup class
    //=======================================================================
    simple_setup_ = og::SimpleSetupPtr(new og::SimpleSetup(space));
    ob::SpaceInformationPtr si = simple_setup_->getSpaceInformation();

    //=======================================================================
    // Create a planner for the defined space
    //=======================================================================
    ob::PlannerPtr planner;
    if (planner_name_.compare("RRT") == 0)
        planner = ob::PlannerPtr(new og::RRT(si));
    if (planner_name_.compare("PRMstar") == 0)
        planner = ob::PlannerPtr(new og::PRMstar(si));
    else if (planner_name_.compare("RRTstar") == 0)
        planner = ob::PlannerPtr(new og::RRTstar(si));
    else if (planner_name_.compare("RRTstarMod") == 0)
        planner = ob::PlannerPtr(new og::RRTstarMod(si));
    else
        planner = ob::PlannerPtr(new og::RRTstar(si));

    //=======================================================================
    // Set the setup planner
    //=======================================================================
    simple_setup_->setPlanner(planner);

    //=======================================================================
    // Create a start and goal states
    //=======================================================================
    double useless_pitch, useless_roll, yaw;
    last_robot_pose_.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);
    start_state_[0] = double(last_robot_pose_.getOrigin().getX()); // x
    start_state_[1] = double(last_robot_pose_.getOrigin().getY()); // y

    // create a start state
    ob::ScopedState<> start(space);

    start[0] = double(start_state_[0]); // x
    start[1] = double(start_state_[1]); // y

    // create a goal state
    ob::ScopedState<> goal(space);

    goal[0] = double(goal_map_frame_[0]); // x
    goal[1] = double(goal_map_frame_[1]); // y
    //=======================================================================
    // Set the start and goal states
    //=======================================================================
    simple_setup_->setStartState(start);
    simple_setup_->setGoalState(goal, goal_radius_);
    // simple_setup_->getStateSpace()->setValidSegmentCountFactor(5.0);

    //=======================================================================
    // Set state validity checking for this space
    //=======================================================================
    ob::StateValidityCheckerPtr om_stat_val_check;
    om_stat_val_check = ob::StateValidityCheckerPtr(
        new GridMapStateValidityCheckerR2(simple_setup_->getSpaceInformation(), opport_collision_check_,
                                          planning_bounds_x_, planning_bounds_y_));
    simple_setup_->setStateValidityChecker(om_stat_val_check);

    //=======================================================================
    // Set optimization objective
    //=======================================================================
    if (optimization_objective_.compare("PathLength") == 0) // path length Objective
        simple_setup_->getProblemDefinition()->setOptimizationObjective(getPathLengthObjective(si));
    else if (optimization_objective_.compare("ExtendedSocialComfort") == 0)
        simple_setup_->getProblemDefinition()->setOptimizationObjective(
            getExtendedSocialComfortObjective(si, motion_cost_interpolation_));
    else
        simple_setup_->getProblemDefinition()->setOptimizationObjective(getPathLengthObjective(si));

    //=======================================================================
    // Perform setup steps for the planner
    //=======================================================================
    simple_setup_->setup();

    //=======================================================================
    // Print information
    //=======================================================================
    // planner->printProperties(//std::cout);// print planner properties
    // si->printSettings(//std::cout);// print the settings for this space

    //=======================================================================
    // Activate a timer for incremental planning
    //=======================================================================
    //	timer_ = nh_.createTimer(ros::Duration(timer_period_), &OnlinePlannFramework::planningTimerCallback,
    // this);
    //
    //	ros::spin();
    ros::Rate loop_rate(1 / (timer_period_ - solving_time_)); // 10 hz
    // goal_available_ = true;

    //	ros::AsyncSpinner spinner(4); // Use 4 threads
    //	spinner.start();
    // ros::waitForShutdown();
    while (ros::ok())
    {
        if (goal_available_)
            ROS_INFO("%s: goal available", ros::this_node::getName().c_str());
        OnlinePlannFramework::planningTimerCallback();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//!  Periodic callback to solve the query.
/*!
 * Solve the query.
 */
void OnlinePlannFramework::planningTimerCallback()
{
    if (goal_available_)
    {
        //=======================================================================
        // Transform from map to odom
        //=======================================================================
        double useless_pitch, useless_roll, yaw;
        ros::Time t;
        std::string err = "";
        tf::StampedTransform tf_map_to_fixed;
        tf_listener_.getLatestCommonTime("map", "odom", t, &err);
        tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);
        tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

        tf::Point goal_point_odom_frame(goal_map_frame_[0], goal_map_frame_[1], 0.0);
        goal_point_odom_frame = tf_map_to_fixed.inverse() * goal_point_odom_frame;
        goal_odom_frame_[0] = goal_point_odom_frame.getX();
        goal_odom_frame_[1] = goal_point_odom_frame.getY();
        goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

        if (dynamic_bounds_)
        {
            //=======================================================================
            // Set the bounds for the state space
            //=======================================================================
            ob::RealVectorBounds bounds(2);

            if (last_robot_pose_.getOrigin().getX() < goal_odom_frame_[0])
            {
                if (last_robot_pose_.getOrigin().getX() - 10.0 < planning_bounds_x_[0])
                    bounds.setLow(0, planning_bounds_x_[0]);
                else
                    bounds.setLow(0, last_robot_pose_.getOrigin().getX() - 10.0);

                if (goal_odom_frame_[0] + 5.0 > planning_bounds_x_[1])
                    bounds.setHigh(0, planning_bounds_x_[1]);
                else
                    bounds.setHigh(0, goal_odom_frame_[0] + 10.0);
            }
            else
            {
                if (last_robot_pose_.getOrigin().getX() + 10.0 > planning_bounds_x_[1])
                    bounds.setHigh(0, planning_bounds_x_[1]);
                else
                    bounds.setHigh(0, last_robot_pose_.getOrigin().getX() + 10.0);

                if (goal_odom_frame_[0] - 10.0 < planning_bounds_x_[0])
                    bounds.setLow(0, planning_bounds_x_[0]);
                else
                    bounds.setLow(0, goal_odom_frame_[0] - 10.0);
            }

            if (last_robot_pose_.getOrigin().getY() < goal_odom_frame_[1])
            {
                if (last_robot_pose_.getOrigin().getY() - 10.0 < planning_bounds_y_[0])
                    bounds.setLow(1, planning_bounds_y_[0]);
                else
                    bounds.setLow(1, last_robot_pose_.getOrigin().getY() - 10.0);

                if (goal_odom_frame_[1] + 10.0 > planning_bounds_y_[1])
                    bounds.setHigh(1, planning_bounds_y_[1]);
                else
                    bounds.setHigh(1, goal_odom_frame_[1] + 10.0);
            }
            else
            {
                if (last_robot_pose_.getOrigin().getY() + 10.0 > planning_bounds_y_[1])
                    bounds.setHigh(1, planning_bounds_y_[1]);
                else
                    bounds.setHigh(1, last_robot_pose_.getOrigin().getY() + 10.0);

                if (goal_odom_frame_[1] - 10.0 < planning_bounds_y_[0])
                    bounds.setLow(1, planning_bounds_y_[0]);
                else
                    bounds.setLow(1, goal_odom_frame_[1] - 10.0);
            }

            simple_setup_->getStateSpace()->as<ob::RealVectorStateSpace>()->setBounds(bounds);
        }
        //=======================================================================
        // Set new start state
        //=======================================================================
        simple_setup_->clearStartStates();
        ob::ScopedState<> start(simple_setup_->getSpaceInformation()->getStateSpace());
        ob::ScopedState<> goal(simple_setup_->getSpaceInformation()->getStateSpace());

        last_robot_pose_.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

        start[0] = double(last_robot_pose_.getOrigin().getX()); // x
        start[1] = double(last_robot_pose_.getOrigin().getY()); // y

        //        if (!simple_setup_->getStateValidityChecker()->isValid(start->as<ob::State>()))
        //        {
        //            std::cout << "start in collision!!!***** " << std::endl;
        //        }

        goal[0] = double(goal_odom_frame_[0]); // x
        goal[1] = double(goal_odom_frame_[1]); // y
        //======================================================================
        // Set the start and goal states
        //=======================================================================
        simple_setup_->clear();
        simple_setup_->clearStartStates();
        simple_setup_->setStartState(start);
        simple_setup_->setGoalState(goal, goal_radius_);
        //
        simple_setup_->getStateSpace()->setValidSegmentCountFactor(15.0);

        //=======================================================================
        // Set a modified sampler
        //=======================================================================
        if (reuse_last_best_solution_)
            simple_setup_->getSpaceInformation()->getStateSpace()->setStateSamplerAllocator(
                std::bind(newAllocStateSampler, std::placeholders::_1, simple_setup_->getPlanner(),
                          solution_path_states_));

        //=======================================================================
        // Set state validity checking for this space
        //=======================================================================
        ob::StateValidityCheckerPtr om_stat_val_check;
        om_stat_val_check = ob::StateValidityCheckerPtr(
            new GridMapStateValidityCheckerR2(simple_setup_->getSpaceInformation(), opport_collision_check_,
                                              planning_bounds_x_, planning_bounds_y_));
        simple_setup_->setStateValidityChecker(om_stat_val_check);

        //=======================================================================
        // Set optimization objective
        //=======================================================================
        if (optimization_objective_.compare("PathLength") == 0) // path length Objective
            simple_setup_->getProblemDefinition()->setOptimizationObjective(
                getPathLengthObjective(simple_setup_->getSpaceInformation()));
        else if (optimization_objective_.compare("ExtendedSocialComfort") == 0) // Social Comfort
            simple_setup_->getProblemDefinition()->setOptimizationObjective(
                getExtendedSocialComfortObjective(simple_setup_->getSpaceInformation(), motion_cost_interpolation_));
        else
            simple_setup_->getProblemDefinition()->setOptimizationObjective(
                getPathLengthObjective(simple_setup_->getSpaceInformation()));

        //=======================================================================
        // Attempt to solve the problem within one second of planning time
        //=======================================================================
        ob::PlannerStatus solved = simple_setup_->solve(solving_time_);

        if (solved && simple_setup_->haveExactSolutionPath())
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path

            og::PathGeometric path = simple_setup_->getSolutionPath();

            // generates varios little segments for the waypoints obtained from the planner
            path.interpolate(int(path.length() / 0.2));

            // path_planning_msgs::PathConstSpeed solution_path;
            ROS_INFO("%s:\n\tpath with cost %f has been found with simple_setup\n",
                     ros::this_node::getName().c_str(),
                     path.cost(simple_setup_->getProblemDefinition()->getOptimizationObjective()).value());

            std::vector<ob::State *> path_states;
            path_states = path.getStates();

            double distance_to_goal =
                sqrt(pow(goal_odom_frame_[0] - path_states[path_states.size() - 1]
                                                   ->as<ob::RealVectorStateSpace::StateType>()
                                                   ->values[0],
                         2.0) +
                     pow(goal_odom_frame_[1] - path_states[path_states.size() - 1]
                                                   ->as<ob::RealVectorStateSpace::StateType>()
                                                   ->values[1],
                         2.0));

            if (simple_setup_->haveExactSolutionPath() || distance_to_goal <= goal_radius_)
            {
                // path.interpolate(int(path.length() / 1.0));
                visualizeRRT(path);

                if (reuse_last_best_solution_)
                {
                    ob::StateSpacePtr space = simple_setup_->getStateSpace();
                    solution_path_states_.clear();
                    // solution_path_states_.reserve(path_states.size());
                    for (int i = path_states.size() - 1; i >= 0; i--)
                    {
                        ob::State *s = space->allocState();
                        space->copyState(s, path_states[i]);
                        solution_path_states_.push_back(s);
                    }
                }

                //=======================================================================
                // Controller
                //=======================================================================
                if (path_states.size() > 0)
                {
                    esc_move_base_msgs::Path2D solution_path_for_control;
                    for (unsigned int i = 0; i < path_states.size(); i++)
                    {
                        geometry_msgs::Pose2D p;
                        p.x = path_states[i]->as<ob::RealVectorStateSpace::StateType>()->values[0];
                        p.y = path_states[i]->as<ob::RealVectorStateSpace::StateType>()->values[1];

                        if (i == (path_states.size() - 1))
                        {
                            if (goal_available_)
                            {
                                ros::Time t;
                                std::string err = "";
                                tf::StampedTransform tf_map_to_fixed;
                                tf_listener_.getLatestCommonTime("map", "odom", t, &err);
                                tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);

                                tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

                                p.theta = goal_map_frame_[2] - yaw;
                            }
                        }
                        solution_path_for_control.waypoints.push_back(p);
                    }
                    // ROS_INFO_STREAM("complete path: " << solution_path_for_control);
                    solution_path_control_pub_.publish(solution_path_for_control);
                }
            }
            //=======================================================================
            // Clear previous solution path
            //=======================================================================
            simple_setup_->clear();
        }
        else
        {
            ROS_INFO("%s:\n\tpath has not been found\n", ros::this_node::getName().c_str());

            if (solution_path_states_.size() > 0)
            {
                std::vector<const ob::State *> solution_path_states_copy_ = solution_path_states_;

                std::reverse(solution_path_states_copy_.begin(), solution_path_states_copy_.end());
                ROS_INFO("%s:\n\tsending partial last possible path\n", ros::this_node::getName().c_str());
                esc_move_base_msgs::Path2D solution_path_for_control;
                og::PathGeometric path_visualize = og::PathGeometric(simple_setup_->getSpaceInformation());

                // adding first waypoint
                if (simple_setup_->getStateValidityChecker()->isValid(solution_path_states_copy_[0]))
                {
                    // ROS_INFO("%s:\n\tadding first waypoint\n", ros::this_node::getName().c_str());

                    geometry_msgs::Pose2D p;
                    p.x = solution_path_states_copy_[0]->as<ob::RealVectorStateSpace::StateType>()->values[0];
                    p.y = solution_path_states_copy_[0]->as<ob::RealVectorStateSpace::StateType>()->values[1];

                    if (0 == (solution_path_states_copy_.size() - 1))
                    {
                        if (goal_available_)
                        {
                            ros::Time t;
                            std::string err = "";
                            tf::StampedTransform tf_map_to_fixed;
                            tf_listener_.getLatestCommonTime("map", "odom", t, &err);
                            tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);

                            tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

                            p.theta = goal_map_frame_[2] - yaw;
                        }
                    }
                    solution_path_for_control.waypoints.push_back(p);
                    path_visualize.append(solution_path_states_copy_[0]);
                }

                // adding rest of nodes
                bool lastNode = false;

                for (unsigned int i = 0; (i < solution_path_states_copy_.size() - 1) && (!lastNode); i++)
                {
                    if (simple_setup_->getSpaceInformation()->checkMotion(solution_path_states_copy_[i],
                                                                          solution_path_states_copy_[i + 1]))
                    {
                        // ROS_INFO("%s:\n\tadding possible waypoint\n", ros::this_node::getName().c_str());

                        geometry_msgs::Pose2D p;
                        p.x = solution_path_states_copy_[i + 1]
                                  ->as<ob::RealVectorStateSpace::StateType>()
                                  ->values[0];
                        p.y = solution_path_states_copy_[i + 1]
                                  ->as<ob::RealVectorStateSpace::StateType>()
                                  ->values[1];

                        if (i == (solution_path_states_copy_.size() - 1))
                        {
                            if (goal_available_)
                            {
                                ros::Time t;
                                std::string err = "";
                                tf::StampedTransform tf_map_to_fixed;
                                tf_listener_.getLatestCommonTime("map", "odom", t, &err);
                                tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);

                                tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

                                p.theta = goal_map_frame_[2] - yaw;
                            }
                        }
                        solution_path_for_control.waypoints.push_back(p);
                        path_visualize.append(solution_path_states_copy_[i + 1]);
                    }
                    else
                    {
                        // ROS_INFO("%s:\n\tfound not possible motion\n", ros::this_node::getName().c_str());

                        double angle = atan2(solution_path_states_copy_[i + 1]
                                                     ->as<ob::RealVectorStateSpace::StateType>()
                                                     ->values[1] -
                                                 solution_path_states_copy_[i]
                                                     ->as<ob::RealVectorStateSpace::StateType>()
                                                     ->values[1],
                                             solution_path_states_copy_[i + 1]
                                                     ->as<ob::RealVectorStateSpace::StateType>()
                                                     ->values[0] -
                                                 solution_path_states_copy_[i]
                                                     ->as<ob::RealVectorStateSpace::StateType>()
                                                     ->values[0]);
                        int counter = 1;
                        while (!lastNode)
                        {
                            ob::ScopedState<> posEv(simple_setup_->getStateSpace());
                            posEv[0] = double(solution_path_states_copy_[i]
                                                  ->as<ob::RealVectorStateSpace::StateType>()
                                                  ->values[0] +
                                              counter * robot_base_radius * std::cos(angle)); // x
                            posEv[1] = double(solution_path_states_copy_[i]
                                                  ->as<ob::RealVectorStateSpace::StateType>()
                                                  ->values[1] +
                                              counter * robot_base_radius * std::sin(angle)); // y

                            if (!simple_setup_->getSpaceInformation()->checkMotion(
                                    solution_path_states_copy_[i], posEv->as<ob::State>()))
                            {
                                // ROS_INFO("%s:\n\tadding last position\n",
                                // ros::this_node::getName().c_str());
                                ob::ScopedState<> posEv(simple_setup_->getStateSpace());
                                posEv[0] = double(solution_path_states_copy_[i]
                                                      ->as<ob::RealVectorStateSpace::StateType>()
                                                      ->values[0] +
                                                  (counter - 1) * robot_base_radius * std::cos(angle)); // x
                                posEv[1] = double(solution_path_states_copy_[i]
                                                      ->as<ob::RealVectorStateSpace::StateType>()
                                                      ->values[1] +
                                                  (counter - 1) * robot_base_radius * std::sin(angle));

                                geometry_msgs::Pose2D p;
                                p.x = posEv[0];
                                p.y = posEv[1];

                                if (goal_available_)
                                {
                                    ros::Time t;
                                    std::string err = "";
                                    tf::StampedTransform tf_map_to_fixed;
                                    tf_listener_.getLatestCommonTime("map", "odom", t, &err);
                                    tf_listener_.lookupTransform("map", "odom", t, tf_map_to_fixed);

                                    tf_map_to_fixed.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

                                    p.theta = goal_map_frame_[2] - yaw;
                                }
                                lastNode = true;
                                path_visualize.append(posEv->as<ob::RealVectorStateSpace::StateType>());
                                solution_path_for_control.waypoints.push_back(p);
                            }
                            counter += 1;
                        }
                    }
                }
                // ROS_INFO("%s:\n\tpartial path sent\n", ros::this_node::getName().c_str());
                // ROS_INFO_STREAM("partial path: " << solution_path_for_control);
                visualizeRRT(path_visualize);
                solution_path_control_pub_.publish(solution_path_for_control);
                // ros::spinOnce();
                //        if (mapping_offline_)
                //            goal_available_ = false;
            }
        }
    }
}

//! Resulting path visualization.
/*!
 * Visualize resulting path.
 */
void OnlinePlannFramework::visualizeRRT(og::PathGeometric &geopath)
{
    // %Tag(MARKER_INIT)%
    tf::Quaternion orien_quat;
    visualization_msgs::Marker visual_rrt, visual_result_path;
    visual_result_path.header.frame_id = visual_rrt.header.frame_id = world_frame_;
    visual_result_path.header.stamp = visual_rrt.header.stamp = ros::Time::now();
    visual_rrt.ns = "online_planner_rrt";
    visual_result_path.ns = "online_planner_result_path";
    visual_result_path.action = visual_rrt.action = visualization_msgs::Marker::ADD;

    visual_result_path.pose.orientation.w = visual_rrt.pose.orientation.w = 1.0;
    // %EndTag(MARKER_INIT)%

    // %Tag(ID)%
    visual_rrt.id = 0;
    visual_result_path.id = 1;
    // %EndTag(ID)%

    // %Tag(TYPE)%
    visual_rrt.type = visual_result_path.type = visualization_msgs::Marker::LINE_LIST;
    // %EndTag(TYPE)%

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    visual_rrt.scale.x = 0.03;
    visual_result_path.scale.x = 0.05;
    // %EndTag(SCALE)%

    // %Tag(COLOR)%
    // Points are green
    visual_result_path.color.g = 1.0;
    visual_result_path.color.a = 1.0;

    // Line strip is blue
    visual_rrt.color.b = 1.0;
    visual_rrt.color.a = 1.0;

    const ob::RealVectorStateSpace::StateType *state_r2;

    geometry_msgs::Point p;

    ob::PlannerData planner_data(simple_setup_->getSpaceInformation());
    simple_setup_->getPlannerData(planner_data);

    std::vector<unsigned int> edgeList;
    int num_parents;
    ROS_DEBUG("%s: number of states in the tree: %d", ros::this_node::getName().c_str(),
              planner_data.numVertices());

    std_msgs::Int32 num_nodes;
    num_nodes.data = (int)planner_data.numVertices();

    num_nodes_pub_.publish(num_nodes);

    if (visualize_tree_)
    {
        for (unsigned int i = 1; i < planner_data.numVertices(); ++i)
        {
            if (planner_data.getVertex(i).getState() && planner_data.getIncomingEdges(i, edgeList) > 0)
            {
                state_r2 = planner_data.getVertex(i).getState()->as<ob::RealVectorStateSpace::StateType>();
                p.x = state_r2->values[0];
                p.y = state_r2->values[1];
                p.z = 0.1;

                visual_rrt.points.push_back(p);

                state_r2 =
                    planner_data.getVertex(edgeList[0]).getState()->as<ob::RealVectorStateSpace::StateType>();
                p.x = state_r2->values[0];
                p.y = state_r2->values[1];
                p.z = 0.1;

                visual_rrt.points.push_back(p);
            }
        }
        solution_path_rviz_pub_.publish(visual_rrt);
    }

    std::vector<ob::State *> states = geopath.getStates();
    for (uint32_t i = 0; i < geopath.getStateCount(); ++i)
    {
        // extract the component of the state and cast it to what we expect

        state_r2 = states[i]->as<ob::RealVectorStateSpace::StateType>();
        p.x = state_r2->values[0];
        p.y = state_r2->values[1];
        p.z = 0.1;

        if (i > 0)
        {
            visual_result_path.points.push_back(p);

            state_r2 = states[i - 1]->as<ob::RealVectorStateSpace::StateType>();
            p.x = state_r2->values[0];
            p.y = state_r2->values[1];
            p.z = 0.1;

            visual_result_path.points.push_back(p);
        }
    }
    solution_path_rviz_pub_.publish(visual_result_path);
}

//! Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "esc_move_base_planner");

    ROS_INFO("%s:\n\toonline planner (C++), using OMPL version %s\n", ros::this_node::getName().c_str(),
             OMPL_VERSION);
    // ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    //	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    //	   ros::console::notifyLoggerLevelsChanged();
    //	}

    OnlinePlannFramework online_planning_framework;
    online_planning_framework.planWithSimpleSetup();
    ros::spin();
    return 0;
}
