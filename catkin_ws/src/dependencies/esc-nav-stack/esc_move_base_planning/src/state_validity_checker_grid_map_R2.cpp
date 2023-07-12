/*! \file state_validity_checker_grid_map_R2.cpp
 * \brief State validity checker.
 *
 * \date November 07, 2022
 * \author Juan David Hernandez Vega, HernandezVegaJ@cardiff.ac.uk
 * \author Steven Alexander Silva Mendoza, silvas1@cardiff.ac.uk
 *
 * \details Check is a given configuration R2 is collision-free.
 *  The workspace is represented by an GridMap and collision check is done by iterating.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#include <state_validity_checker_grid_map_R2.h>

GridMapStateValidityCheckerR2::GridMapStateValidityCheckerR2(const ob::SpaceInformationPtr &si,
                                                             const bool opport_collision_check,
                                                             std::vector<double> planning_bounds_x,
                                                             std::vector<double> planning_bounds_y)
    : ob::StateValidityChecker(si), local_nh_("~"), robot_base_radius_(0.4)
{
    GetGridMap::Request req;
    GetGridMap::Response resp;

    opport_collision_check_ = opport_collision_check;
    planning_bounds_x_ = planning_bounds_x;
    planning_bounds_y_ = planning_bounds_y;

    local_nh_.param("robot_base_radius", robot_base_radius_, robot_base_radius_);
    local_nh_.param("grid_map_service", grid_map_service_, grid_map_service_);

    // ! GRID MAP REQUEST

    ROS_DEBUG("%s: requesting the map to %s...", ros::this_node::getName().c_str(),
              nh_.resolveName(grid_map_service_).c_str());

    ros::service::call(grid_map_service_, req, resp);

    if (grid_map::GridMapRosConverter::fromMessage(resp.map, grid_map_))
    {
        ROS_DEBUG("Obtained gridmap successfully");
        grid_map_msgs_ = resp.map;

        grid_map_max_x_ = grid_map_msgs_.info.pose.position.x + (grid_map_msgs_.info.length_x / 2);
        grid_map_min_x_ = grid_map_msgs_.info.pose.position.x - (grid_map_msgs_.info.length_x / 2);

        grid_map_max_y_ = grid_map_msgs_.info.pose.position.y + (grid_map_msgs_.info.length_y / 2);
        grid_map_min_y_ = grid_map_msgs_.info.pose.position.y - (grid_map_msgs_.info.length_y / 2);
    }
    else
    {
        ROS_ERROR("Error reading GridMap");
    }

    try
    {
        obstacles_grid_map_ = grid_map_["full"];
        comfort_grid_map_ = grid_map_["comfort"];
    }
    catch (...)
    {
    }
}

bool GridMapStateValidityCheckerR2::isValid(const ob::State *state) const
{
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    // ompl::tools::Profiler::Begin("collision");

    // extract the component of the state and cast it to what we expect

    if (opport_collision_check_ &&
        (state_r2->values[0] < grid_map_min_x_ || state_r2->values[1] < grid_map_min_y_ ||
         state_r2->values[0] > grid_map_max_x_ || state_r2->values[1] > grid_map_max_y_))
    {
        // ompl::tools::Profiler::End("collision");
        return true;
    }

    if (state_r2->values[0] < planning_bounds_x_[0] || state_r2->values[1] < planning_bounds_y_[0] ||
        state_r2->values[0] > planning_bounds_x_[1] || state_r2->values[1] > planning_bounds_y_[1])
    {
        // ompl::tools::Profiler::End("collision");
        return false;
    }

    grid_map::Position query(state_r2->values[0], state_r2->values[1]);

    for (grid_map::CircleIterator iterator(grid_map_, query, robot_base_radius_);
         !iterator.isPastEnd(); ++iterator)
    {
        const grid_map::Index index(*iterator);

        if (obstacles_grid_map_(index(0), index(1)) > 50)
        {
            return false;
        }
    }

    return true;
}

double GridMapStateValidityCheckerR2::checkExtendedSocialComfort(const ob::State *state,
                                                                 const ob::SpaceInformationPtr space) const
{
    double state_risk = 0.0;

    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    grid_map::Position query(state_r2->values[0], state_r2->values[1]);

    grid_map::Index index;

    if (grid_map_.getIndex(query, index))
    {
        state_risk = comfort_grid_map_(index(0), index(1));
    }

    if (state_risk < 1 || isnan(state_risk))
    {
        state_risk = 1;
    }

    return state_risk;
}

bool GridMapStateValidityCheckerR2::isValidPoint(const ob::State *state) const
{

    // extract the component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    grid_map::Position query(state_r2->values[0], state_r2->values[1]);

    grid_map::Index index;

    if (grid_map_.getIndex(query, index))
    {
        if (obstacles_grid_map_(index(0), index(1)) > 50)
        {
            return false;
        }
    }

    return true;
}

GridMapStateValidityCheckerR2::~GridMapStateValidityCheckerR2()
{
}
