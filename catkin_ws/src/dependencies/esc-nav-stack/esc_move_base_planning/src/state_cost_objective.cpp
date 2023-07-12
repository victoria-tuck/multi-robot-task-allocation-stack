/*! \file state_cost_objective.cpp
 * \brief State cost objective.
 *
 * \date Mar 31, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
 *
 * \details State cost objective. Define different state cost objectives.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#include <state_cost_objective.h>

ob::OptimizationObjectivePtr getExtendedSocialComfortObjective(const ob::SpaceInformationPtr &si,
                                                               bool motion_cost_interpolation)
{
    // ROS_INFO_STREAM("Sending extended social comfort objective");
    return ob::OptimizationObjectivePtr(new ExtendedSocialComfortObjective(si, motion_cost_interpolation));
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr &si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}
