/*! \file state_cost_objective.hpp
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

#ifndef OMPL_CONTRIB_STATE_COST_OBJECTIVES_
#define OMPL_CONTRIB_STATE_COST_OBJECTIVES_

// OMPL
#include <ompl/config.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>

#include <state_validity_checker_grid_map_R2.h>

namespace ob = ompl::base;

class ExtendedSocialComfortObjective : public ob::StateCostIntegralObjective
{
private:
public:
    ExtendedSocialComfortObjective(const ob::SpaceInformationPtr &si, bool enableMotionCostInterpolation)
        : ob::StateCostIntegralObjective(si, enableMotionCostInterpolation)
    {
        // ROS_INFO_STREAM("running extended social comfort objective checker");
    }

    // in case of modifying the cost calculation function here it is where it should be done

    ob::Cost stateCost(const ob::State *s) const
    {
        std::shared_ptr<GridMapStateValidityCheckerR2> state_vality_checker =
            std::static_pointer_cast<GridMapStateValidityCheckerR2>(si_->getStateValidityChecker());
        return ob::Cost(state_vality_checker->checkExtendedSocialComfort(s, si_));
    }

    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const
    {
        if (interpolateMotionCost_)
        {
            ob::Cost totalCost = this->identityCost();

            int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
            // nd = int(nd/10);

            ob::State *test1 = si_->cloneState(s1);
            ob::Cost prevStateCost = this->stateCost(test1);
            if (nd > 1)
            {
                ob::State *test2 = si_->allocState();
                for (int j = 1; j < nd; ++j)
                {
                    si_->getStateSpace()->interpolate(s1, s2, (double)j / (double)nd, test2);
                    ob::Cost nextStateCost = this->stateCost(test2);
                    totalCost = ob::Cost(
                        totalCost.value() +
                        this->trapezoid(prevStateCost, nextStateCost, si_->distance(test1, test2)).value());
                    std::swap(test1, test2);
                    prevStateCost = nextStateCost;
                }
                si_->freeState(test2);
            }

            // Lastly, add s2
            totalCost = ob::Cost(
                totalCost.value() +
                this->trapezoid(prevStateCost, this->stateCost(s2), si_->distance(test1, s2)).value());

            si_->freeState(test1);

            // ROS_INFO_STREAM("Total cost: " << totalCost);

            return totalCost;
        }
        else
        {
            // ROS_INFO_STREAM("Trapezoid cost: " << this->trapezoid(this->stateCost(s1), this->stateCost(s2),
            //                                                       si_->distance(s1, s2)));
            return this->trapezoid(this->stateCost(s1), this->stateCost(s2), si_->distance(s1, s2));
        }
    }
};

/** Return an optimization objective which attempts to steer the robot
    away from social agents in order to keep comfort with an extended social comfort model. */
ob::OptimizationObjectivePtr getExtendedSocialComfortObjective(const ob::SpaceInformationPtr &si,
                                                               bool motion_cost_interpolation);

/** Returns a structure representing the optimization objective to use
    for optimal motion planning. This method returns an objective
    which attempts to minimize the length in configuration space of
    computed paths. */
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr &si);

#endif
