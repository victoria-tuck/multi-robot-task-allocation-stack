/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Rice University
 *  All rights reserved.
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
 *   * Neither the name of the Rice University nor the names of its
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
 *  Modified. Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 *  http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 *********************************************************************/

/* Author: Javier V. Gómez*/

#include <new_state_sampler.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

ob::StateSamplerPtr newAllocStateSampler(const ob::StateSpace *space, const ob::PlannerPtr &planner,
                                         const std::vector<const ob::State *> &start_states)
{
    NewStateSampler *sampler = new NewStateSampler(space, space->allocDefaultStateSampler());
    // std::cout << "start_states.size(): " << start_states.size() << std::endl;
    // std::cout << "bias: " << planner->as<og::RRTstarMod>()->getGoalBias() << std::endl;
    sampler->setStatesToSample(start_states);
    sampler->setPlanner(planner);
    return ob::StateSamplerPtr(sampler);
}

void NewStateSampler::setPlanner(const ob::PlannerPtr &planner)
{
    planner_ = planner;
}

void NewStateSampler::sampleUniform(ob::State *state)
{
    // std::cout << "statesToSample_.size(): " << statesToSample_.size() << std::endl;
    if (!statesToSample_.empty())
    {
        // simple_setup_->get()->getPlanner()->as<og::RRTstar>()->setGoalBias(0.0);
        planner_->as<og::RRTstar>()->setGoalBias(0.0);
        getNextSample(state);
        // std::cout << "reusing state: " << state->as<ob::ReedsSheppStateSpace::StateType>()->getX() << ", "
        // << state->as<ob::ReedsSheppStateSpace::StateType>()->getY() << std::endl; std::cout << "bias: " <<
        // planner_->as<og::RRTstar>()->getGoalBias() << std::endl;
    }
    else
    {  // if(!reusing_){
        // simple_setup_->get()->getPlanner()->as<og::RRTstar>();//->setGoalBias(0.05);
        planner_->as<og::RRTstar>()->setGoalBias(0.05);
        sampler_->sampleUniform(state);
        //		std::cout << "state: " << state->as<ob::ReedsSheppStateSpace::StateType>()->getX() << ", " <<
        //state->as<ob::ReedsSheppStateSpace::StateType>()->getY() << std::endl; std::cout << "bias: " <<
        // planner_->as<og::RRTstar>()->getGoalBias() << std::endl;
    }
    //	else
    //		std::cout << "reusing state: " << state->as<ob::RealVectorStateSpace::StateType>()->values[0] << ",
    //" << state->as<ob::RealVectorStateSpace::StateType>()->values[1] << std::endl;
}

void NewStateSampler::sampleUniformNear(ob::State *state, const ob::State *near, const double distance)
{
    if (!statesToSample_.empty())
        getNextSample(state);
    else
        sampler_->sampleUniformNear(state, near, distance);
}

void NewStateSampler::sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev)
{
    if (!statesToSample_.empty())
        getNextSample(state);
    else
        sampler_->sampleGaussian(state, mean, stdDev);
}

void NewStateSampler::setStatesToSample(const std::vector<const ob::State *> &states)
{
    // boost::mutex::scoped_lock slock(statesLock_);
    for (size_t i = 0; i < statesToSample_.size(); ++i)
        space_->freeState(statesToSample_[i]);
    statesToSample_.clear();

    statesToSample_.reserve(states.size());
    // std::cout << "state to be recovered: " << states.size() << std::endl;
    // push in reverse order, so that the states are popped in order in getNextSample()
    // for (std::vector<const ob::State *>::const_iterator st = states.begin(); st != states.end(); ++st)
    reusing_ = false;
    for (size_t i = 0; i < states.size(); i++)
    {
        reusing_ = true;
        ob::State *s = space_->allocState();
        space_->copyState(s, states[i]);
        statesToSample_.push_back(s);
        // std::cout << "recovering state: " << s->as<ob::RealVectorStateSpace::StateType>()->values[0] << ",
        // " << s->as<ob::RealVectorStateSpace::StateType>()->values[1] << std::endl;
    }
}

void NewStateSampler::getNextSample(ob::State *state)
{
    // boost::mutex::scoped_lock slock(statesLock_);
    space_->copyState(state, statesToSample_.back());
    space_->freeState(statesToSample_.back());
    statesToSample_.pop_back();
}

void NewStateSampler::clear()
{
    // boost::mutex::scoped_lock slock(statesLock_);
    for (size_t i = 0; i < statesToSample_.size(); ++i)
        space_->freeState(statesToSample_[i]);
    statesToSample_.clear();
    sampler_.reset();
}

//-------
