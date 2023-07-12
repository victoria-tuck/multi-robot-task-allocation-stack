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
 *  Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 *  http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 *********************************************************************/

/* Author: Javier V. Gómez */

#ifndef OMPL_BASE_SAMPLERS_NEW_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_NEW_STATE_SAMPLER_

#include <ompl/base/StateSpace.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

// Eigen
#include <Eigen/Dense>

// OMPL namespaces
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

ob::StateSamplerPtr newAllocStateSampler(const ob::StateSpace *space, const ob::PlannerPtr &planner,
                                         const std::vector<const ob::State *> &start_states);

/** \brief Extended state sampler to use with the CForest planning algorithm. It wraps the user-specified
    state sampler.*/
class NewStateSampler : public ob::StateSampler
{
public:
    /** \brief Constructor */
    NewStateSampler(const ob::StateSpace *space, ob::StateSamplerPtr sampler)
      : ob::StateSampler(space), sampler_(sampler)
    {
    }

    /** \brief Destructor */
    ~NewStateSampler()
    {
        clear();
    }

    /** \brief It will sample the next state of the vector StatesToSample_. If this is empty,
        it will call the sampleUniform() method of the specified sampler. */
    virtual void sampleUniform(ob::State *state);

    /** \brief It will sample the next state of the vector StatesToSample_. If this is empty,
        it will call the sampleUniformNear() method of the specified sampler. */
    virtual void sampleUniformNear(ob::State *state, const ob::State *near, const double distance);

    /** \brief It will sample the next state of the vector StatesToSample_. If this is empty,
        it will call the sampleGaussian() method of the specified sampler. */
    virtual void sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev);

    const ob::StateSpace *getStateSpace() const
    {
        return space_;
    }

    /** \brief Fills the vector StatesToSample_ of states to be sampled in the next
        calls to sampleUniform(), sampleUniformNear() or sampleGaussian(). */
    void setStatesToSample(const std::vector<const ob::State *> &states);

    void setPlanner(const ob::PlannerPtr &planner);

    void clear();

protected:
    /** \brief Extracts the next sample when statesToSample_ is not empty. */
    void getNextSample(ob::State *state);

    /** \brief States to be sampled */
    std::vector<ob::State *> statesToSample_;

    /** \brief Underlying, user-specified state sampler. */
    ob::StateSamplerPtr sampler_;

    ob::PlannerPtr planner_;

    /** \brief Lock to control the access to the statesToSample_ vector. */
    // boost::mutex statesLock_;
    bool reusing_;
};

//------

oc::ControlSamplerPtr newAllocControlSampler(const oc::ControlSpace *cspace,
                                             const std::vector<const oc::Control *> &start_controls);

/** \brief Extended state sampler to use with the CForest planning algorithm. It wraps the user-specified
    state sampler.*/
class NewControlSampler : public oc::ControlSampler
{
public:
    /** \brief Constructor */
    NewControlSampler(const oc::ControlSpace *cspace, oc::ControlSamplerPtr control_sampler)
      : oc::ControlSampler(cspace), csampler_(control_sampler)
    {
    }

    /** \brief Destructor */
    ~NewControlSampler()
    {
        clear();
    }

    /** \brief Sample a control. All other control sampling functions default to this one,
        unless a user-specified implementation is given. */
    virtual void sample(oc::Control *control);

    /** \brief Sample a control, given it is applied to a specific state (state). The default
        implementation calls the previous definition of sample(). Providing a different
        implementation of this function is useful if, for example, the sampling of controls
        depends on the state of the system. When attempting to sample controls that keep a
        system stable, for example, knowing the state at which the control is applied is important. */
    virtual void sample(oc::Control *control, const ob::State *state);

    /** \brief Sample a control, given the previously applied control. The default implementation
        calls the first definition of sample(). For some systems it is possible that large changes
        in controls are not desirable. For example, switching from maximum acceleration to maximum
        deceleration is not desirable when driving a car. */
    virtual void sampleNext(oc::Control *control, const oc::Control *previous);

    /** \brief Sample a control, given the previously applied control and that it is applied to a
        specific state. The default implementation calls the first definition of sample(), even if
        other implementations of the sampleNext() shown above are provided. Often this function needs
        to be overridden as it is the function planners typically call. */
    virtual void sampleNext(oc::Control *control, const oc::Control *previous, const ob::State *state);

    /** \brief Fills the vector ControlsToSample_ of controls to be sampled in the next
        calls to sample(), sampleNext() or sampleGaussian(). */
    void setControlsToSample(const std::vector<const oc::Control *> &controls);

    void clear();

protected:
    /** \brief Extracts the next sample when controlsToSample_ is not empty. */
    void getNextControl(oc::Control *control);

    /** \brief Controls to be sampled */
    std::vector<oc::Control *> controlsToSample_;

    /** \brief Underlying, user-specified control sampler. */
    oc::ControlSamplerPtr csampler_;

    /** \brief Lock to control the access to the statesToSample_ vector. */
    // boost::mutex statesLock_;
};

//------

oc::DirectedControlSamplerPtr newAllocDirectedControlSampler(const oc::SpaceInformation *si);

class NewDirectedControlSampler : public oc::DirectedControlSampler
{
public:
    NewDirectedControlSampler(const oc::SpaceInformation *si);

    virtual ~NewDirectedControlSampler();

    unsigned int getNumControlSamples() const
    {
        return numControlSamples_;
    }

    void setNumControlSamples(unsigned int numSamples)
    {
        numControlSamples_ = numSamples;
    }

    virtual unsigned int sampleTo(oc::Control *control, const ob::State *source, ob::State *dest);

    virtual unsigned int sampleTo(oc::Control *control, const oc::Control *previous, const ob::State *source,
                                  ob::State *dest);

protected:
    virtual unsigned int getBestControl(oc::Control *control, const ob::State *source, ob::State *dest,
                                        const oc::Control *previous);
    unsigned int propagateWhileValid(const ob::State *state, const ob::State *dest, int steps,
                                     ob::State *result) const;

    oc::ControlSamplerPtr cs_;

    unsigned int numControlSamples_;
};

void DynUnicycle(const ob::State *start, const ob::State *desired, const double duration, ob::State *result);

#endif
