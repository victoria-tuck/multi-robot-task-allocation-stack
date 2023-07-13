/*/
 * Copyright (c) 2020 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Harmish Khambhaita, Phani Teja Singamaneni
 */

#ifndef EDGE_AGENT_AGENT_SAFETY_H_
#define EDGE_AGENT_AGENT_SAFETY_H_

#include <hateb_local_planner/g2o_types/penalties.h>
#include <hateb_local_planner/g2o_types/vertex_pose.h>
#include <hateb_local_planner/hateb_config.h>
#include <hateb_local_planner/g2o_types/base_teb_edges.h>

// #include "g2o/core/base_unary_edge.h"

namespace hateb_local_planner {

class EdgeAgentAgentSafety : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose>
{
public:
  EdgeAgentAgentSafety()
  {
    this->setMeasurement(0.);
  }

  // virtual ~EdgeAgentAgentSafety() {
  //   for (unsigned int i = 0; i < 2; i++) {
  //     if (_vertices[i])
  //       _vertices[i]->edges().erase(this);
  //   }
  // }

  void computeError() {
    ROS_ASSERT_MSG(cfg_ &&  agent_radius_ < std::numeric_limits<double>::infinity(), "You must call setParameters() on EdgeAgentAgentSafety()");
    const VertexPose *agent1_bandpt = static_cast<const VertexPose *>(_vertices[0]);
    const VertexPose *agent2_bandpt = static_cast<const VertexPose *>(_vertices[1]);

    double dist = std::hypot(agent1_bandpt->x() - agent2_bandpt->x(), agent1_bandpt->y() - agent2_bandpt->y()) - (2 * agent_radius_);

    ROS_DEBUG_THROTTLE(0.5, "agent agent external dist = %f", dist);
    _error[0] = penaltyBoundFromBelowQuad(dist,  cfg_->hateb.min_agent_agent_dist, cfg_->optim.penalty_epsilon);


    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAgentAgentSafety::computeError() _error[0]=%f\n", _error[0]);
  }

  void setAgentRadius(const double agent_radius) {
    agent_radius_ = agent_radius;
  }

  void setParameters(const HATebConfig &cfg, const double agent_radius) {
    cfg_ = &cfg;
    agent_radius_ = agent_radius;
  }

protected:
  double agent_radius_ = std::numeric_limits<double>::infinity();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}; // end namespace

#endif // EDGE_AGENT_AGENT_SAFETY_H_
