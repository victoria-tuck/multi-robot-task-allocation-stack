/*********************************************************************
 *
 * Copyright (c) 2022 LAAS/CNRS
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
 * Author: Phani Teja Singamaneni
 *********************************************************************/

#ifndef EDGE_INVISIBLEHUMAN_VELOCITY_H
#define EDGE_INVISIBLEHUMAN_VELOCITY_H

#include <hateb_local_planner/g2o_types/vertex_pose.h>
#include <hateb_local_planner/g2o_types/vertex_timediff.h>
#include <hateb_local_planner/g2o_types/penalties.h>
#include <hateb_local_planner/g2o_types/base_teb_edges.h>
#include <hateb_local_planner/obstacles.h>
#include <hateb_local_planner/hateb_config.h>
#include <hateb_local_planner/robot_footprint_model.h>

namespace hateb_local_planner
{

class EdgeInvisibleHumanVelocity : public BaseTebMultiEdge<1, const Obstacle*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeInvisibleHumanVelocity() : t_(0)
  {
    this->resize(3);
  }

  EdgeInvisibleHumanVelocity(double t) : t_(t)
  {
    this->resize(3);
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setHATebConfig(), setObstacle() and setRobotModel() on EdgeInvisibleHuman()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose *bandpt_nxt = static_cast<const VertexPose *>(_vertices[1]);
    const VertexTimeDiff *dt_ = static_cast<const VertexTimeDiff *>(_vertices[2]);


    double dist = robot_model_->calculateDistance(bandpt->pose(), _measurement);
    auto robot_vel = (bandpt_nxt->position() - bandpt->position())/dt_->dt();

    double cost = V_i/dist;
    if(t_ > 0.5) //Accounting for human reaction time
      cost = (std::max(V_i - a_norm*t_, 0.0)+ robot_vel.norm() + 1.0)/dist;

    _error[0] = penaltyBoundFromAbove(cost, cfg_->hateb.invisible_human_threshold, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeInvisibleHuman::computeError() _error[0]=%f\n",_error[0]);
  }

  /**
   * @brief Set Obstacle for the underlying cost function
   * @param obstacle Const pointer to an Obstacle or derived Obstacle
   */
  void setObstacle(const Obstacle* obstacle)
  {
    _measurement = obstacle;
  }

  /**
   * @brief Set pointer to the robot model
   * @param robot_model Robot model required for distance calculation
   */
  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg HATebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setParameters(const HATebConfig& cfg, const BaseRobotFootprintModel* robot_model, const Obstacle* obstacle)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = obstacle;
  }

protected:

  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  double t_; //!< Estimated time until current pose is reached
  double V_i = 1.5;
  double a_min = 0.1;
  double a_norm = 0.68;
  double a_max = 2.94; // 0.3g

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};




} // end namespace

#endif
