/***********************************************************************/
/**                                                                    */
/** PedestrianSFMPlugin.h                                              */
/**                                                                    */
/** Copyright (c) 2022, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#ifndef GAZEBO_PLUGINS_PEDESTRIANSFMPLUGIN_HH_
#define GAZEBO_PLUGINS_PEDESTRIANSFMPLUGIN_HH_

// C++
#include <algorithm>
#include <string>
#include <vector>

// Gazebo
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

namespace gazebo {
class GZ_PLUGIN_VISIBLE PedestrianCmdVelPlugin : public ModelPlugin {
  /// \brief Constructor
public:
  PedestrianCmdVelPlugin();

  /// \brief Load the actor plugin.
  /// \param[in] _model Pointer to the parent model.
  /// \param[in] _sdf Pointer to the plugin's SDF elements.
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation Inherited.
public:
  virtual void Reset();

  /// \brief Function that is called every update cycle.
  /// \param[in] _info Timing information
private:
  void OnUpdate(const common::UpdateInfo &_info);

  // private: void InitializePedestrians();

  /// \brief Pointer to the parent actor.
private:
  physics::ActorPtr actor;

  /// \brief Pointer to the world, for convenience.
private:
  physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
private:
  sdf::ElementPtr sdf;

  /// \brief Velocity of the actor
private:
  ignition::math::Vector3d velocity;

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
private:
  double animationFactor = 1.0;

  /// \brief Time of the last update.
private:
  common::Time lastUpdate;

  /// \brief Animation name of this actor
private:
  std::string animationName;

  /// \brief Custom trajectory info.
private:
  physics::TrajectoryInfoPtr trajectoryInfo;
};
} // namespace gazebo
#endif
