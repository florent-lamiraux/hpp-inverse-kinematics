//
// Copyright (c) 2025 CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/frame.hh>
#include <hpp/pinocchio/joint.hh>

namespace hpp {
namespace inverseKinematics {

using Eigen::BlockIndex;
inline BlockIndex::segments_t vectorOfBoolToIntervals(std::vector<bool>& v) {
  BlockIndex::segments_t ret;
  for (std::size_t i = 0; i < v.size(); ++i)
    if (v[i]) ret.push_back(BlockIndex::segment_t(i, 1));
  BlockIndex::shrink(ret);
  return ret;
}

// Computes the root joint of the robot arm
// joint1, joint holding the gripper (last joint of robot arm)
JointConstPtr_t root(const JointConstPtr_t& joint1) {
  pinocchio::JointIndex i = 0;
  if (joint1) i = joint1->index();
  if (i < 6) throw std::logic_error("Staubli-TX2: wrong gripper joint");
  return Joint::create(std::const_pointer_cast<Device>(joint1->robot()), i-6);
}

// This function computes the configuration variables of
//   - root and its parents up to the common ancestor with joint2,
//   - joint2 and its parents up to the common ancestor with root.
inline std::vector<bool> invKineConfVariables(const DevicePtr_t& robot, JointConstPtr_t joint1,
                                              JointConstPtr_t joint2, size_type extraDof) {
  JointConstPtr_t r(root(joint1));
  std::vector<bool> conf(robot->configSize(), false);
  while (r && r->index() != 0) {
    for (size_type i = 0; i < r->configSize(); ++i)
      conf[r->rankInConfiguration() + i] =
          !conf[r->rankInConfiguration() + i];
    hppDout(info, "Adding joint " << r->name() << " as input variable.");
    r = r->parentJoint();
  }
  while (joint2 && joint2->index() != 0) {
    for (size_type i = 0; i < joint2->configSize(); ++i)
      conf[joint2->rankInConfiguration() + i] =
          !conf[joint2->rankInConfiguration() + i];
    hppDout(info, "Adding joint2 " << joint2->name() << " as input variable.");
    joint2 = joint2->parentJoint();
  }
  conf[extraDof] = true;
  return conf;
}

// This function computes the velocity variables of
//   - root and its parents up to the common ancestor with joint2,
//   - joint2 and its parents up to the common ancestor with root.
inline std::vector<bool> invKineVelVariables(const DevicePtr_t& robot, JointConstPtr_t joint1,
					     JointConstPtr_t joint2, size_type extraDof) {
  JointConstPtr_t r(root(joint1));
  std::vector<bool> vel(robot->numberDof(), false);
  while (r && r->index() != 0) {
    for (size_type i = 0; i < r->numberDof(); ++i)
      vel[r->rankInVelocity() + i] =
          !vel[r->rankInVelocity() + i];
    hppDout(info, "Adding joint " << r->name() << " as input variable.");
    r = r->parentJoint();
  }
  while (joint2 && joint2->index() != 0) {
    for (size_type i = 0; i < joint2->numberDof(); ++i)
      vel[joint2->rankInVelocity() + i] =
          !vel[joint2->rankInVelocity() + i];
    hppDout(info, "Adding joint2 " << joint2->name() << " as input variable.");
    joint2 = joint2->parentJoint();
  }
  // The rank of the extra-dof is the rank in the configuration vector. To get the rank in the
  // velocity vector, we need to get the difference between the configuration and velocity sizes
  extraDof += robot->numberDof() - robot->configSize();
  vel[extraDof] = true;
  return vel;
}

// This function computes the configuration variables of
//   - joint1 and its parents up to the common ancestor with joint2,
//   - joint2 parent and its parents up to the common ancestor with
//     joint1.
inline BlockIndex::segments_t inputConfVariables(const DevicePtr_t& robot, JointConstPtr_t joint1,
						 JointConstPtr_t joint2, size_type extraDof) {
  std::vector<bool> conf(invKineConfVariables(robot, joint1, joint2, extraDof));
  return vectorOfBoolToIntervals(conf);
}

// This function computes the velocity variables of
//   - joint1 and its parents up to the common ancestor with joint2,
//   - joint2 parent and its parents up to the common ancestor with
//     joint1.
inline BlockIndex::segments_t inputVelVariables(const DevicePtr_t& robot, JointConstPtr_t joint1,
						JointConstPtr_t joint2, size_type extraDof) {
  std::vector<bool> conf(invKineVelVariables(robot, joint1, joint2, extraDof));
  return vectorOfBoolToIntervals(conf);
}

inline BlockIndex::segments_t outputConfVariables(JointConstPtr_t joint1) {
  JointConstPtr_t r(root(joint1));
  size_type rank = 0;
  if (r) rank = r->rankInConfiguration() + r->configSize();
  return BlockIndex::segments_t{BlockIndex::segment_t(rank,6)};
}

inline BlockIndex::segments_t outputVelVariables(JointConstPtr_t joint1) {
  JointConstPtr_t r(root(joint1));
  size_type rank = 0;
  if (r) rank = r->rankInVelocity() + r->numberDof();
  return BlockIndex::segments_t{BlockIndex::segment_t(rank,6)};
}

} // namespace inverseKinematics
} // namespace hpp
