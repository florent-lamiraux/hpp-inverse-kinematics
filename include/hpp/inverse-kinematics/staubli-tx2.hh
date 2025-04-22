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

#ifndef HPP_INVERSE_KINEMATICS_STAUBLI_TX2_HH
#define HPP_INVERSE_KINEMATICS_STAUBLI_TX2_HH

#include <hpp/inverse-kinematics/fwd.hh>

namespace hpp {
namespace inverseKinematics {
  /// Create a 6D grasp explicit constraint
  /// \param gripper, handle define local position and joints that are constrained,
  /// \param extraDof rank of the extra degree of freedom that determines which solution of
  ///        the inverse kinematics to return.
  /// \param baseLinkName Name of the robot arm base link (origin of the root frame),
  /// \param n name of the constraint.
  /// \note does not care about the handle mask
  ImplicitPtr_t createGrasp(const GripperPtr_t& gripper, const HandleConstPtr_t& handle,
      size_type extraDof, const std::string& baseLinkName, std::string n);
  /// Create a trivial constraint
  ImplicitPtr_t createGraspComplement(const GripperPtr_t& gripper, const HandleConstPtr_t& handle,
				      std::string n);
  /// Create a 6D pregrasp explicit constraint
  /// \param gripper, handle define local position and joints that are constrained,
  /// \param extraDof rank of the extra degree of freedom that determines which solution of
  ///        the inverse kinematics to return.
  /// \param baseLinkName Name of the robot arm base link (origin of the root frame),
  /// \param n name of the constraint.
  /// \note does not care about the handle mask
  ImplicitPtr_t createPreGrasp(const GripperPtr_t& gripper, const HandleConstPtr_t& handle,
      size_type extraDof, const value_type& shift, const std::string& baseLinkName, std::string n);

  
} // namespace inverseKinematics
} // namespace hpp

#endif // HPP_INVERSE_KINEMATICS_STAUBLI_TX2_HH
