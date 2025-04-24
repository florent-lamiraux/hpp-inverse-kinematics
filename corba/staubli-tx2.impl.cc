// Copyright (c) 2025, LAAS-CNRS
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

#include <../corba/staubli-tx2.impl.hh>
#include <../corba/staubli-tx2.hh>

#include <pinocchio/multibody/frame.hpp>

#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/liegroup-space.hh>

#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>
#include <hpp/manipulation/problem-solver.hh>

#include <hpp/inverse-kinematics/staubli-tx2.hh>

namespace hpp {
namespace inverse_kinematics {
namespace staubli_tx2 {
namespace impl {

using constraints::ImplicitPtr_t;
using manipulation::ConstraintAndComplement_t;
using manipulation::HandlePtr_t;
using pinocchio::GripperPtr_t;
using pinocchio::size_type;
  
manipulation::DevicePtr_t StaubliTx2::getRobotOrThrow()
{
  manipulation::DevicePtr_t robot(problemSolver()->robot());
  if (!robot){
    throw std::runtime_error("No robot has been loaded.");
  }
  return robot;
}

StaubliTx2::StaubliTx2() : server_(0x0) {}

hpp::manipulation::ProblemSolverPtr_t StaubliTx2::problemSolver()
{
  return server_->problemSolver();
}

void StaubliTx2::createGrasp(const char* graspName, const char* gripperName, const char* handleName,
			     const char* baseLinkName, ::CORBA::Long extraDof)
{
  try{
    manipulation::DevicePtr_t robot(getRobotOrThrow());
    GripperPtr_t g = robot->grippers.get(gripperName, GripperPtr_t());
    if (!g) throw std::runtime_error("No gripper with name " + std::string(gripperName) + ".");
    HandlePtr_t h = robot->handles.get(handleName, HandlePtr_t());
    if (!h) throw std::runtime_error("No handle with name " + std::string(handleName) + ".");
    const std::string cname = std::string(graspName) + "/complement";
    const std::string bname = std::string(graspName) + "/hold";
    ImplicitPtr_t constraint(hpp::inverseKinematics::createGrasp(g, h, (size_type) extraDof,
								 baseLinkName, graspName));
    ImplicitPtr_t complement(hpp::inverseKinematics::createGraspComplement(g, h, cname));
    ImplicitPtr_t both(constraint);
    problemSolver()->addNumericalConstraint(graspName, constraint);
    problemSolver()->addNumericalConstraint(cname, complement);
    problemSolver()->addNumericalConstraint(bname, both);

    problemSolver()->constraintsAndComplements.push_back(
        ConstraintAndComplement_t(constraint, complement, both));

  } catch (const std::exception& exc){
    throw Error(exc.what());
  }
}

void StaubliTx2::createPreGrasp(const char* pregraspName, const char* gripperName,
    const char* handleName, const char* baseLinkName, ::CORBA::Long extraDof) {
  try {
    manipulation::DevicePtr_t robot(getRobotOrThrow());
    GripperPtr_t g = robot->grippers.get(gripperName, GripperPtr_t());
    if (!g) throw std::runtime_error("No gripper with name " + std::string(gripperName) + ".");
    HandlePtr_t h = robot->handles.get(handleName, HandlePtr_t());
    if (!h) throw std::runtime_error("No handle with name " + std::string(handleName) + ".");

    value_type c = h->clearance() + g->clearance();
    ImplicitPtr_t constraint(hpp::inverseKinematics::createPreGrasp(g, h, (size_type) extraDof, c,
								    baseLinkName, pregraspName));
    problemSolver()->addNumericalConstraint(pregraspName, constraint);
  } catch (const std::exception& exc){
    throw Error(exc.what());
  }
}

} // namespace impl
} // namespace staubli_tx2
} // namespace inverse_kinematics
} // namespace hpp
