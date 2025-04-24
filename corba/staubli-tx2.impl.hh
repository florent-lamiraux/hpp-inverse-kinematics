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

#ifndef HPP_INVERSE_KINEMATICS_CORBA_STAUBLI_TX2_IMPL_HH
#define HPP_INVERSE_KINEMATICS_CORBA_STAUBLI_TX2_IMPL_HH

#include <hpp/inverse-kinematics/fwd.hh>
#include <corba/staubli-tx2-idl.hh>
#include <hpp/manipulation/fwd.hh>

namespace hpp {
namespace inverse_kinematics {
namespace staubli_tx2 {
class Server;
namespace impl {

using pinocchio::DevicePtr_t;

class StaubliTx2 : public virtual POA_hpp::corbaserver::inverse_kinematics::StaubliTx2
{
public:
  StaubliTx2();
  void setServer(Server* server) { server_ = server; }

  virtual void createGrasp(const char* graspName, const char* gripperName, const char* handleName,
      const char* baseLinkName, ::CORBA::Long extraDof);
  virtual void createPreGrasp(const char* graspName, const char* gripperName,
      const char* handleName, const char* baseLinkName, ::CORBA::Long extraDof);
private:
  manipulation::ProblemSolverPtr_t problemSolver();
  manipulation::DevicePtr_t getRobotOrThrow();
  Server* server_;
}; // class StaubliTx2
} // namespace impl
} // namespace staubli_tx2
} // namespace inverse_kinematics
} // namespace hpp

#endif // HPP_INVERSE_KINEMATICS_CORBA_STAUBLI_TX2_IMPL_HH
