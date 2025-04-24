// Copyright (c) 2025 CNRS
// Author: Florent Lamiraux
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
#include <hpp/inverse-kinematics/config.hh>
#include <hpp/corbaserver/server-plugin.hh>

namespace hpp{
namespace inverse_kinematics{
namespace staubli_tx2 {

class HPP_INVERSE_KINEMATICS_DLLAPI Server : public corbaServer::ServerPlugin
{
public:
  Server(corbaServer::Server* parent);
    ~Server();
  std::string name() const;
  manipulation::ProblemSolverPtr_t problemSolver();
  /// Call hpp::corba::Server <impl::BinPicking>::startCorbaServer
  void startCorbaServer(const std::string& contextId,
                        const std::string& contextKind);
  ::CORBA::Object_ptr servant(const std::string& name) const;
private:
  corba::Server<impl::StaubliTx2>* staubliTx2Impl_;

}; // class Server
} // namespace staubli_tx2
} // namespace inverse_kinematics
} // name hpp

