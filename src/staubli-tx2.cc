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


#include <cstdlib>
#include <hpp/constraints/explicit.hh>
#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/matrix-view.hh>
#include <hpp/inverse-kinematics/staubli-tx2.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/util/debug.hh>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <../src/configuration-variables.hh>


namespace hpp {
namespace inverseKinematics {
namespace staubliTx2 {

typedef constraints::FunctionNotDefinedForThisValue NoSolution;

inline matrix3_t cross(const vector3_t& u) {
  matrix3_t res;
  res.setZero();
  res(0,1) = -u(2);
  res(0,2) =  u(1);
  res(1,0) =  u(2);
  res(1,2) = -u(0);
  res(2,0) = -u(1);
  res(2,1) =  u(0);
  return res;
}

class InverseKinematics : public DifferentiableFunction {
public:
  typedef shared_ptr<InverseKinematics> Ptr_t;
  typedef weak_ptr<InverseKinematics> WkPtr_t;
  static Ptr_t create(const std::string& name, const DevicePtr_t& robot,
		      const JointConstPtr_t& joint1, const JointConstPtr_t& joint2,
		      const Transform3s& frame1, const Transform3s& frame2,
		      const std::string& baseLinkName, const segments_t inConf,
		      const segments_t outConf, const segments_t inVel) {
    return Ptr_t(new InverseKinematics(name, robot, joint1, joint2, frame1, frame2, baseLinkName,
				       inConf, outConf, inVel));
  }
protected:
  InverseKinematics(const std::string& name, const DevicePtr_t& robot,
		    const JointConstPtr_t& joint1, const JointConstPtr_t& joint2,
		    const Transform3s& frame1, const Transform3s& frame2,
		    const std::string& baseLinkName, const segments_t inConf,
		    const segments_t outConf, const segments_t inVel) :
    DifferentiableFunction(BlockIndex::cardinal(inConf), BlockIndex::cardinal(inVel),
			   LiegroupSpace::Rn(6), name), robot_(robot), joint1_ (joint1),
    joint2_ (joint2), frame1_ (frame1), frame2_ (frame2), baseLinkIdx_(
      robot->model().getFrameId(baseLinkName)),
    rootIdx_(robot->model().frames[baseLinkIdx_].parentJoint),
    inConf_(inConf), inVel_(inVel), outConf_(outConf),
    nq_(robot->model().nq), data_(robot->model()) {
    hppDout(info, "baseLinkName=" << baseLinkName);
    hppDout(info, "baseLinkIdx_=" << baseLinkIdx_);
    q_.resize(robot->configSize());
    ::pinocchio::neutral(robot->model(), q_.head(nq_));
  }

  virtual void impl_compute(LiegroupElementRef result, vectorIn_t argument) const {
    forwardKinematics(argument);
    int extraDof = (int) argument.tail(1)[0];
    // pose of the robot root joint
    Transform3s _0Mb(data_.oMf[baseLinkIdx_]);
    // pose of joint1: J2 * F2 * F1^{-1}
    Transform3s joint1Pose(data_.oMi[joint2_->index()] * frame2_ * frame1_.inverse());
    // pose of last joint in robot arm base_link
    Transform3s Minput(_0Mb.inverse() * joint1Pose);
    hppDout(info, "_0Mb=" << _0Mb);
    hppDout(info, "joint1Pose=" << joint1Pose);
    hppDout(info, "Minput=" << Minput);
    // Get joint limits
    int joint1Iq = robot_->model().joints[joint1_->index()].idx_q();
    double q1_min = robot_->model().lowerPositionLimit[joint1Iq - 6+1];
    double q2_min = robot_->model().lowerPositionLimit[joint1Iq - 6+2];
    double q3_min = robot_->model().lowerPositionLimit[joint1Iq - 6+3];
    double q4_min = robot_->model().lowerPositionLimit[joint1Iq - 6+4];
    double q5_min = robot_->model().lowerPositionLimit[joint1Iq - 6+5];
    double q6_min = robot_->model().lowerPositionLimit[joint1Iq - 6+6];
    double q1_max = robot_->model().upperPositionLimit[joint1Iq - 6+1];
    double q2_max = robot_->model().upperPositionLimit[joint1Iq - 6+2];
    double q3_max = robot_->model().upperPositionLimit[joint1Iq - 6+3];
    double q4_max = robot_->model().upperPositionLimit[joint1Iq - 6+4];
    double q5_max = robot_->model().upperPositionLimit[joint1Iq - 6+5];
    double q6_max = robot_->model().upperPositionLimit[joint1Iq - 6+6];
    hppDout(info, "joint1Iq=" << joint1Iq);

    // Computation of the indices of the solution for each configuration variable
    std::div_t res = div(extraDof, 4);
    int i1 = res.rem;
    int i = (extraDof - i1)/4;
    res = div(i, 2);
    int i3 = res.rem;
    i = (i - i3)/2;
    res = div(i, 2);
    int i5 = res.rem;
    i = (i - i5)/2;
    res = div(i, 3);
    int i4 = res.rem;
    int i6 = res.quot;
    hppDout(info, "extraDof=" << extraDof);
    hppDout(info, "i1=" << i1);
    hppDout(info, "i3=" << i3);
    hppDout(info, "i5=" << i5);
    hppDout(info, "i4=" << i4);
    hppDout(info, "i6=" << i6);
    
    constexpr double pi = 3.14159265358979323846;

    double q1, q2, q3, q4, q5, q6;

    double r1 = 0.478;
    double r2 = 0.050;
    double r3 = 0.050;
    double r4 = 0.425;
    double r5 = 0.425;
    double r6 = 0.1;

    vector3_t offset; offset << 0, 0, -r6;
    vector3_t concurrentPoint = Minput.act(offset);

    hppDout(info, "concurrentPoint = " << concurrentPoint.transpose());
    double X = concurrentPoint(0);
    double Y = concurrentPoint(1);
    double Z = concurrentPoint(2);

    double rho = sqrt(X*X+Y*Y);
    if (rho < r3) throw NoSolution();
    double theta = atan2(Y, X);

    switch (i1) {
    case 0:
      q1 = theta - asin(r3/rho);
      break;
    case 1:
      q1 = theta - asin(r3/rho) + 2*pi;
      break;
    case 2:
      q1 = theta + pi + asin(r3/rho);
      break;
    case 3:
      q1 = theta - pi + asin(r3/rho);
      break;
    default:
      abort();
    }
    if ((q1_min > q1) || (q1 > q1_max)) throw NoSolution();
    double c1 = cos(q1), s1 = sin(q1);
    double x = c1*X + s1*Y;

    switch(i3) {
    case 0:
      q3 = acos(((x-r2)*(x-r2) + (Z-r1)*(Z-r1) - r4*r4 - r5*r5)/(2*r4*r5));
      break;
    case 1:
      q3 = -acos(((x-r2)*(x-r2) + (Z-r1)*(Z-r1) - r4*r4 - r5*r5)/(2*r4*r5));
      break;
    default:
      abort();
    }
    if (std::isnan(q3) || (q3_min > q3) || (q3 > q3_max)) throw NoSolution();
    double c3 = cos(q3), s3 = sin(q3);

    q2 = atan2((r5*c3+r4)*(x-r2)-r5*s3*(Z-r1),r5*s3*(x-r2)+(r5*c3+r4)*(Z-r1));
    if ((q2_min > q2) || (q2 > q2_max)) throw NoSolution();
    double c2 = cos(q2), s2 = sin(q2);

    // assert(fabs(r5*c1*s23 - r3*s1 + r4*c1*s2 + r2*c1 - X) < 1e-8);
    // assert(fabs(r5*s1*s23 + r3*c1 + r4*s1*s2 + r2*s1 - Y) < 1e-8);
    assert(fabs(r5*(c2*c3 - s2*s3) + r4*c2 + r1 - Z) < 1e-8);
    assert(fabs(r5*(s2*c3 + c2*s3) + r4*s2 + r2 - x) < 1e-8);
    assert(fabs(c1*Y - s1*X-r3) < 1e-8);
    assert(fabs(X - rho*cos(theta)) < 1e-8);
    assert(fabs(Y - rho*sin(theta)) < 1e-8);
    assert(fabs(r3 - rho*(c1*sin(theta)-cos(theta)*s1)) < 1e-8);
    assert(fabs(sin(theta - q1) - r3/rho) < 1e-8);
    matrix3_t R03;
    R03 <<
      c1*c2*c3 - c1*s2*s3, -s1, c1*c2*s3 + c1*s2*c3,
      s1*c2*c3 - s1*s2*s3,  c1, s1*c2*s3 + s1*s2*c3,
      -s2*c3 - c2*s3,        0, -s2*s3 + c2*c3;

    matrix3_t R36 = R03.transpose() * Minput.rotation();

    switch(i5) {
    case 0:
      q5 = acos(R36(2,2));
      break;
    case 1:
      q5 = -acos(R36(2,2));
      break;
    default:
      abort();
    }
    if ((std::isnan(q5)) || (q5_min > q5) || (q5 > q5_max)) throw NoSolution();
    double s5 = sin(q5);
    if (fabs(s5) < 1e-6) throw NoSolution(); // Singularity
    switch(i4) {
    case 0:
      q4 = atan2(R36(1,2)/s5, R36(0,2)/s5) - 2*pi;
      break;
    case 1:
      q4 = atan2(R36(1,2)/s5, R36(0,2)/s5);
      break;
    case 2:
      q4 = atan2(R36(1,2)/s5, R36(0,2)/s5) + 2*pi;
      break;
    default:
      abort();
    }
    if ((q4_min > q4) || (q4 > q4_max)) throw NoSolution();

    switch(i6) {
    case 0:
      q6 = atan2(R36(2,1)/s5, -R36(2,0)/s5) - 2*pi;
      break;
    case 1:
      q6 = atan2(R36(2,1)/s5, -R36(2,0)/s5);
      break;
    case 2:
      q6 = atan2(R36(2,1)/s5, -R36(2,0)/s5) + 2*pi;
      break;
    default:
      abort();
    }
    if ((q6_min > q6) || (q6 > q6_max)) throw NoSolution();

    result.vector()[0] = q1;
    result.vector()[1] = q2;
    result.vector()[2] = q3;
    result.vector()[3] = q4;
    result.vector()[4] = q5;
    result.vector()[5] = q6;
    hppDout(info, "result.vector()=" << result.vector().transpose());

  }

  virtual void impl_jacobian(matrixOut_t jacobian, vectorIn_t arg) const {
    forwardKinematics(arg);
    J_.resize(6, inVel_.nbIndices());
    Transform3s _0M1(data_.oMi[joint1_->index()]);
    Transform3s _0M2(data_.oMi[joint2_->index()]);
    Transform3s _0Mr(data_.oMi[rootIdx_]);
    Transform3s _rM2(_0Mr.inverse() * _0M2);
    matrix3_t _0R1(_0M1.rotation());
    matrix3_t _1R2(_0R1.transpose() * _0M2.rotation());
    matrix3_t _1Rr(_0R1.transpose() * _0Mr.rotation());
    matrix3_t _2th_cross(cross(frame2_.translation()));
    // pose of the robot root joint
    Transform3s _0Mg(_0M1*frame1_);
    Transform3s _1Mg(_0M1.inverse() * _0Mg);
    vector3_t _1tg(_1Mg.translation());
    vector3_t _rtg((_0Mr.inverse()*_0Mg).translation());

    J2_.resize(6, robot_->numberDof());
    J2_.setZero();
    Jr_.resize(6, robot_->numberDof());
    Jr_.setZero();
    J1_.resize(6, robot_->numberDof());
    J1_.setZero();
    ::pinocchio::getJointJacobian(
      robot_->model(), data_, joint2_->index(),::pinocchio::LOCAL, J2_);
    ::pinocchio::getJointJacobian(robot_->model(), data_, rootIdx_, ::pinocchio::LOCAL, Jr_);
    J2_in_ = inConf_.rview(J2_);
    Jr_in_ = inConf_.rview(Jr_);
    J_.topRows(3) = _1R2*(-_2th_cross*J2_in_.bottomRows(3) + J2_in_.topRows(3)) +
      _1Rr*(cross(_rtg)*Jr_in_.bottomRows(3) - Jr_in_.topRows(3)) +
      _0R1*cross(_1tg)*(_1R2*J2_in_.bottomRows(3) - _1Rr*Jr_in_.bottomRows(3));
    J_.bottomRows(3) = _rM2.rotation() * J2_in_.bottomRows(3) - Jr_in_.bottomRows(3);
    matrix6_t Jout(outConf_.rview(J1_));
    jacobian = Jout.inverse() * J_;
  }
private:
  // Compute forward kinematics with only input variables
  void forwardKinematics(vectorIn_t arg) const {
    qsmall_ = inConf_.rview(robot_->currentConfiguration());
    if (qsmall_ != arg) {
      inConf_.lview(q_) = arg;
    }
    ::pinocchio::forwardKinematics(robot_->model(), data_, q_.head(nq_));
    ::pinocchio::computeJointJacobians(robot_->model(), data_, q_.head(nq_));
    ::pinocchio::updateFramePlacements(robot_->model(), data_);
  }

  DevicePtr_t robot_;
  JointConstPtr_t joint1_, joint2_;
  Transform3s frame1_, frame2_;
  FrameIndex baseLinkIdx_;
  JointIndex rootIdx_;
  Eigen::RowBlockIndices inConf_;
  Eigen::ColBlockIndices inVel_;
  Eigen::RowBlockIndices outConf_;
  int nq_;
  // Local members to avoid memory allocation
  mutable vector_t qsmall_, q_;
  mutable ::pinocchio::Data data_;
  mutable matrix_t J_, J1_, J2_, J2_in_, Jr_, Jr_in_;
}; // class InverseKinematics

class Explicit : public constraints::Explicit {
public:
  typedef shared_ptr<Explicit> Ptr_t;
  typedef weak_ptr<Explicit> WkPtr_t;
  /// Copy object and return shared pointer to copy
  virtual ImplicitPtr_t copy() const{ return createCopy(weak_.lock()); }
  /// Create instance and return shared pointer
  ///
  /// \param name the name of the constraints,
  /// \param robot the robot the constraints is applied to,
  /// \param joint1 the first joint the transformation of which is
  ///               constrained,
  /// \param joint2 the second joint the transformation of which is
  ///               constrained,
  /// \param frame1 position of a fixed frame in joint 1,
  /// \param frame2 position of a fixed frame in joint 2,
  /// \param baseLinkName name of the robot base_link (origin of the robot frame),
  /// \param extraDof extra degree of freedom used to store an integer. This integrer
  ///        determines which inverse kinematics solution to return when several exist.
  /// \note if joint1 is 0x0, joint 1 frame is considered to be the global
  ///       frame.
  static Ptr_t create(
      const std::string& name, const DevicePtr_t& robot,
      const JointConstPtr_t& joint1, const JointConstPtr_t& joint2, const Transform3s& frame1,
      const Transform3s& frame2, const std::string& baseLinkName, size_type extraDof) {
    Explicit* ptr(new Explicit(name, robot, joint1, joint2, frame1, frame2, baseLinkName,
				   extraDof));
    Ptr_t shPtr(ptr);
    WkPtr_t wkPtr(shPtr);
    ptr->init(wkPtr);
    return shPtr;
  }


  static Ptr_t createCopy(const Ptr_t& other) {
    Explicit* ptr(new Explicit(*other));
    Ptr_t shPtr(ptr);
    WkPtr_t wkPtr(shPtr);
    ptr->init(wkPtr);
    return shPtr;
  }

  /// Compute output value assuming right hand side is 0.
  void outputValue(LiegroupElementRef result, vectorIn_t qin, LiegroupElementConstRef rhs) const
  {
    assert(rhs == rhs.space()->neutral());
    explicitFunction()->value(result, qin);
  }
  /// Compute Jacobian value assuming right hand side is 0.
  void jacobianOutputValue(vectorIn_t qin, LiegroupElementConstRef,
			   LiegroupElementConstRef rhs, matrixOut_t jacobian) const
  {
    assert(rhs == rhs.space()->neutral());
    explicitFunction()->jacobian(jacobian, qin);
  }

 protected:
  /// Constructor
  ///
  /// \param name the name of the constraints,
  /// \param robot the robot the constraints is applied to,
  /// \param joint1 the first joint the transformation of which is
  ///               constrained,
  /// \param joint2 the second joint the transformation of which is
  ///               constrained,
  /// \param frame1 position of a fixed frame in joint 1,
  /// \param frame2 position of a fixed frame in joint 2,
  /// \note if joint1 is 0x0, joint 1 frame is considered to be the global
  ///       frame.
  Explicit(const std::string& name, const DevicePtr_t& robot,
	     const JointConstPtr_t& joint1, const JointConstPtr_t& joint2,
	     const Transform3s& frame1, const Transform3s& frame2, const std::string& baseLinkName,
	     size_type extraDof)
    : constraints::Explicit(RelativeTransformationR3xSO3::create(name, robot, joint1, joint2,
								 frame1, frame2,
								 std::vector<bool>(6, true)),
               InverseKinematics::create(name, robot, joint1, joint2,
	           frame1, frame2, baseLinkName, inputConfVariables(robot, joint1, joint2,
								    extraDof),
	           outputConfVariables(joint1), inputVelVariables(robot, joint1, joint2, extraDof)),
               inputConfVariables(robot, joint1, joint2, extraDof), outputConfVariables(joint1),
               inputVelVariables(robot, joint1, joint2, extraDof), outputVelVariables(joint1),
	       ComparisonTypes_t(6*constraints::EqualToZero), std::vector<bool>(6, true)),
      joint1_(joint1),
      joint2_(joint2),
      frame1_(frame1),
      frame2_(frame2),
      extraDof_(extraDof) {
    // Check that extra-dof is actually an extra configuration space variable, i.e. not a
    // configuration variable of pinocchio model
    if (extraDof_ < robot->model().nq) {
      std::ostringstream os;
      os << "Extra degree of freedom (" << extraDof << ") should not be a configuration variable "
	"of the kinematic chain, i.e. should be at least " << robot->model().nq;
      throw std::logic_error(os.str().c_str());
    }
    if (extraDof_ >= robot->configSize()) {
      std::ostringstream os;
      os << "Extra degree of freedom (" << extraDof << ") should be smaller than dimension of "
	"robot configuration space (" << robot->configSize() << ")";
      throw std::logic_error(os.str().c_str());
    }
  }


  /// Copy constructor
  Explicit(const Explicit& other)
    : constraints::Explicit(other),
      joint1_(other.joint1_),
      joint2_(other.joint2_),
      frame1_(other.frame1_),
      frame2_(other.frame2_) {}


  /// Store weak pointer to itself
  void init(WkPtr_t weak) {
    constraints::Explicit::init(weak);
    weak_ = weak;
  }

 private:
  // Create LiegroupSpace instances to avoid useless allocation.
  JointConstPtr_t joint1_, joint2_;
  Transform3s frame1_;
  Transform3s frame2_;
  size_type extraDof_;
  WkPtr_t weak_;

}; // class Explicit

namespace {
static const matrix3_t I3 = matrix3_t::Identity();

struct NoOutputFunction : public DifferentiableFunction {
  NoOutputFunction(size_type sIn, size_type sInD,
               std::string name = std::string("Empty function"))
      : DifferentiableFunction(sIn, sInD, LiegroupSpace::empty(), name) {
    context("Grasp complement");
  }

  inline void impl_compute(pinocchio::LiegroupElementRef, vectorIn_t) const {}
  inline void impl_jacobian(matrixOut_t, vectorIn_t) const {}
};
}  // namespace

ImplicitPtr_t createGrasp(const GripperPtr_t& gripper, const HandleConstPtr_t& handle,
                          size_type extraDof, const std::string& baseLinkName, std::string n)
{
  if (n.empty()) {
    n = gripper->name() + "_grasps_" + handle->name() + "_(explicit)";
  }

  return Explicit::create(n, gripper->joint()->robot(), gripper->joint(), handle->joint(),
      gripper->objectPositionInJoint(), handle->localPosition(), baseLinkName, extraDof);
}
/// Create a trivial constraint
ImplicitPtr_t createGraspComplement(const GripperPtr_t& gripper, const HandleConstPtr_t& handle,
				    std::string n)
{
  if (n.empty()) {
    n = gripper->name() + "_grasps_" + handle->name() + "/complement_" + "_(empty)";
  }
  DevicePtr_t r = handle->robot();
  return Implicit::create(shared_ptr<NoOutputFunction>(new NoOutputFunction(r->configSize(),
      r->numberDof(), n)), ComparisonTypes_t());
}
/// Create a 6D pregrasp explicit constraint
ImplicitPtr_t createPreGrasp(const GripperPtr_t& gripper, const HandleConstPtr_t& handle,
    size_type extraDof, const value_type& shift, const std::string& baseLinkName, std::string n)
{
  Transform3s M = gripper->objectPositionInJoint() *
                  Transform3s(I3, vector3_t(shift, 0, 0));
  if (n.empty())
    n = "Pregrasp_(explicit)_" + handle->name() + "_" + gripper->name();
  return Explicit::create(n, gripper->joint()->robot(), gripper->joint(), handle->joint(),
			    M, handle->localPosition(), baseLinkName, extraDof);
}
} // namespace staubliTx2
} // namespace inverseKinematics
} // namespace hpp
