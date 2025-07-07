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
#include <Eigen/Geometry>
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
		      const JointConstPtr_t& jointa, const JointConstPtr_t& jointb,
		      const Transform3s& framea, const Transform3s& frameb,
		      const std::string& baseLinkName, const segments_t inConf,
		      const segments_t outConf, const segments_t inVel) {
    return Ptr_t(new InverseKinematics(name, robot, jointa, jointb, framea, frameb, baseLinkName,
				       inConf, outConf, inVel));
  }
protected:
  InverseKinematics(const std::string& name, const DevicePtr_t&,
		    const JointConstPtr_t&, const JointConstPtr_t&,
		    const Transform3s&, const Transform3s&,
		    const std::string&, const segments_t inConf,
		    const segments_t, const segments_t inVel) :
    DifferentiableFunction(BlockIndex::cardinal(inConf), BlockIndex::cardinal(inVel),
			   LiegroupSpace::Rn(6), name) {
  }

  virtual void impl_compute(LiegroupElementRef, vectorIn_t) const {
    abort();
  }

  virtual void impl_jacobian(matrixOut_t, vectorIn_t) const {
    abort();
  }
private:
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
  /// \param jointa the first joint the transformation of which is
  ///               constrained,
  /// \param jointb the second joint the transformation of which is
  ///               constrained,
  /// \param framea position of a fixed frame in joint a,
  /// \param frameb position of a fixed frame in joint b,
  /// \param baseLinkName name of the robot base_link (origin of the robot frame),
  /// \param extraDof extra degree of freedom used to store an integer. This integrer
  ///        determines which inverse kinematics solution to return when several exist.
  /// \note if jointa is 0x0, joint 1 frame is considered to be the global
  ///       frame.
  static Ptr_t create(
      const std::string& name, const DevicePtr_t& robot,
      const JointConstPtr_t& jointa, const JointConstPtr_t& jointb, const Transform3s& framea,
      const Transform3s& frameb, const std::string& baseLinkName, size_type extraDof) {
    Explicit* ptr(new Explicit(name, robot, jointa, jointb, framea, frameb, baseLinkName,
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
    Transform3s rhs3s(Eigen::Quaternion<value_type>(vector4_t(rhs.vector().tail<4>())),
		      vector3_t(rhs.vector().head<3>()));
    forwardKinematics(qin);
    int extraDof = (int) qin.tail(1)[0];
    // pose of the robot root joint
    Transform3s _0Mb(data_.oMf[baseLinkIdx_]);
    // pose of joint a: Jb * Fb * Fa^{-1}
    Transform3s jointaPose(data_.oMi[jointb_->index()] * frameb_ * rhs3s.inverse() *
			   framea_.inverse());
    // pose of last joint in robot arm base_link
    Transform3s Minput(_0Mb.inverse() * jointaPose);
    // Get joint limits
    int jointaIq = robot_->model().joints[jointa_->index()].idx_q();
    double q1_min = robot_->model().lowerPositionLimit[jointaIq - 6+1];
    double q2_min = robot_->model().lowerPositionLimit[jointaIq - 6+2];
    double q3_min = robot_->model().lowerPositionLimit[jointaIq - 6+3];
    double q4_min = robot_->model().lowerPositionLimit[jointaIq - 6+4];
    double q5_min = robot_->model().lowerPositionLimit[jointaIq - 6+5];
    double q6_min = robot_->model().lowerPositionLimit[jointaIq - 6+6];
    double q1_max = robot_->model().upperPositionLimit[jointaIq - 6+1];
    double q2_max = robot_->model().upperPositionLimit[jointaIq - 6+2];
    double q3_max = robot_->model().upperPositionLimit[jointaIq - 6+3];
    double q4_max = robot_->model().upperPositionLimit[jointaIq - 6+4];
    double q5_max = robot_->model().upperPositionLimit[jointaIq - 6+5];
    double q6_max = robot_->model().upperPositionLimit[jointaIq - 6+6];

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

    constexpr double pi = 3.14159265358979323846;

    double q1, q2, q3, q4, q5, q6;

    // double r1 = 0.478;
    // double r2 = 0.050;
    // double r3 = 0.050;
    // double r4 = 0.425;
    // double r5 = 0.425;
    // double r6 = 0.1;

    vector3_t offset; offset << 0, 0, -r6_;
    vector3_t concurrentPoint = Minput.act(offset);

    double X = concurrentPoint(0);
    double Y = concurrentPoint(1);
    double Z = concurrentPoint(2);

    double rho = sqrt(X*X+Y*Y);
    if (rho < r3_) throw NoSolution();
    double theta = atan2(Y, X);

    switch (i1) {
    case 0:
      q1 = theta - asin(r3_/rho);
      break;
    case 1:
      q1 = theta - asin(r3_/rho) + 2*pi;
      break;
    case 2:
      q1 = theta + pi + asin(r3_/rho);
      break;
    case 3:
      q1 = theta - pi + asin(r3_/rho);
      break;
    default:
      abort();
    }
    if ((q1_min > q1) || (q1 > q1_max)) throw NoSolution();
    double c1 = cos(q1), s1 = sin(q1);
    double x = c1*X + s1*Y;

    switch(i3) {
    case 0:
      q3 = acos(((x-r2_)*(x-r2_) + (Z-r1_)*(Z-r1_) - r4_*r4_ - r5_*r5_)/(2*r4_*r5_));
      break;
    case 1:
      q3 = -acos(((x-r2_)*(x-r2_) + (Z-r1_)*(Z-r1_) - r4_*r4_ - r5_*r5_)/(2*r4_*r5_));
      break;
    default:
      abort();
    }
    if (std::isnan(q3) || (q3_min > q3) || (q3 > q3_max)) throw NoSolution();
    double c3 = cos(q3), s3 = sin(q3);

    q2 = atan2((r5_*c3+r4_)*(x-r2_)-r5_*s3*(Z-r1_),r5_*s3*(x-r2_)+(r5_*c3+r4_)*(Z-r1_));
    if ((q2_min > q2) || (q2 > q2_max)) throw NoSolution();
    double c2 = cos(q2), s2 = sin(q2);

    // assert(fabs(r5_*c1*s23 - r3_*s1 + r4_*c1*s2 + r2_*c1 - X) < 1e-8);
    // assert(fabs(r5_*s1*s23 + r3_*c1 + r4_*s1*s2 + r2_*s1 - Y) < 1e-8);
    assert(fabs(r5_*(c2*c3 - s2*s3) + r4_*c2 + r1_ - Z) < 1e-8);
    assert(fabs(r5_*(s2*c3 + c2*s3) + r4_*s2 + r2_ - x) < 1e-8);
    assert(fabs(c1*Y - s1*X-r3_) < 1e-8);
    assert(fabs(X - rho*cos(theta)) < 1e-8);
    assert(fabs(Y - rho*sin(theta)) < 1e-8);
    assert(fabs(r3_ - rho*(c1*sin(theta)-cos(theta)*s1)) < 1e-8);
    assert(fabs(sin(theta - q1) - r3_/rho) < 1e-8);
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
  }
  /// Compute Jacobian value assuming right hand side is 0.
  void jacobianOutputValue(vectorIn_t qin, LiegroupElementConstRef,
			   LiegroupElementConstRef rhs, matrixOut_t jacobian) const
  {
    Transform3s rhs3s(Eigen::Quaternion<value_type>(vector4_t(rhs.vector().tail<4>())),
		      vector3_t(rhs.vector().head<3>()));
    forwardKinematics(qin);
    J_.resize(6, BlockIndex::cardinal(inputVelocity()));
    Transform3s _0Ma(data_.oMi[jointa_->index()]);
    Transform3s _0Mb(data_.oMi[jointb_->index()]);
    Transform3s _0Mr(data_.oMi[rootIdx_]);
    Transform3s _rMb(_0Mr.inverse() * _0Mb);
    matrix3_t _0Ra(_0Ma.rotation());
    matrix3_t _aRb(_0Ra.transpose() * _0Mb.rotation());
    matrix3_t _aRr(_0Ra.transpose() * _0Mr.rotation());
    matrix3_t _bth_cross(cross(frameb_.translation()));
    // pose of the robot root joint
    Transform3s _0Mgp(_0Ma*framea_);
    Transform3s _aMgp(_0Ma.inverse() * _0Mgp);
    vector3_t _atgp(_aMgp.translation());
    vector3_t _rtgp((_0Mr.inverse()*_0Mgp).translation());

    Jb_.resize(6, robot_->numberDof());
    Jb_.setZero();
    Jr_.resize(6, robot_->numberDof());
    Jr_.setZero();
    Ja_.resize(6, robot_->numberDof());
    Ja_.setZero();
    ::pinocchio::getJointJacobian(
      robot_->model(), data_, jointb_->index(),::pinocchio::LOCAL, Jb_);
    ::pinocchio::getJointJacobian(robot_->model(), data_, rootIdx_, ::pinocchio::LOCAL, Jr_);
    Jb_in_ = Eigen::RowBlockIndices(inputConf()).rview(Jb_);
    Jr_in_ = Eigen::RowBlockIndices(inputConf()).rview(Jr_);
    J_.topRows(3) = _aRb*(-_bth_cross*Jb_in_.bottomRows(3) + Jb_in_.topRows(3)) +
      _aRr*(cross(_rtgp)*Jr_in_.bottomRows(3) - Jr_in_.topRows(3)) +
      _0Ra*cross(_atgp)*(_aRb*Jb_in_.bottomRows(3) - _aRr*Jr_in_.bottomRows(3));
    J_.bottomRows(3) = _rMb.rotation() * Jb_in_.bottomRows(3) - Jr_in_.bottomRows(3);
    matrix6_t Jout(Eigen::RowBlockIndices(outputConf()).rview(Ja_));
    jacobian = Jout.inverse() * J_;
  }

 protected:
  /// Constructor
  ///
  /// \param name the name of the constraints,
  /// \param robot the robot the constraints is applied to,
  /// \param jointa the first joint the transformation of which is
  ///               constrained,
  /// \param jointb the second joint the transformation of which is
  ///               constrained,
  /// \param framea position of a fixed frame in joint a,
  /// \param frameb position of a fixed frame in joint b,
  /// \note if jointa is 0x0, joint a frame is considered to be the global
  ///       frame.
  Explicit(const std::string& name, const DevicePtr_t& robot,
	     const JointConstPtr_t& jointa, const JointConstPtr_t& jointb,
	     const Transform3s& framea, const Transform3s& frameb, const std::string& baseLinkName,
	     size_type extraDof)
    : constraints::Explicit(RelativeTransformationR3xSO3::create(name, robot, jointa, jointb,
								 framea, frameb,
								 std::vector<bool>(6, true)),
               InverseKinematics::create(name, robot, jointa, jointb,
	           framea, frameb, baseLinkName, inputConfVariables(robot, jointa, jointb,
								    extraDof),
	           outputConfVariables(jointa), inputVelVariables(robot, jointa, jointb, extraDof)),
               inputConfVariables(robot, jointa, jointb, extraDof), outputConfVariables(jointa),
               inputVelVariables(robot, jointa, jointb, extraDof), outputVelVariables(jointa),
	       ComparisonTypes_t(6*constraints::EqualToZero), std::vector<bool>(6, true)),
    robot_(robot), jointa_ (jointa),
    jointb_ (jointb), framea_ (framea), frameb_ (frameb), extraDof_(extraDof), baseLinkIdx_(
      robot->model().getFrameId(baseLinkName)),
    rootIdx_(robot->model().frames[baseLinkIdx_].parentJoint),
    nq_(robot->model().nq), r1_(0), r2_(0), r3_(0), r4_(0), r5_(0), r6_(0),
    data_(robot->model())
  {
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
    q_.resize(robot->configSize());
    ::pinocchio::neutral(robot->model(), q_.head(nq_));
    retrieveGeometricalParameters(baseLinkName);
  }


  /// Copy constructor
  Explicit(const Explicit& other)
    : constraints::Explicit(other),
      jointa_(other.jointa_),
      jointb_(other.jointb_),
      framea_(other.framea_),
      frameb_(other.frameb_) {}


  /// Store weak pointer to itself
  void init(WkPtr_t weak) {
    constraints::Explicit::init(weak);
    weak_ = weak;
  }

 private:
  // Compute forward kinematics with only input variables
  void forwardKinematics(vectorIn_t arg) const {
    qsmall_ = Eigen::RowBlockIndices(inputConf()).rview(robot_->currentConfiguration());
    if (qsmall_ != arg) {
      Eigen::RowBlockIndices(inputConf()).lview(q_) = arg;
    }
    ::pinocchio::forwardKinematics(robot_->model(), data_, q_.head(nq_));
    ::pinocchio::computeJointJacobians(robot_->model(), data_, q_.head(nq_));
    ::pinocchio::updateFramePlacements(robot_->model(), data_);
  }

  void retrieveGeometricalParameters(const std::string& baseLinkName)
  {
    JointIndex i1 = jointa_->index() - 5,
      i2 = jointa_->index() - 4,
      i3 = jointa_->index() - 3,
      i4 = jointa_->index() - 2,
      i5 = jointa_->index() - 1,
      i6 = jointa_->index() - 0;
    // Check that base_link and jointa have same parent joint
    if (robot_->model().parents[i1] != rootIdx_) {
      std::ostringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: base_link ("
	 << baseLinkName << ") does not have the same parent as joint ("
	 << robot_->model().names[i1] << ").";
      throw std::logic_error(os.str().c_str());
    }
    // Placement of base link i parent joint
    pinocchio::SE3 pMb(robot_->model().frames[baseLinkIdx_].placement);
    pinocchio::SE3 pMj1(robot_->model().jointPlacements[i1]);
    pinocchio::SE3 j1Mj2(robot_->model().jointPlacements[i2]);
    pinocchio::SE3 j2Mj3(robot_->model().jointPlacements[i3]);
    pinocchio::SE3 j3Mj4(robot_->model().jointPlacements[i4]);
    pinocchio::SE3 j4Mj5(robot_->model().jointPlacements[i5]);
    pinocchio::SE3 j5Mj6(robot_->model().jointPlacements[i6]);
    // Check that center of jointa is on z-axis of base_link
    pinocchio::SE3 bMj1(pMb.inverse()*pMj1);
    // Use a high threshold in case the robot has been calibrated
    if ((fabs(bMj1.translation()(0)) > 1e-3) || (fabs(bMj1.translation()(0)) > 1e-3)) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: joint ("
	 << robot_->model().names[i1] << ") center is not on z-axis of base_link ("
	 << baseLinkName << ").";
      throw std::logic_error(os.str().c_str());
    }
    // check that jointa is oriented like base_link
    if (::pinocchio::log3(bMj1.rotation()).norm() > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: jointa ("
	 << robot_->model().names[i1] << ") has different orientation than base_link ("
	 << baseLinkName << ").";
      throw std::logic_error(os.str().c_str());
    }

    pinocchio::SE3 bMj2(bMj1*j1Mj2);
    // check that jointb is oriented like base_link
    if (::pinocchio::log3(bMj2.rotation()).norm() > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: joint ("
	 << robot_->model().names[i2] << ") has different orientation than base_link ("
	 << baseLinkName << ").";
      throw std::logic_error(os.str().c_str());
    }
    // retrieve r1 and r2
    r1_ = bMj2.translation()(2);
    r2_ = bMj2.translation()(0);

    // check that joint3 is oriented like joint 2
    if (::pinocchio::log3(j2Mj3.rotation()).norm() > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: joint ("
	 << robot_->model().names[i3] << ") has different orientation than joint ("
	 << robot_->model().names[i2] << ").";
      throw std::logic_error(os.str().c_str());
    }
    if (fabs(j2Mj3.translation()(0)) > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: center of joint ("
	 << robot_->model().names[i3] << ") should have 0 x-coordinate in joint ("
	 << robot_->model().names[i2] << "): got " << j2Mj3.translation()(0) << ".";
      throw std::logic_error(os.str().c_str());
    }
    r4_ = j2Mj3.translation()(2);

    ::pinocchio::SE3 j2Mj4(j2Mj3*j3Mj4);
    // check that joint4 is oriented like joint 2
    if (::pinocchio::log3(j2Mj4.rotation()).norm() > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: joint ("
	 << robot_->model().names[i4] << ") has different orientation than joint ("
	 << robot_->model().names[i2] << ").";
      throw std::logic_error(os.str().c_str());
    }
    // retrieve parameters r3 and r4
    r3_ = j2Mj4.translation()(1);

    pinocchio::SE3 j3Mj5(j3Mj4*j4Mj5);
    // check that joint5 is oriented like joint 3
    if (::pinocchio::log3(j3Mj5.rotation()).norm() > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: joint ("
	 << robot_->model().names[i4] << ") has different orientation than joint ("
	 << robot_->model().names[i2] << ").";
      throw std::logic_error(os.str().c_str());
    }
    // retrieve parameter r5
    r5_ = j3Mj5.translation()(2);
    // check that joint6 is oriented like joint 5
    if (::pinocchio::log3(j5Mj6.rotation()).norm() > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: joint ("
	 << robot_->model().names[i6] << ") has different orientation than joint ("
	 << robot_->model().names[i5] << ").";
      throw std::logic_error(os.str().c_str());
    }
    if (fabs(j5Mj6.translation()(0)) > 1e-3 || fabs(j5Mj6.translation()(1)) > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: center of joint ("
	 << robot_->model().names[i6] << ") should be on z-axis of joint ("
	 << robot_->model().names[i6] << ") coordinates are "
	 << j5Mj6.translation() << ".";
      throw std::logic_error(os.str().c_str());
    }
    r6_ = j5Mj6.translation()(2);
    // Check rotation axes of each joint using Jacobian matrix
    vector_t q(robot_->model().nq);
    ::pinocchio::neutral(robot_->model(), q);
    ::pinocchio::forwardKinematics(robot_->model(), data_, q);
    ::pinocchio::computeJointJacobians(robot_->model(), data_, q);
    vector3_t axis;
    vector3_t y_axis; y_axis << 0, 1, 0;
    vector3_t z_axis; z_axis << 0, 0, 1;
    // joint 1
    axis = data_.J.col(robot_->model().idx_vs[i1]).tail(3);
    if ((axis-z_axis).norm() > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: joint ("
	 << robot_->model().names[i1] << ") should rotate around z-axis. Rotation axis is "
	 << axis << ".";
      throw std::logic_error(os.str().c_str());
    }
    // joint 2
    axis = data_.J.col(robot_->model().idx_vs[i2]).tail(3);
    if ((axis-y_axis).norm() > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: joint ("
	 << robot_->model().names[i2] << ") should rotate around y-axis. Rotation axis is "
	 << axis << ".";
      throw std::logic_error(os.str().c_str());
    }
    // joint 3
    axis = data_.J.col(robot_->model().idx_vs[i3]).tail(3);
    if ((axis-y_axis).norm() > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: joint ("
	 << robot_->model().names[i3] << ") should rotate around y-axis. Rotation axis is "
	 << axis << ".";
      throw std::logic_error(os.str().c_str());
    }
    // joint 4
    axis = data_.J.col(robot_->model().idx_vs[i4]).tail(3);
    if ((axis-z_axis).norm() > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: joint ("
	 << robot_->model().names[i4] << ") should rotate around z-axis. Rotation axis is "
	 << axis << ".";
      throw std::logic_error(os.str().c_str());
    }
    // joint 5
    axis = data_.J.col(robot_->model().idx_vs[i5]).tail(3);
    if ((axis-y_axis).norm() > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: joint ("
	 << robot_->model().names[i5] << ") should rotate around y-axis. Rotation axis is "
	 << axis << ".";
      throw std::logic_error(os.str().c_str());
    }
    // joint 6
    axis = data_.J.col(robot_->model().idx_vs[i6]).tail(3);
    if ((axis-z_axis).norm() > 1e-3) {
      std::stringstream os;
      os << "hpp::inverseKinematics::StaubliTx2::InverseKinematics: joint ("
	 << robot_->model().names[i6] << ") should rotate around z-axis. Rotation axis is "
	 << axis << ".";
      throw std::logic_error(os.str().c_str());
    }
  }
  // Create LiegroupSpace instances to avoid useless allocation.
  DevicePtr_t robot_;
  JointConstPtr_t jointa_, jointb_;
  Transform3s framea_;
  Transform3s frameb_;
  size_type extraDof_;
  FrameIndex baseLinkIdx_;
  JointIndex rootIdx_;
  int nq_;
  // Geometrical values retrieved from the model
  value_type r1_, r2_, r3_, r4_, r5_, r6_;
  // Local members to avoid memory allocation
  mutable vector_t qsmall_, q_;
  mutable ::pinocchio::Data data_;
  mutable matrix_t J_, Ja_, Jb_, Jb_in_, Jr_, Jr_in_;
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
