/// \mainpage Exact inverse kinematics for some industrial robots
///
/// \section hpp_inverse_kinematics_introduction Introduction
///
/// This package provides computation of exact inverse kinematics for some industrial robots.
/// As of now, only robots of the class Stäubli TX2 are supported, but other robots may be added
/// in the future.
///
/// \section hpp_inverse_kinematics_usage Usage
///
/// This package contains a library and a CORBA interface.
///
/// \subsection hpp_inverse_kinematics_subsec_library Library
///
/// The library provides three functions
/// \li \link hpp::inverseKinematics::staubliTx2::createGrasp createGrasp \endlink,
/// \li \link hpp::inverseKinematics::staubliTx2::createGraspComplement createGraspComplement \endlink,
/// \li \link hpp::inverseKinematics::staubliTx2::createGrasp createPreGrasp \endlink.
///
/// corresponding to classical grasps as defined in hpp-manipulation package: hpp::manipulation::Handle::createGrasp. Unlike the latter, method createGrasp in this package computes the angles of the robot axes explicitely, instead of the pose of the object grasped.
///
/// As several solutions may exist for a given desired pose of the gripper, an extra dof is used to
/// select among those solutions. For Stäubli TX2, this extra dof may take integer values between
/// 0 and 143.
///
/// \subsection hpp_inverse_kinematics_subsec_corba CORBA interface
///
/// The above library is exposed as a CORBA interface hpp::corbaserver::inverse_kinematics::StaubliTx2 in a plugin.


