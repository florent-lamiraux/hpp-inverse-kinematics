# Exact inverse kinematics for some industrial robots

This package implements exact inverse kinematics for some industrial robots as an explicit
constraint as defined in `hpp-constraints`. The constraint is a grasp constraint as defined in
`hpp-manipulation`. It takes an additional variable since exact inverse kinematics might have
several solutions. The additional  variable may be an extra-configuration space variable and is
assumed to be integer valued.

Right now, only robots of the serie Stäubli TX2 are supported, but extension to other robots
should not be difficult.

## Validation

Inverse kinematics for the Stäubli TX2 series has been validated for the following models:

  - Stäubli TX2 90: retrieval of Denavit-Hartenberg (DH) parameters, computation of inverse
    kinematics,
  - Stäubli TX2 60: retrieval of DH parameters.
