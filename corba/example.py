import os
import numpy as np
from eigenpy import Quaternion
from math import sqrt, cos, sin, pi
from hpp.corbaserver import loadServerPlugin
from hpp.rostools import process_xacro, retrieve_resource
from hpp.corbaserver.manipulation import ConstraintGraph, ConstraintGraphFactory, Constraints, \
    Robot, newProblem, ProblemSolver
from hpp.gepetto.manipulation import ViewerFactory
from hpp.corbaserver.coverage import Client as CovClient
from hpp.corbaserver.inverse_kinematics import Client as IkClient

class Part:
    urdfFilename = "package://hpp-coverage/urdf/part.urdf"
    srdfFilename = "package://hpp-coverage/srdf/part.srdf"
    rootJointType = "freeflyer"

# Loading a manipulation server plugin and refresh problem
loadServerPlugin ("corbaserver", "manipulation-corba.so")
loadServerPlugin ("corbaserver", "coverage.so")
loadServerPlugin ("corbaserver", "inverse_kinematics.so")
newProblem()

Robot.urdfFilename = "package://hpp-coverage/urdf/syabot.urdf"
Robot.srdfFilename = "package://hpp-coverage/srdf/syabot.srdf"

robot= Robot("staubli-part", "staubli", rootJointType="freeflyer")

ps = ProblemSolver(robot)
vf = ViewerFactory(ps)

vf.loadRobotModel(Part, "part")
robot.setJointBounds("part/root_joint", [-1., 1., -1., 1.,-1., 1.])
# Add a configuration variable for exact inverse kinematics
robot.client.basic.robot.setDimensionExtraConfigSpace(1)
robot.client.basic.robot.setExtraConfigSpaceBounds([0, 16])

# [0:7] : freeflyer arm
# [7:13] : arm joints,
# [13:20] : freeflyer part
# [20:21] : extra dof
q0 = 6*[0.] + [1.] + 6*[0.] + [-.7, 0, -0.1] + [0, 0, sqrt(2)/2, sqrt(2)/2] + [0]

# Create exact inverse kinematics constraints
s = IkClient()
s.staubli_tx2.createGrasp("staubli/tooltip grasps part/part_top", "staubli/tooltip",
                          "part/part_top", "staubli/base_link", 20)

s.staubli_tx2.createPreGrasp("staubli/tooltip pregrasps part/part_top", "staubli/tooltip",
                             "part/part_top", "staubli/base_link", 20)

# Create the constraint graph
cg = ConstraintGraph(robot, 'graph')
factory = ConstraintGraphFactory(cg)
factory.setGrippers(["staubli/tooltip"])
factory.setObjects(["part"], [["part/part_top"]], [[]])
factory.generate()
cg.initialize()

q0[0] = .28
q0[1] = .14

res, q1, err = cg.applyNodeConstraints("staubli/tooltip grasps part/part_top", q0)
