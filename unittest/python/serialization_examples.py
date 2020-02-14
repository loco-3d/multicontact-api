import unittest

import numpy as np
from numpy import array, array_equal
from random import uniform
from math import sqrt, sin, cos
import pathlib
import pinocchio as pin
from pinocchio import SE3, Quaternion
import curves
from curves import SE3Curve, polynomial, bezier, piecewise, piecewise_SE3
from multicontact_api import ContactSequence

pin.switchToNumpyArray()

PATH = str(pathlib.Path(__file__).parent.parent.parent.absolute())+"/examples/"
print("PATH : ", PATH)



def assertTrajNotNone(testCase, phase, root,  wholeBody):
    testCase.assertIsNotNone(phase.c_t)
    testCase.assertIsNotNone(phase.dc_t)
    testCase.assertIsNotNone(phase.ddc_t)
    testCase.assertIsNotNone(phase.L_t)
    testCase.assertIsNotNone(phase.dL_t)
    if root:
        testCase.assertIsNotNone(phase.root_t)
    if wholeBody:
        testCase.assertIsNotNone(phase.q_t)
        testCase.assertIsNotNone(phase.dq_t)
        testCase.assertIsNotNone(phase.ddq_t)
        testCase.assertIsNotNone(phase.tau_t)


def testTrajMinMax(testCase, phase,root, wholeBody):
    testCase.assertTrue(phase.c_t.min() >= 0.)
    testCase.assertTrue(phase.dc_t.min() >= 0.)
    testCase.assertTrue(phase.ddc_t.min() >= 0.)
    testCase.assertTrue(phase.L_t.min() >= 0.)
    testCase.assertTrue(phase.dL_t.min() >= 0.)
    testCase.assertTrue(phase.c_t.max() >= 0.)
    testCase.assertTrue(phase.dc_t.max() >= 0.)
    testCase.assertTrue(phase.ddc_t.max() >= 0.)
    testCase.assertTrue(phase.L_t.max() >= 0.)
    testCase.assertTrue(phase.dL_t.max() >= 0.)
    if root:
        testCase.assertTrue(phase.root_t.min() >= 0.)
        testCase.assertTrue(phase.root_t.max() >= 0.)
    if wholeBody:
        testCase.assertTrue(phase.q_t.max() >= 0.)
        testCase.assertTrue(phase.dq_t.max() >= 0.)
        testCase.assertTrue(phase.ddq_t.max() >= 0.)
        testCase.assertTrue(phase.tau_t.max() >= 0.)
        testCase.assertTrue(phase.q_t.min() >= 0.)
        testCase.assertTrue(phase.dq_t.min() >= 0.)
        testCase.assertTrue(phase.ddq_t.min() >= 0.)
        testCase.assertTrue(phase.tau_t.min() >= 0.)

def testCallTraj(testCase, phase, root, quasistatic, wholeBody):
    testCase.assertTrue(phase.c_t((phase.c_t.max() + phase.c_t.min()) / 2.).any())
    if not quasistatic:
        testCase.assertTrue(phase.dc_t((phase.dc_t.max() + phase.dc_t.min()) / 2.).any())
        testCase.assertTrue(phase.ddc_t((phase.ddc_t.max() + phase.ddc_t.min()) / 2.).any())
    #testCase.assertTrue(phase.L_t((phase.L_t.max() + phase.L_t.min()) / 2.).any())
    #testCase.assertTrue(phase.dL_t((phase.dL_t.max() + phase.dL_t.min()) / 2.).any())
    if root:
        testCase.assertTrue(phase.root_t.max() >= 0.)
    if wholeBody:
        testCase.assertTrue(phase.q_t((phase.q_t.max() + phase.q_t.min()) / 2.).any())
        testCase.assertTrue(phase.dq_t((phase.dq_t.max() + phase.dq_t.min()) / 2.).any())
        testCase.assertTrue(phase.ddq_t((phase.ddq_t.max() + phase.ddq_t.min()) / 2.).any())
        testCase.assertTrue(phase.tau_t((phase.tau_t.max() + phase.tau_t.min()) / 2.).any())


def testEffectorTraj(testCase, phase):
    for eeName, traj in phase.effectorTrajectories().items():
        testCase.assertIsNotNone(traj)
        testCase.assertTrue(traj.min() >= 0.)
        testCase.assertTrue(traj.max() >= 0.)
        testCase.assertTrue(traj((traj.min() + traj.max()) / 2.).any())


def testContactForce(testCase, phase):
    for eeName, traj in phase.contactForces().items():
        testCase.assertIsNotNone(traj)
        testCase.assertTrue(traj.min() >= 0.)
        testCase.assertTrue(traj.max() >= 0.)
        testCase.assertTrue(traj((traj.min() + traj.max()) / 2.).any())


def checkPhase(testCase, phase, root = False, quasistatic = False, effector = False, wholeBody = False):
    assertTrajNotNone(testCase, phase, root, wholeBody)
    testTrajMinMax(testCase, phase, root, wholeBody)
    testCallTraj(testCase, phase, root, quasistatic, wholeBody)
    if effector:
        testEffectorTraj(testCase, phase)
    if wholeBody:
        testContactForce(testCase, phase)

def checkCS(testCase, cs, root = False, quasistatic = False, effector = False, wholeBody = False):
    for phase in cs.contactPhases:
        checkPhase(testCase, phase, root, quasistatic, effector, wholeBody)

class ExamplesSerialization(unittest.TestCase):

    def test_com_motion_above_feet_COM(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"com_motion_above_feet_COM.cs")
        self.assertEqual(cs.size(), 1)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        checkCS(self, cs)



    def test_com_motion_above_feet_WB(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"com_motion_above_feet_WB.cs")
        self.assertEqual(cs.size(), 1)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        self.assertTrue(cs.haveJointsTrajectories())
        self.assertTrue(cs.haveJointsDerivativesTrajectories())
        self.assertTrue(cs.haveContactForcesTrajectories())
        self.assertTrue(cs.haveZMPtrajectories())
        checkCS(self, cs, wholeBody = True)



    def test_step_in_place(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"step_in_place.cs")
        self.assertEqual(cs.size(), 9)
        self.assertTrue(cs.haveConsistentContacts())


    def test_step_in_place_COM(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"step_in_place_COM.cs")
        self.assertEqual(cs.size(), 9)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        checkCS(self, cs, effector = False, wholeBody = False)


    def test_step_in_place_REF(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"step_in_place_REF.cs")
        self.assertEqual(cs.size(), 9)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        self.assertTrue(cs.haveEffectorsTrajectories())
        checkCS(self, cs, root = True, effector = True, wholeBody = False)



    def test_step_in_place_WB(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"step_in_place_WB.cs")
        self.assertEqual(cs.size(), 9)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        self.assertTrue(cs.haveEffectorsTrajectories(1e-1))
        self.assertTrue(cs.haveJointsTrajectories())
        self.assertTrue(cs.haveJointsDerivativesTrajectories())
        self.assertTrue(cs.haveContactForcesTrajectories())
        self.assertTrue(cs.haveZMPtrajectories())
        checkCS(self, cs, effector = True, wholeBody = True)

    def test_step_in_place_quasistatic(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"step_in_place_quasistatic.cs")
        self.assertEqual(cs.size(), 9)
        self.assertTrue(cs.haveConsistentContacts())


    def test_step_in_place_quasistatic_COM(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"step_in_place_quasistatic_COM.cs")
        self.assertEqual(cs.size(), 9)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        checkCS(self, cs, quasistatic = True, effector = False, wholeBody = False)

    def test_step_in_place_quasistatic_REF(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"step_in_place_quasistatic_REF.cs")
        self.assertEqual(cs.size(), 9)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        self.assertTrue(cs.haveEffectorsTrajectories())
        checkCS(self, cs, root = True, quasistatic = True, effector = True, wholeBody = False)

    def test_step_in_place_quasistatic_WB(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"step_in_place_quasistatic_WB.cs")
        self.assertEqual(cs.size(), 9)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        self.assertTrue(cs.haveEffectorsTrajectories(1e-1))
        self.assertTrue(cs.haveJointsTrajectories())
        self.assertTrue(cs.haveJointsDerivativesTrajectories())
        self.assertTrue(cs.haveContactForcesTrajectories())
        self.assertTrue(cs.haveZMPtrajectories())
        checkCS(self, cs, quasistatic = True, effector = True, wholeBody = True)

    def test_walk_20cm(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"walk_20cm.cs")
        self.assertEqual(cs.size(), 23)
        self.assertTrue(cs.haveConsistentContacts())


    def test_walk_20cm_COM(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"walk_20cm_COM.cs")
        self.assertEqual(cs.size(), 23)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        checkCS(self, cs, effector = False, wholeBody = False)

    def test_walk_20cm_REF(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"walk_20cm_REF.cs")
        self.assertEqual(cs.size(), 23)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        self.assertTrue(cs.haveEffectorsTrajectories())
        checkCS(self, cs, root = True, effector = True, wholeBody = False)

    def test_walk_20cm_WB(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"walk_20cm_WB.cs")
        self.assertEqual(cs.size(), 23)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        self.assertTrue(cs.haveEffectorsTrajectories(1e-1))
        self.assertTrue(cs.haveJointsTrajectories())
        self.assertTrue(cs.haveJointsDerivativesTrajectories())
        self.assertTrue(cs.haveContactForcesTrajectories())
        self.assertTrue(cs.haveZMPtrajectories())
        checkCS(self, cs, effector = True, wholeBody = True)

    def test_walk_20cm_quasistatic(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"walk_20cm_quasistatic.cs")
        self.assertEqual(cs.size(), 23)
        self.assertTrue(cs.haveConsistentContacts())


    def test_walk_20cm_quasistatic_COM(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"walk_20cm_quasistatic_COM.cs")
        self.assertEqual(cs.size(), 23)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        checkCS(self, cs, quasistatic = True, effector = False, wholeBody = False)

    def test_walk_20cm_quasistatic_REF(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"walk_20cm_quasistatic_REF.cs")
        self.assertEqual(cs.size(), 23)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        self.assertTrue(cs.haveEffectorsTrajectories())
        checkCS(self, cs, root = True, quasistatic = True, effector = True, wholeBody = False)

    def test_walk_20cm_quasistatic_WB(self):
        cs = ContactSequence()
        cs.loadFromBinary(PATH+"walk_20cm_quasistatic_WB.cs")
        self.assertEqual(cs.size(), 23)
        self.assertTrue(cs.haveConsistentContacts())
        self.assertTrue(cs.haveTimings())
        self.assertTrue(cs.haveCentroidalValues())
        self.assertTrue(cs.haveCentroidalTrajectories())
        self.assertTrue(cs.haveEffectorsTrajectories(1e-1))
        self.assertTrue(cs.haveJointsTrajectories())
        self.assertTrue(cs.haveJointsDerivativesTrajectories())
        self.assertTrue(cs.haveContactForcesTrajectories())
        self.assertTrue(cs.haveZMPtrajectories())
        checkCS(self, cs, quasistatic = True, effector = True, wholeBody = True)

if __name__ == '__main__':
    unittest.main()
