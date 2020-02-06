import unittest

import numpy as np
from numpy import array, array_equal
from random import uniform
from math import sqrt, sin, cos

import pinocchio as pin
from pinocchio import SE3, Quaternion
from curves import SE3Curve, polynomial, bezier, piecewise, piecewise_SE3
from multicontact_api import ContactModelPlanar, ContactPatch, ContactPhase, ContactSequence

pin.switchToNumpyArray()


def assertTrajNotNone(testCase, phase):
    testCase.assertIsNotNone(phase.c_t)
    testCase.assertIsNotNone(phase.dc_t)
    testCase.assertIsNotNone(phase.ddc_t)
    testCase.assertIsNotNone(phase.L_t)
    testCase.assertIsNotNone(phase.dL_t)
    testCase.assertIsNotNone(phase.q_t)
    testCase.assertIsNotNone(phase.dq_t)
    testCase.assertIsNotNone(phase.ddq_t)
    testCase.assertIsNotNone(phase.tau_t)
    testCase.assertIsNotNone(phase.root_t)


def testTrajMinMax(testCase, phase):
    testCase.assertTrue(phase.c_t.min() >= 0.)
    testCase.assertTrue(phase.dc_t.min() >= 0.)
    testCase.assertTrue(phase.ddc_t.min() >= 0.)
    testCase.assertTrue(phase.L_t.min() >= 0.)
    testCase.assertTrue(phase.dL_t.min() >= 0.)
    testCase.assertTrue(phase.q_t.min() >= 0.)
    testCase.assertTrue(phase.dq_t.min() >= 0.)
    testCase.assertTrue(phase.ddq_t.min() >= 0.)
    testCase.assertTrue(phase.tau_t.min() >= 0.)
    testCase.assertTrue(phase.root_t.min() >= 0.)
    testCase.assertTrue(phase.c_t.max() >= 0.)
    testCase.assertTrue(phase.dc_t.max() >= 0.)
    testCase.assertTrue(phase.ddc_t.max() >= 0.)
    testCase.assertTrue(phase.L_t.max() >= 0.)
    testCase.assertTrue(phase.dL_t.max() >= 0.)
    testCase.assertTrue(phase.q_t.max() >= 0.)
    testCase.assertTrue(phase.dq_t.max() >= 0.)
    testCase.assertTrue(phase.ddq_t.max() >= 0.)
    testCase.assertTrue(phase.tau_t.max() >= 0.)
    testCase.assertTrue(phase.root_t.max() >= 0.)


def testCallTraj(testCase, phase):
    testCase.assertTrue(phase.c_t((phase.c_t.max() + phase.c_t.min()) / 2.).any())
    testCase.assertTrue(phase.dc_t((phase.dc_t.max() + phase.dc_t.min()) / 2.).any())
    testCase.assertTrue(phase.ddc_t((phase.ddc_t.max() + phase.ddc_t.min()) / 2.).any())
    testCase.assertTrue(phase.L_t((phase.L_t.max() + phase.L_t.min()) / 2.).any())
    testCase.assertTrue(phase.dL_t((phase.dL_t.max() + phase.dL_t.min()) / 2.).any())
    testCase.assertTrue(phase.q_t((phase.q_t.max() + phase.q_t.min()) / 2.).any())
    testCase.assertTrue(phase.dq_t((phase.dq_t.max() + phase.dq_t.min()) / 2.).any())
    testCase.assertTrue(phase.ddq_t((phase.ddq_t.max() + phase.ddq_t.min()) / 2.).any())
    testCase.assertTrue(phase.tau_t((phase.tau_t.max() + phase.tau_t.min()) / 2.).any())
    testCase.assertTrue(phase.root_t((phase.root_t.max() + phase.root_t.min()) / 2.).any())


def testEffectorTraj(testCase, phase):
    testCase.assertTrue(phase.effectorHaveAtrajectory("right-hand"))
    testCase.assertTrue(phase.effectorHaveAtrajectory("left-hand"))
    eR = phase.effectorTrajectory("right-hand")
    eL = phase.effectorTrajectory("left-hand")
    testCase.assertIsNotNone(eR)
    testCase.assertIsNotNone(eL)
    testCase.assertTrue(eR.min() >= 0.)
    testCase.assertTrue(eR.max() >= 0.)
    testCase.assertTrue(eR((eR.min() + eR.max()) / 2.).any())
    testCase.assertTrue(eL.min() >= 0.)
    testCase.assertTrue(eL.max() >= 0.)
    testCase.assertTrue(eL((eL.min() + eL.max()) / 2.).any())


def testContactForce(testCase, phase):
    fR = phase.contactForce("right-leg")
    testCase.assertIsNotNone(fR)
    testCase.assertTrue(fR.min() >= 0.)
    testCase.assertTrue(fR.max() >= 0.)
    testCase.assertTrue(fR((fR.min() + fR.max()) / 2.).any())


def checkPhase(testCase, phase):
    assertTrajNotNone(testCase, phase)
    testTrajMinMax(testCase, phase)
    testCallTraj(testCase, phase)
    testEffectorTraj(testCase, phase)
    testContactForce(testCase, phase)


class ContactPhaseTest(unittest.TestCase):
    def test_deserialize_text(self):
        cp = ContactPhase()
        cp.loadFromText("cp_test_full.txt")
        checkPhase(self, cp)

    def test_deserialize_bin(self):
        cp = ContactPhase()
        cp.loadFromBinary("cp_test_full")
        checkPhase(self, cp)

    def test_deserialize_xml(self):
        cp = ContactPhase()
        cp.loadFromXML("cp_test_full.xml", 'ContactPhase')
        checkPhase(self, cp)


class ContactSequenceTest(unittest.TestCase):
    def test_deserialize_text(self):
        cs = ContactSequence()
        cs.loadFromText("cs_test_full.txt")
        self.assertEqual(cs.size(), 10)
        for cp in cs.contactPhases:
            checkPhase(self, cp)

    def test_deserialize_bin(self):
        cs = ContactSequence()
        cs.loadFromBinary("cs_test_full")
        self.assertEqual(cs.size(), 10)
        for cp in cs.contactPhases:
            checkPhase(self, cp)

    def test_deserialize_xml(self):
        cs = ContactSequence()
        cs.loadFromXML("cs_test_full.xml", 'ContactSequence')
        self.assertEqual(cs.size(), 10)
        for cp in cs.contactPhases:
            checkPhase(self, cp)


if __name__ == '__main__':
    unittest.main()
