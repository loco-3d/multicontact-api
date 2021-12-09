# Copyright (c) 2019, CNRS
# Authors: Pierre Fernbach <pfernbac@laas.fr>
import unittest
from math import cos, sin, sqrt
from random import uniform

import numpy as np
from ndcurves import SE3Curve, bezier, piecewise, piecewise_SE3, polynomial
from numpy import array, array_equal, isclose, random

import pinocchio as pin
from multicontact_api import ContactModel, ContactPatch, ContactPhase, ContactSequence, ContactType
from pinocchio import SE3, Quaternion
import pickle
pin.switchToNumpyArray()


def randomQuaternion():
    u1 = uniform(0., 1.)
    u2 = uniform(0., 2. * np.pi)
    u3 = uniform(0., 2. * np.pi)
    a = sqrt(1 - u1)
    b = sqrt(u1)
    q = Quaternion(a * sin(u2), a * cos(u2), b * sin(u3), b * cos(u3))
    q.normalize()
    return q


def createRandomPiecewisePolynomial(dim, t_min=0, t_max=2):
    """
    Build random piecewise polynomial with 2 polynomial of degree 3
    between 01 and 12
    """
    t_mid = (t_min + t_max) / 2.
    coefs0 = np.random.rand(dim, 4)  # degree 3
    pol0 = polynomial(coefs0, t_min, t_mid)
    pc = piecewise(pol0)
    coefs1 = np.random.rand(dim, 4)  # degree 3
    pc.append(polynomial(coefs1, t_mid, t_max))
    return pc


def createRandomSE3Traj(t_min=0, t_max=2):
    p0 = SE3()
    p0.setRandom()
    p1 = SE3()
    p1.setRandom()
    curve = SE3Curve(p0, p1, t_min, t_max)
    return curve


def addRandomPointsValues(cp):
    c_init = np.random.rand(3)
    dc_init = np.random.rand(3)
    ddc_init = np.random.rand(3)
    L_init = np.random.rand(3)
    dL_init = np.random.rand(3)
    q_init = np.random.rand(35)
    c_final = np.random.rand(3)
    dc_final = np.random.rand(3)
    ddc_final = np.random.rand(3)
    L_final = np.random.rand(3)
    dL_final = np.random.rand(3)
    q_final = np.random.rand(35)
    cp.c_init = c_init
    cp.dc_init = dc_init
    cp.ddc_init = ddc_init
    cp.L_init = L_init
    cp.dL_init = dL_init
    cp.q_init = q_init
    cp.c_final = c_final
    cp.dc_final = dc_final
    cp.ddc_final = ddc_final
    cp.L_final = L_final
    cp.dL_final = dL_final
    cp.q_final = q_final


def addRandomCurvesValues(cp):
    q = createRandomPiecewisePolynomial(31)
    dq = createRandomPiecewisePolynomial(30)
    ddq = createRandomPiecewisePolynomial(30)
    tau = createRandomPiecewisePolynomial(30)
    dc = createRandomPiecewisePolynomial(3)
    ddc = createRandomPiecewisePolynomial(3)
    L = createRandomPiecewisePolynomial(3)
    dL = createRandomPiecewisePolynomial(3)
    wrench = createRandomPiecewisePolynomial(6)
    zmp = createRandomPiecewisePolynomial(3)
    root = createRandomSE3Traj()
    coefs = np.random.rand(3, 7)  # degree 3
    c1 = polynomial(coefs, 0, 2)
    # assign trajectories :
    cp.q_t = q
    cp.dq_t = dq
    cp.ddq_t = ddq
    cp.tau_t = tau
    cp.c_t = c1
    cp.dc_t = dc
    cp.ddc_t = ddc
    cp.L_t = L
    cp.dL_t = dL
    cp.wrench_t = wrench
    cp.zmp_t = zmp
    cp.root_t = root


def addRandomContacts(cp):
    p = SE3()
    p.setRandom()
    patchRF = ContactPatch(p, 0.5)
    cp.addContact("right-leg", patchRF)
    p = SE3()
    p.setRandom()
    patchLF = ContactPatch(p, 0.5)
    cp.addContact("left-leg", patchLF)


def addRandomForcesTrajs(cp):
    fR = createRandomPiecewisePolynomial(12)
    fL = createRandomPiecewisePolynomial(12)
    # fL2 = createRandomPiecewisePolynomial(12)
    cp.addContactForceTrajectory("right-leg", fR)
    cp.addContactForceTrajectory("left-leg", fL)
    fR = createRandomPiecewisePolynomial(1)
    fL = createRandomPiecewisePolynomial(1)
    cp.addContactNormalForceTrajectory("right-leg", fR)
    cp.addContactNormalForceTrajectory("left-leg", fL)


def addRandomEffectorTrajectories(cp):
    fR = createRandomSE3Traj()
    fL = createRandomSE3Traj()
    cp.addEffectorTrajectory("right-hand", fR)
    cp.addEffectorTrajectory("left-hand", fL)


def buildRandomContactPhase(min=-1, max=-1):
    if min >= 0 and max >= 0:
        cp = ContactPhase(min, max)
    else:
        cp = ContactPhase()
    addRandomPointsValues(cp)
    addRandomCurvesValues(cp)
    addRandomContacts(cp)
    addRandomForcesTrajs(cp)
    addRandomEffectorTrajectories(cp)
    return cp


class ContactModelTest(unittest.TestCase):
    def test_contact_model(self):
        mu = 0.3
        # default constructor
        mp = ContactModel()
        self.assertEqual(mp.mu, -1.)
        self.assertEqual(mp.contact_type, ContactType.CONTACT_UNDEFINED)
        self.assertEqual(mp.num_contact_points, 1)
        self.assertEqual(len(mp.contact_points_positions.shape), 1)
        self.assertEqual(mp.contact_points_positions.shape[0], 3)
        self.assertTrue(not mp.contact_points_positions.any())

        # constructor with friction
        mp_mu = ContactModel(mu)
        self.assertEqual(mp_mu.mu, mu)
        self.assertEqual(mp_mu.contact_type, ContactType.CONTACT_UNDEFINED)
        self.assertEqual(mp.num_contact_points, 1)
        self.assertEqual(len(mp.contact_points_positions.shape), 1)
        self.assertEqual(mp.contact_points_positions.shape[0], 3)
        self.assertTrue(not mp.contact_points_positions.any())

        # constructor with both values
        mp1 = ContactModel(mu, ContactType.CONTACT_PLANAR)
        # test getter bindings
        self.assertEqual(mp1.mu, mu)
        self.assertEqual(mp1.contact_type, ContactType.CONTACT_PLANAR)
        self.assertEqual(mp.num_contact_points, 1)
        self.assertEqual(len(mp.contact_points_positions.shape), 1)
        self.assertEqual(mp.contact_points_positions.shape[0], 3)
        self.assertTrue(not mp.contact_points_positions.any())

        # copy constructor :
        mp2 = ContactModel(mp1)
        self.assertEqual(mp2.mu, mu)
        self.assertEqual(mp2.contact_type, ContactType.CONTACT_PLANAR)
        self.assertEqual(mp.num_contact_points, 1)
        self.assertEqual(len(mp.contact_points_positions.shape), 1)
        self.assertEqual(mp.contact_points_positions.shape[0], 3)
        self.assertTrue(not mp.contact_points_positions.any())

        # test operator ==
        self.assertTrue(mp1 == mp2)
        mp1.mu = 0.5
        self.assertTrue(mp1 != mp2)

    def test_contact_model_contact_points(self):
        mp1 = ContactModel(0.5, ContactType.CONTACT_PLANAR)
        mp1.num_contact_points = 4
        self.assertEqual(mp1.num_contact_points, 4)
        self.assertEqual(mp1.contact_points_positions.shape[0], 3)
        self.assertEqual(mp1.contact_points_positions.shape[1], 4)
        self.assertTrue(not mp1.contact_points_positions.any())

        pos = np.random.rand(3, 5)
        mp1.contact_points_positions = pos
        self.assertEqual(mp1.num_contact_points, 5)
        self.assertEqual(mp1.contact_points_positions.shape[0], 3)
        self.assertEqual(mp1.contact_points_positions.shape[1], 5)
        self.assertTrue(isclose(mp1.contact_points_positions, pos).all())

        generators = mp1.generatorMatrix()
        self.assertEqual(generators.shape[0], 6)
        self.assertEqual(generators.shape[1], 5*3)

        mp1.num_contact_points = 2
        self.assertEqual(mp1.num_contact_points, 2)
        self.assertEqual(mp1.contact_points_positions.shape[0], 3)
        self.assertEqual(mp1.contact_points_positions.shape[1], 2)
        self.assertTrue(not mp1.contact_points_positions.any())

    def test_contact_model_serialization_default(self):
        mp1 = ContactModel()
        mp1.saveAsText("mp_test.txt")
        mp_txt = ContactModel()
        mp_txt.loadFromText("mp_test.txt")
        self.assertEqual(mp1, mp_txt)
        mp1.saveAsBinary("mp_test")
        mp_bin = ContactModel()
        mp_bin.loadFromBinary("mp_test")
        self.assertEqual(mp1, mp_bin)
        mp1.saveAsXML("mp_test.xml", 'ContactModel')
        mp_xml = ContactModel()
        mp_xml.loadFromXML("mp_test.xml", 'ContactPatch')
        self.assertEqual(mp1, mp_xml)
        mp_pickled = pickle.dumps(mp1)
        mp_from_pickle = pickle.loads(mp_pickled)
        self.assertEqual(mp1, mp_from_pickle)

    def test_contact_model_serialization_full(self):
        mu = 0.3
        # constructor with both values
        mp1 = ContactModel(mu, ContactType.CONTACT_PLANAR)
        mp1.saveAsText("mp_test.txt")
        mp_txt = ContactModel()
        mp_txt.loadFromText("mp_test.txt")
        self.assertEqual(mp1, mp_txt)
        mp1.saveAsBinary("mp_test")
        mp_bin = ContactModel()
        mp_bin.loadFromBinary("mp_test")
        self.assertEqual(mp1, mp_bin)
        mp1.saveAsXML("mp_test.xml", 'ContactModel')
        mp_xml = ContactModel()
        mp_xml.loadFromXML("mp_test.xml", 'ContactPatch')
        self.assertEqual(mp1, mp_xml)
        mp_pickled = pickle.dumps(mp1)
        mp_from_pickle = pickle.loads(mp_pickled)
        self.assertEqual(mp1, mp_from_pickle)


class ContactPatchTest(unittest.TestCase):
    def test_default_constructor(self):
        cp = ContactPatch()
        self.assertEqual(cp.friction, -1.0)
        self.assertTrue(cp.placement == SE3.Identity())

    def test_setter_getter(self):
        cp = ContactPatch()
        p = SE3()
        p.setRandom()
        cp.placement = p
        self.assertTrue(cp.placement == p)
        cp.friction = 0.7
        self.assertTrue(cp.friction == 0.7)
        self.assertTrue(cp.placement == p)

    def test_constructor_with_arguments(self):
        p = SE3()
        p.setRandom()
        cp = ContactPatch(p)
        self.assertTrue(cp.friction == -1.0)
        self.assertTrue(cp.placement == p)
        # check that the value have been copied and it's not the same pointer anymore :
        p.setRandom()
        self.assertTrue(cp.placement != p)

        p = SE3()
        p.setRandom()
        cp = ContactPatch(p, 0.9)
        self.assertTrue(cp.friction == 0.9)
        self.assertTrue(cp.placement == p)

    def test_constructor_with_contact_model(self):
        cm = ContactModel(0.5, ContactType.CONTACT_PLANAR)
        cm.num_contact_points = 4
        p = SE3()
        p.setRandom()
        cp = ContactPatch(p, cm)
        self.assertTrue(cp.friction == 0.5)
        self.assertTrue(cp.placement == p)
        self.assertTrue(cp.contact_model.num_contact_points == 4)

        # check that the value have been copied and it's not the same pointer anymore :
        cm.num_contact_points = 6
        self.assertTrue(cp.contact_model.num_contact_points == 4)

    def test_operator_equal(self):
        cp1 = ContactPatch()
        cp2 = ContactPatch()
        self.assertTrue(cp1 == cp2)
        self.assertFalse(cp1 != cp2)
        cp1.friction = 0.5
        self.assertTrue(cp1 != cp2)
        self.assertFalse(cp1 == cp2)
        cp2.friction = 0.5
        self.assertTrue(cp1 == cp2)

        p = SE3()
        p.setRandom()
        cp1 = ContactPatch(p, 0.9)
        cp2 = ContactPatch(p, 0.9)
        self.assertTrue(cp1 == cp2)
        cp1.placement.setRandom()
        self.assertTrue(cp1 != cp2)

    def test_copy_constructor(self):
        p = SE3()
        p.setRandom()
        cp1 = ContactPatch(p, 0.9)
        cp2 = ContactPatch(cp1)
        self.assertTrue(cp1 == cp2)
        cp1.placement.setRandom()
        self.assertTrue(cp1 != cp2)

    def test_serialization_no_friction(self):
        p = SE3()
        p.setRandom()
        cp1 = ContactPatch(p)
        cp1.saveAsText("cp_test.txt")
        cp_txt = ContactPatch()
        cp_txt.loadFromText("cp_test.txt")
        self.assertEqual(cp1, cp_txt)
        cp1.saveAsBinary("cp_test")
        cp_bin = ContactPatch()
        cp_bin.loadFromBinary("cp_test")
        self.assertEqual(cp1, cp_bin)
        cp1.saveAsXML("cp_test.xml", 'ContactPatch')
        cp_xml = ContactPatch()
        cp_xml.loadFromXML("cp_test.xml", 'ContactPatch')
        self.assertEqual(cp1, cp_xml)
        cp_pickled = pickle.dumps(cp1)
        cp_from_pickle = pickle.loads(cp_pickled)
        self.assertEqual(cp1, cp_from_pickle)

    def test_serialization_full(self):
        p = SE3()
        p.setRandom()
        cp1 = ContactPatch(p, 0.9)
        cp1.saveAsText("cp_test.txt")
        cp_txt = ContactPatch()
        cp_txt.loadFromText("cp_test.txt")
        self.assertEqual(cp1, cp_txt)
        cp1.saveAsBinary("cp_test")
        cp_bin = ContactPatch()
        cp_bin.loadFromBinary("cp_test")
        self.assertEqual(cp1, cp_bin)
        cp1.saveAsXML("cp_test.xml", 'ContactPatch')
        cp_xml = ContactPatch()
        cp_xml.loadFromXML("cp_test.xml", 'ContactPatch')
        self.assertEqual(cp1, cp_xml)
        cp_pickled = pickle.dumps(cp1)
        cp_from_pickle = pickle.loads(cp_pickled)
        self.assertEqual(cp1, cp_from_pickle)

    def test_contact_patch_model_accessor(self):
        p = SE3()
        p.setRandom()
        cp1 = ContactPatch(p, 0.9)
        cm = cp1.contact_model
        self.assertEqual(cm.mu, 0.9)
        cm.mu = 0.5
        self.assertEqual(cp1.friction, 0.5)

        cp1.contact_model.contact_type = ContactType.CONTACT_PLANAR
        self.assertEqual(cp1.contact_model.contact_type, ContactType.CONTACT_PLANAR)

        cp1.friction = 2
        self.assertEqual(cp1.contact_model.mu, 2)
        self.assertEqual(cm.mu, 2)

        pos = np.random.rand(3, 4)
        cp1.contact_model.contact_points_positions = pos
        self.assertEqual(cp1.contact_model.num_contact_points, 4)
        self.assertEqual(cp1.contact_model.contact_points_positions.shape[0], 3)
        self.assertEqual(cp1.contact_model.contact_points_positions.shape[1], 4)
        self.assertTrue(isclose(cp1.contact_model.contact_points_positions, pos).all())


class ContactPhaseTest(unittest.TestCase):
    def test_default_constructor(self):
        cp = ContactPhase()
        self.assertEqual(cp.timeInitial, -1)
        self.assertEqual(cp.timeFinal, -1)
        self.assertEqual(cp.duration, 0)
        self.assertEqual(cp.numContacts(), 0)
        self.assertEqual(len(cp.effectorsInContact()), 0)

    def test_constructor_with_arguments(self):
        cp = ContactPhase(1, 5)
        self.assertEqual(cp.timeInitial, 1)
        self.assertEqual(cp.timeFinal, 5)
        self.assertEqual(cp.duration, 4)
        self.assertEqual(cp.numContacts(), 0)
        self.assertEqual(len(cp.effectorsInContact()), 0)
        with self.assertRaises(ValueError):
            cp = ContactPhase(1, 0.5)

    def test_timings_setter(self):
        cp = ContactPhase()
        cp.timeInitial = 1.5
        cp.timeFinal = 3.
        self.assertEqual(cp.timeInitial, 1.5)
        self.assertEqual(cp.timeFinal, 3.)
        self.assertEqual(cp.duration, 1.5)
        cp.duration = 2.
        self.assertEqual(cp.timeInitial, 1.5)
        self.assertEqual(cp.timeFinal, 3.5)
        self.assertEqual(cp.duration, 2.)
        with self.assertRaises(ValueError):
            cp.timeFinal = 1.
        with self.assertRaises(ValueError):
            cp.duration = -0.5

    def test_contact_methods(self):
        cp = ContactPhase(1.5, 3)
        p = SE3()
        p.setRandom()
        patchRF = ContactPatch(p, 0.5)
        new = cp.addContact("right-leg", patchRF)
        self.assertTrue(new)
        self.assertTrue(cp.isEffectorInContact("right-leg"))
        self.assertTrue("right-leg" in cp.effectorsInContact())
        self.assertEqual(patchRF, cp.contactPatch("right-leg"))
        self.assertEqual(cp.numContacts(), 1)

        # add another contact :
        p = SE3()
        p.setRandom()
        patchLF = ContactPatch(p, 0.5)
        new = cp.addContact("left-leg", patchLF)
        self.assertTrue(new)
        self.assertTrue(cp.isEffectorInContact("right-leg"))
        self.assertTrue("right-leg" in cp.effectorsInContact())
        self.assertEqual(patchRF, cp.contactPatch("right-leg"))
        self.assertTrue(cp.isEffectorInContact("left-leg"))
        self.assertTrue("left-leg" in cp.effectorsInContact())
        self.assertEqual(patchLF, cp.contactPatch("left-leg"))
        self.assertEqual(cp.numContacts(), 2)
        # check that the patch can be overwritten:
        p = SE3()
        p.setRandom()
        patchRF2 = ContactPatch(p, 0.5)
        new = cp.addContact("right-leg", patchRF2)
        self.assertFalse(new)

        # check deletion of contacts :
        exist = cp.removeContact("right-leg")
        self.assertTrue(exist)
        self.assertTrue(cp.isEffectorInContact("left-leg"))
        self.assertTrue("left-leg" in cp.effectorsInContact())
        self.assertEqual(patchLF, cp.contactPatch("left-leg"))
        self.assertEqual(cp.numContacts(), 1)
        self.assertFalse(cp.isEffectorInContact("right-leg"))
        self.assertFalse("right-leg" in cp.effectorsInContact())
        exist = cp.removeContact("right-leg")
        self.assertFalse(exist)

        exist = cp.removeContact("left-leg")
        self.assertTrue(exist)
        self.assertFalse(cp.isEffectorInContact("left-leg"))
        self.assertFalse("left-leg" in cp.effectorsInContact())
        self.assertFalse(cp.isEffectorInContact("right-leg"))
        self.assertFalse("right-leg" in cp.effectorsInContact())
        self.assertEqual(cp.numContacts(), 0)

    def test_contact_patch_access(self):
        cp = ContactPhase(1.5, 3)
        p = SE3()
        p.setRandom()
        patchRF = ContactPatch(p, 0.5)
        cp.addContact("right-leg", patchRF)
        # check that the contactPatch have been copied and it's not a pointer :
        patchRF.placement.setRandom()
        self.assertNotEqual(patchRF, cp.contactPatch("right-leg"))
        patchRF = ContactPatch(cp.contactPatch("right-leg"))
        self.assertEqual(patchRF, cp.contactPatch("right-leg"))
        patchRF.placement.translation += np.array([0, 0.1, 0])
        self.assertNotEqual(patchRF, cp.contactPatch("right-leg"))
        patchRF = ContactPatch(cp.contactPatch("right-leg"))
        # check that the getter of contactPatch is a non const reference:
        cp.contactPatch('right-leg').placement.setRandom()
        self.assertNotEqual(patchRF, cp.contactPatch("right-leg"))
        patchRF = ContactPatch(cp.contactPatch("right-leg"))
        cp.contactPatch('right-leg').friction = 0.7
        self.assertNotEqual(patchRF, cp.contactPatch("right-leg"))
        patchRF = ContactPatch(cp.contactPatch("right-leg"))
        cp.contactPatch("right-leg").placement.translation += np.array([0, 0.1, 0])
        self.assertNotEqual(patchRF, cp.contactPatch("right-leg"))

        patchRF = cp.contactPatch("right-leg")
        self.assertEqual(patchRF, cp.contactPatch("right-leg"))
        patchRF.placement.translation += np.array([0, 0.1, 0])
        self.assertEqual(patchRF, cp.contactPatch("right-leg"))
        # check errors :
        with self.assertRaises(ValueError):
            cp.contactPatch("left-leg")

    def test_contact_patch_dict(self):
        cp = ContactPhase(1.5, 3)
        p = SE3()
        p.setRandom()
        patchRF = ContactPatch(p, 0.5)
        cp.addContact("right-leg", patchRF)
        dict = cp.contactPatches()
        self.assertTrue("right-leg" in dict.keys())
        self.assertEqual(dict["right-leg"], patchRF)
        self.assertEqual(len(dict.keys()), 1)

        # add another contact :
        p = SE3()
        p.setRandom()
        patchLF = ContactPatch(p, 0.5)
        cp.addContact("left-leg", patchLF)
        # check that it's not a pointer :
        self.assertEqual(len(dict.keys()), 1)
        self.assertFalse("left-leg" in dict.keys())
        # check that the contact have been added
        dict = cp.contactPatches()
        self.assertTrue("right-leg" in dict.keys())
        self.assertTrue("left-leg" in dict.keys())
        self.assertEqual(dict["right-leg"], patchRF)
        self.assertEqual(dict["left-leg"], patchLF)
        self.assertEqual(len(dict.keys()), 2)

        # check that changing the dict doesn't change the contact phase:
        p = SE3()
        p.setRandom()
        patch2 = ContactPatch(p, 0.5)
        dict.update({"test": patch2})
        self.assertFalse("test" in cp.contactPatches().keys())
        # check that the map is const
        cp.contactPatches().update({"test": patch2})  # should not have any effect
        self.assertFalse("test" in cp.contactPatches().keys())

        # check deletion :
        cp.removeContact("right-leg")
        dict = cp.contactPatches()
        self.assertFalse("right-leg" in dict.keys())
        self.assertTrue("left-leg" in dict.keys())
        self.assertEqual(dict["left-leg"], patchLF)
        self.assertEqual(len(dict.keys()), 1)

    def test_effector_trajectory(self):
        cp = ContactPhase(1.5, 3)
        p = SE3()
        p.setRandom()
        patchRF = ContactPatch(p, 0.5)
        cp.addContact("right-leg", patchRF)
        # create a SE3 trajectory :
        init_pose = SE3.Identity()
        end_pose = SE3.Identity()
        init_pose.translation = array([0.2, -0.7, 0.6])
        end_pose.translation = array([3.6, -2.2, -0.9])
        init_pose.rotation = Quaternion.Identity().normalized().matrix()
        end_pose.rotation = Quaternion(sqrt(2.) / 2., sqrt(2.) / 2., 0, 0).normalized().matrix()
        effL = SE3Curve(init_pose, end_pose, 0.5, 2.5)
        # add the trajectory to the contact phase :
        new = cp.addEffectorTrajectory("left-leg", effL)
        self.assertTrue(new)
        self.assertTrue(cp.effectorHaveAtrajectory("left-leg"))
        self.assertTrue("left-leg" in cp.effectorsWithTrajectory())
        self.assertEqual(cp.effectorTrajectory("left-leg"), effL)
        self.assertEqual(cp.effectorTrajectory("left-leg").min(), 0.5)
        self.assertEqual(cp.effectorTrajectory("left-leg").max(), 2.5)
        self.assertTrue(cp.effectorTrajectory("left-leg").evaluateAsSE3(0.5).isApprox(init_pose))
        self.assertTrue(cp.effectorTrajectory("left-leg").evaluateAsSE3(2.5).isApprox(end_pose))

        # check with piecewise SE3
        effH = piecewise_SE3(effL)
        end_pose2 = SE3.Identity()
        end_pose2.translation = array([-4.9, 0.8, 0.9])
        end_pose2.rotation = Quaternion(sqrt(2.) / 2., 0., sqrt(2.) / 2., 0).normalized().matrix()
        effH.append(end_pose2, 4.)
        new = cp.addEffectorTrajectory("hand", effH)
        self.assertTrue(new)
        self.assertTrue(cp.effectorHaveAtrajectory("left-leg"))
        self.assertTrue("left-leg" in cp.effectorsWithTrajectory())
        self.assertTrue(cp.effectorHaveAtrajectory("hand"))
        self.assertTrue("hand" in cp.effectorsWithTrajectory())
        self.assertEqual(cp.effectorTrajectory("left-leg"), effL)
        self.assertEqual(cp.effectorTrajectory("hand"), effH)
        self.assertEqual(cp.effectorTrajectory("hand").min(), 0.5)
        self.assertEqual(cp.effectorTrajectory("hand").max(), 4.)
        self.assertTrue(cp.effectorTrajectory("hand").evaluateAsSE3(0.5).isApprox(init_pose))
        self.assertTrue(cp.effectorTrajectory("hand").evaluateAsSE3(4).isApprox(end_pose2))

        # check that the getter return a pointer to a non const object :
        end_pose3 = SE3.Identity()
        end_pose3.setRandom()
        cp.effectorTrajectory("hand").append(end_pose3, 6.5)
        self.assertEqual(cp.effectorTrajectory("hand").max(), 6.5)
        self.assertTrue(cp.effectorTrajectory("hand").evaluateAsSE3(6.5).isApprox(end_pose3))

        effH = cp.effectorTrajectory("hand")
        end_pose4 = SE3.Identity()
        end_pose4.setRandom()
        effH.append(end_pose4, 10.)
        self.assertEqual(cp.effectorTrajectory("hand").max(), 10.)
        self.assertTrue(cp.effectorTrajectory("hand").evaluateAsSE3(10.).isApprox(end_pose4))

        # check errors :
        with self.assertRaises(ValueError):
            cp.addEffectorTrajectory("right-leg", effL)

        # check that we cannot add other kind of trajectories than SE3 :
        waypoints = array([[1., 2., 3.], [4., 5., 6.]]).transpose()
        a = bezier(waypoints, 0., 1.)
        with self.assertRaises(BaseException):
            cp.addEffectorTrajectory("other-leg", a)

    def test_effector_trajectory_dict(self):
        cp = ContactPhase(1.5, 3)
        p = SE3()
        p.setRandom()
        patchRF = ContactPatch(p, 0.5)
        cp.addContact("right-leg", patchRF)
        # create a SE3 trajectory :
        init_pose = SE3.Identity()
        end_pose = SE3.Identity()
        init_pose.translation = array([0.2, -0.7, 0.6])
        end_pose.translation = array([3.6, -2.2, -0.9])
        init_pose.rotation = Quaternion.Identity().normalized().matrix()
        end_pose.rotation = Quaternion(sqrt(2.) / 2., sqrt(2.) / 2., 0, 0).normalized().matrix()
        effL = SE3Curve(init_pose, end_pose, 0.5, 2.5)
        # add the trajectory to the contact phase :
        cp.addEffectorTrajectory("left-leg", effL)
        dict = cp.effectorTrajectories()
        self.assertEqual(len(dict.keys()), 1)
        self.assertTrue("left-leg" in dict.keys())
        self.assertEqual(dict["left-leg"], effL)
        self.assertEqual(dict["left-leg"].min(), 0.5)
        self.assertEqual(dict["left-leg"].max(), 2.5)
        self.assertTrue(dict["left-leg"].evaluateAsSE3(0.5).isApprox(init_pose))
        self.assertTrue(dict["left-leg"].evaluateAsSE3(2.5).isApprox(end_pose))

        # check that changing the dict doesn't change the contact phase:
        effH = piecewise_SE3(effL)
        end_pose2 = SE3.Identity()
        end_pose2.translation = array([-4.9, 0.8, 0.9])
        end_pose2.rotation = Quaternion(sqrt(2.) / 2., 0., sqrt(2.) / 2., 0).normalized().matrix()
        effH.append(end_pose2, 4.)
        dict.update({"hand": effH})
        self.assertFalse("hand" in cp.effectorTrajectories().keys())
        # check that the map is const
        cp.effectorTrajectories().update({"hand": effH})  # should not have any effect
        self.assertFalse("hand" in cp.effectorTrajectories().keys())

    def test_contact_force_trajectory(self):
        # create phase and add two contacts
        cp = ContactPhase(1.5, 3)
        p = SE3()
        p.setRandom()
        cp.addContact("right-leg", ContactPatch(p, 0.5))
        p = SE3()
        p.setRandom()
        cp.addContact("left-leg", ContactPatch(p, 0.5))
        # create a polynomial 12D trajectory
        fR = createRandomPiecewisePolynomial(12)
        fL = createRandomPiecewisePolynomial(12)
        new = cp.addContactForceTrajectory("right-leg", fR)
        self.assertTrue(new)
        self.assertEqual(cp.contactForce("right-leg"), fR)
        self.assertEqual(cp.contactForce("right-leg").min(), 0)
        self.assertEqual(cp.contactForce("right-leg").max(), 2.)
        self.assertTrue(array_equal(cp.contactForce("right-leg")(0.5), fR(0.5)))
        self.assertTrue(array_equal(cp.contactForce("right-leg")(1.5), fR(1.5)))

        new = cp.addContactForceTrajectory("left-leg", fL)
        self.assertTrue(new)
        self.assertEqual(cp.contactForce("left-leg"), fL)
        self.assertEqual(cp.contactForce("left-leg").min(), 0)
        self.assertEqual(cp.contactForce("left-leg").max(), 2.)
        self.assertTrue(array_equal(cp.contactForce("left-leg")(0.5), fL(0.5)))
        self.assertTrue(array_equal(cp.contactForce("left-leg")(1.5), fL(1.5)))

        new = cp.addContactForceTrajectory("left-leg", fL)
        self.assertFalse(new)

        # check that the getter return a pointer to a non const object :
        cp.contactForce("left-leg").append(np.random.rand(12, 1), 3.5)
        self.assertEqual(cp.contactForce("left-leg").max(), 3.5)

        pc = cp.contactForce("left-leg")
        pc.append(np.random.rand(12, 1), 6.)
        self.assertEqual(cp.contactForce("left-leg").max(), 6.)
        self.assertTrue(array_equal(cp.contactForce("left-leg")(6.), pc(6.)))

        # check errors :
        with self.assertRaises(ValueError):
            cp.addContactForceTrajectory("hand", fL)

    def test_contact_force_trajectory_dict(self):
        # create phase and add two contacts
        cp = ContactPhase(1.5, 3)
        p = SE3()
        p.setRandom()
        cp.addContact("right-leg", ContactPatch(p, 0.5))
        p = SE3()
        p.setRandom()
        cp.addContact("left-leg", ContactPatch(p, 0.5))
        # create a polynomial 12D trajectory
        fR = createRandomPiecewisePolynomial(12)
        fL = createRandomPiecewisePolynomial(12)
        cp.addContactForceTrajectory("right-leg", fR)
        dict = cp.contactForces()
        self.assertEqual(len(dict.keys()), 1)
        self.assertTrue("right-leg" in dict.keys())
        self.assertEqual(dict["right-leg"], fR)
        self.assertEqual(dict["right-leg"].min(), 0)
        self.assertEqual(dict["right-leg"].max(), 2.)
        self.assertTrue(array_equal(dict["right-leg"](0.5), fR(0.5)))
        self.assertTrue(array_equal(dict["right-leg"](1.5), fR(1.5)))

        cp.addContactForceTrajectory("left-leg", fL)
        self.assertEqual(len(dict.keys()), 1)
        self.assertTrue("right-leg" in dict.keys())
        self.assertFalse("left-leg" in dict.keys())
        dict = cp.contactForces()
        self.assertEqual(len(dict.keys()), 2)
        self.assertTrue("right-leg" in dict.keys())
        self.assertTrue("left-leg" in dict.keys())

        # check that changing the dict doesn"t change the contact phase
        f2 = createRandomPiecewisePolynomial(12)
        dict.update({"hand": f2})
        self.assertFalse("hand" in cp.contactForces().keys())
        # check that the map is const
        cp.contactForces().update({"hand": f2})  # should not have any effect
        self.assertFalse("hand" in cp.contactForces().keys())

    def test_contact_normal_force_trajectory(self):
        # create phase and add two contacts
        cp = ContactPhase(1.5, 3)
        p = SE3()
        p.setRandom()
        cp.addContact("right-leg", ContactPatch(p, 0.5))
        p = SE3()
        p.setRandom()
        cp.addContact("left-leg", ContactPatch(p, 0.5))
        # create a polynomial 12D trajectory
        fR = createRandomPiecewisePolynomial(1)
        fL = createRandomPiecewisePolynomial(1)
        new = cp.addContactNormalForceTrajectory("right-leg", fR)
        self.assertTrue(new)
        self.assertEqual(cp.contactNormalForce("right-leg"), fR)
        self.assertEqual(cp.contactNormalForce("right-leg").min(), 0)
        self.assertEqual(cp.contactNormalForce("right-leg").max(), 2.)
        self.assertTrue(array_equal(cp.contactNormalForce("right-leg")(0.5), fR(0.5)))
        self.assertTrue(array_equal(cp.contactNormalForce("right-leg")(1.5), fR(1.5)))

        new = cp.addContactNormalForceTrajectory("left-leg", fL)
        self.assertTrue(new)
        self.assertEqual(cp.contactNormalForce("left-leg"), fL)
        self.assertEqual(cp.contactNormalForce("left-leg").min(), 0)
        self.assertEqual(cp.contactNormalForce("left-leg").max(), 2.)
        self.assertTrue(array_equal(cp.contactNormalForce("left-leg")(0.5), fL(0.5)))
        self.assertTrue(array_equal(cp.contactNormalForce("left-leg")(1.5), fL(1.5)))

        new = cp.addContactNormalForceTrajectory("left-leg", fL)
        self.assertFalse(new)

        # check that the getter return a pointer to a non const object :
        cp.contactNormalForce("left-leg").append(np.random.rand(1, 1), 3.5)
        self.assertEqual(cp.contactNormalForce("left-leg").max(), 3.5)

        pc = cp.contactNormalForce("left-leg")
        pc.append(np.random.rand(1, 1), 6.)
        self.assertEqual(cp.contactNormalForce("left-leg").max(), 6.)
        self.assertTrue(array_equal(cp.contactNormalForce("left-leg")(6.), pc(6.)))

        # check errors :
        with self.assertRaises(ValueError):
            cp.addContactNormalForceTrajectory("hand", fL)

        fL = createRandomPiecewisePolynomial(3)
        with self.assertRaises(ValueError):
            cp.addContactNormalForceTrajectory("left-leg", fL)

    def test_contact_normal_force_trajectory_dict(self):
        # create phase and add two contacts
        cp = ContactPhase(1.5, 3)
        p = SE3()
        p.setRandom()
        cp.addContact("right-leg", ContactPatch(p, 0.5))
        p = SE3()
        p.setRandom()
        cp.addContact("left-leg", ContactPatch(p, 0.5))
        # create a polynomial 12D trajectory
        fR = createRandomPiecewisePolynomial(1)
        fL = createRandomPiecewisePolynomial(1)
        cp.addContactNormalForceTrajectory("right-leg", fR)
        dict = cp.contactNormalForces()
        self.assertEqual(len(dict.keys()), 1)
        self.assertTrue("right-leg" in dict.keys())
        self.assertEqual(dict["right-leg"], fR)
        self.assertEqual(dict["right-leg"].min(), 0)
        self.assertEqual(dict["right-leg"].max(), 2.)
        self.assertTrue(array_equal(dict["right-leg"](0.5), fR(0.5)))
        self.assertTrue(array_equal(dict["right-leg"](1.5), fR(1.5)))

        cp.addContactNormalForceTrajectory("left-leg", fL)
        self.assertEqual(len(dict.keys()), 1)
        self.assertTrue("right-leg" in dict.keys())
        self.assertFalse("left-leg" in dict.keys())
        dict = cp.contactNormalForces()
        self.assertEqual(len(dict.keys()), 2)
        self.assertTrue("right-leg" in dict.keys())
        self.assertTrue("left-leg" in dict.keys())

        # check that changing the dict doesn"t change the contact phase
        f2 = createRandomPiecewisePolynomial(1)
        dict.update({"hand": f2})
        self.assertFalse("hand" in cp.contactNormalForces().keys())
        # check that the map is const
        cp.contactNormalForces().update({"hand": f2})  # should not have any effect
        self.assertFalse("hand" in cp.contactNormalForces().keys())

    def test_members_points(self):
        cp = ContactPhase()
        # check default values :
        self.assertTrue(array_equal(np.zeros(3), cp.c_init))
        self.assertTrue(array_equal(np.zeros(3), cp.dc_init))
        self.assertTrue(array_equal(np.zeros(3), cp.ddc_init))
        self.assertTrue(array_equal(np.zeros(3), cp.L_init))
        self.assertTrue(array_equal(np.zeros(3), cp.dL_init))
        self.assertTrue(array_equal(np.zeros(3), cp.c_final))
        self.assertTrue(array_equal(np.zeros(3), cp.dc_final))
        self.assertTrue(array_equal(np.zeros(3), cp.ddc_final))
        self.assertTrue(array_equal(np.zeros(3), cp.L_final))
        self.assertTrue(array_equal(np.zeros(3), cp.dL_final))
        # set random values :
        c_init = np.random.rand(3)
        dc_init = np.random.rand(3)
        ddc_init = np.random.rand(3)
        L_init = np.random.rand(3)
        dL_init = np.random.rand(3)
        q_init = np.random.rand(35)
        c_final = np.random.rand(3)
        dc_final = np.random.rand(3)
        ddc_final = np.random.rand(3)
        L_final = np.random.rand(3)
        dL_final = np.random.rand(3)
        q_final = np.random.rand(35)
        cp.c_init = c_init
        cp.dc_init = dc_init
        cp.ddc_init = ddc_init
        cp.L_init = L_init
        cp.dL_init = dL_init
        cp.q_init = q_init
        cp.c_final = c_final
        cp.dc_final = dc_final
        cp.ddc_final = ddc_final
        cp.L_final = L_final
        cp.dL_final = dL_final
        cp.q_final = q_final
        self.assertTrue(array_equal(cp.c_init, c_init))
        self.assertTrue(array_equal(cp.dc_init, dc_init))
        self.assertTrue(array_equal(cp.ddc_init, ddc_init))
        self.assertTrue(array_equal(cp.L_init, L_init))
        self.assertTrue(array_equal(cp.dL_init, dL_init))
        self.assertTrue(array_equal(cp.q_init, q_init))
        self.assertTrue(array_equal(cp.c_final, c_final))
        self.assertTrue(array_equal(cp.dc_final, dc_final))
        self.assertTrue(array_equal(cp.ddc_final, ddc_final))
        self.assertTrue(array_equal(cp.L_final, L_final))
        self.assertTrue(array_equal(cp.dL_final, dL_final))
        self.assertTrue(array_equal(cp.q_final, q_final))
        # check that it's not a pointer :
        ci = cp.c_init
        ci = np.random.rand(3)
        self.assertFalse(array_equal(cp.c_init, ci))
        # it's a copy (limitation from eigenpy ...) :
        dc_init = cp.dc_init.copy()
        cp.dc_init += np.array([0.1, 0., -2.])  # this work as += call the setter
        self.assertFalse(array_equal(cp.dc_init, dc_init))
        dc_init = cp.dc_init.copy()
        cp.dc_init[2] = 0.  # this line have no effect
        self.assertTrue(array_equal(cp.dc_init, dc_init))

        # check error due to incorrect dimensions :
        print("# Expected warning messages about dimension / column vector : ")
        with self.assertRaises(BaseException):
            cp.c_init = np.random.rand(4)
        with self.assertRaises(BaseException):
            cp.dc_init = np.random.rand(2)
        with self.assertRaises(BaseException):
            cp.ddc_init = np.random.rand(10)
        with self.assertRaises(BaseException):
            cp.L_init = np.random.rand(1)
        with self.assertRaises(BaseException):
            cp.dL_init = np.random.rand(4)
        with self.assertRaises(BaseException):
            cp.c_final = np.random.rand(5)
        with self.assertRaises(BaseException):
            cp.dc_final = np.random.rand(3, 2)
        with self.assertRaises(BaseException):
            cp.ddc_final = np.random.rand(3, 3)
        with self.assertRaises(BaseException):
            cp.L_final = np.random.rand(1, 2)
        with self.assertRaises(BaseException):
            cp.dL_final = np.random.rand(1, 3)
        print("# End of Expected warning messages.")

    def test_member_curves(self):
        cp = ContactPhase()
        # check default values :
        self.assertIsNone(cp.q_t)
        self.assertIsNone(cp.dq_t)
        self.assertIsNone(cp.ddq_t)
        self.assertIsNone(cp.tau_t)
        self.assertIsNone(cp.c_t)
        self.assertIsNone(cp.dc_t)
        self.assertIsNone(cp.ddc_t)
        self.assertIsNone(cp.L_t)
        self.assertIsNone(cp.dL_t)
        self.assertIsNone(cp.wrench_t)
        self.assertIsNone(cp.zmp_t)
        self.assertIsNone(cp.root_t)
        # build random trajectories :
        q = createRandomPiecewisePolynomial(31)
        dq = createRandomPiecewisePolynomial(30)
        ddq = createRandomPiecewisePolynomial(30)
        tau = createRandomPiecewisePolynomial(30)
        c = createRandomPiecewisePolynomial(3)
        dc = createRandomPiecewisePolynomial(3)
        ddc = createRandomPiecewisePolynomial(3)
        L = createRandomPiecewisePolynomial(3)
        dL = createRandomPiecewisePolynomial(3)
        wrench = createRandomPiecewisePolynomial(6)
        zmp = createRandomPiecewisePolynomial(3)
        root = createRandomSE3Traj()
        # assign trajectories :
        cp.q_t = q
        cp.dq_t = dq
        cp.ddq_t = ddq
        cp.tau_t = tau
        cp.c_t = c
        cp.dc_t = dc
        cp.ddc_t = ddc
        cp.L_t = L
        cp.dL_t = dL
        cp.wrench_t = wrench
        cp.zmp_t = zmp
        cp.root_t = root
        # check getter :
        self.assertEqual(cp.q_t, q)
        self.assertEqual(cp.dq_t, dq)
        self.assertEqual(cp.ddq_t, ddq)
        self.assertEqual(cp.tau_t, tau)
        self.assertEqual(cp.c_t, c)
        self.assertEqual(cp.dc_t, dc)
        self.assertEqual(cp.ddc_t, ddc)
        self.assertEqual(cp.L_t, L)
        self.assertEqual(cp.dL_t, dL)
        self.assertEqual(cp.wrench_t, wrench)
        self.assertEqual(cp.zmp_t, zmp)
        self.assertEqual(cp.root_t, root)
        for t in np.linspace(0., 2., 10):
            self.assertTrue(array_equal(cp.q_t(t), q(t)))
            self.assertTrue(array_equal(cp.dq_t(t), dq(t)))
            self.assertTrue(array_equal(cp.ddq_t(t), ddq(t)))
            self.assertTrue(array_equal(cp.tau_t(t), tau(t)))
            self.assertTrue(array_equal(cp.c_t(t), c(t)))
            self.assertTrue(array_equal(cp.dc_t(t), dc(t)))
            self.assertTrue(array_equal(cp.ddc_t(t), ddc(t)))
            self.assertTrue(array_equal(cp.L_t(t), L(t)))
            self.assertTrue(array_equal(cp.dL_t(t), dL(t)))
            self.assertTrue(array_equal(cp.wrench_t(t), wrench(t)))
            self.assertTrue(array_equal(cp.zmp_t(t), zmp(t)))
            self.assertTrue(array_equal(cp.root_t(t), root(t)))
            self.assertEqual(cp.root_t.evaluateAsSE3(t), root.evaluateAsSE3(t))

        # check that deleting python variables doesn't delete members after assignement:
        del q
        self.assertIsNotNone(cp.q_t)
        self.assertEqual(cp.q_t.min(), 0)
        self.assertEqual(cp.q_t.max(), 2)
        self.assertIsNotNone(cp.q_t(1.))
        c = None
        self.assertIsNotNone(cp.c_t)
        self.assertEqual(cp.c_t.min(), 0)
        self.assertEqual(cp.c_t.max(), 2)
        self.assertIsNotNone(cp.c_t(1.))
        # check that curve have not been copied and that it's the same pointer
        dc.append(np.random.rand(3, 1), 3.5)
        self.assertEqual(cp.dc_t.min(), 0)
        self.assertEqual(cp.dc_t.max(), 3.5)
        self.assertEqual(cp.dc_t, dc)
        # check that the return of the getter is not const :
        cp.dq_t.append(np.random.rand(30, 1), 4.)
        self.assertEqual(cp.dq_t.min(), 0)
        self.assertEqual(cp.dq_t.max(), 4.)
        self.assertEqual(cp.dq_t, dq)

    def test_operator_equal(self):
        cp1 = ContactPhase()
        cp2 = ContactPhase()
        # check timings
        self.assertTrue(cp1 == cp2)
        cp1.timeInitial = 1.
        self.assertTrue(cp1 != cp2)
        cp2.timeInitial = 1.
        self.assertTrue(cp1 == cp2)
        cp1.timeFinal = 3.5
        self.assertTrue(cp1 != cp2)
        cp2.duration = 2.5
        self.assertTrue(cp1 == cp2)
        # check public members :
        # points :
        c_init = np.random.rand(3)
        dc_init = np.random.rand(3)
        ddc_init = np.random.rand(3)
        L_init = np.random.rand(3)
        dL_init = np.random.rand(3)
        q_init = np.random.rand(35)
        c_final = np.random.rand(3)
        dc_final = np.random.rand(3)
        ddc_final = np.random.rand(3)
        L_final = np.random.rand(3)
        dL_final = np.random.rand(3)
        q_final = np.random.rand(35)
        cp1.c_init = c_init
        self.assertTrue(cp1 != cp2)
        cp2.c_init = c_init
        self.assertTrue(cp1 == cp2)
        cp1.dc_init = dc_init
        self.assertTrue(cp1 != cp2)
        cp2.dc_init = dc_init
        self.assertTrue(cp1 == cp2)
        cp1.ddc_init = ddc_init
        self.assertTrue(cp1 != cp2)
        cp2.ddc_init = ddc_init
        self.assertTrue(cp1 == cp2)
        cp1.L_init = L_init
        self.assertTrue(cp1 != cp2)
        cp2.L_init = L_init
        self.assertTrue(cp1 == cp2)
        cp1.dL_init = dL_init
        self.assertTrue(cp1 != cp2)
        cp2.dL_init = dL_init
        self.assertTrue(cp1 == cp2)
        cp1.q_init = q_init
        self.assertTrue(cp1 != cp2)
        cp2.q_init = q_init
        self.assertTrue(cp1 == cp2)
        cp1.c_final = c_final
        self.assertTrue(cp1 != cp2)
        cp2.c_final = c_final.copy()
        self.assertTrue(cp1 == cp2)
        cp1.dc_final = dc_final
        self.assertTrue(cp1 != cp2)
        cp2.dc_final = dc_final.copy()
        self.assertTrue(cp1 == cp2)
        cp1.ddc_final = ddc_final
        self.assertTrue(cp1 != cp2)
        cp2.ddc_final = ddc_final.copy()
        self.assertTrue(cp1 == cp2)
        cp1.L_final = L_final
        self.assertTrue(cp1 != cp2)
        L_final2 = np.array(L_final)
        cp2.L_final = L_final2
        self.assertTrue(cp1 == cp2)
        cp1.dL_final = dL_final
        self.assertTrue(cp1 != cp2)
        dL_final2 = np.array(dL_final)
        cp2.dL_final = dL_final2
        self.assertTrue(cp1 == cp2)
        cp1.q_final = q_final
        self.assertTrue(cp1 != cp2)
        cp2.q_final = q_final
        self.assertTrue(cp1 == cp2)
        # curves :
        q = createRandomPiecewisePolynomial(31)
        dq = createRandomPiecewisePolynomial(30)
        ddq = createRandomPiecewisePolynomial(30)
        tau = createRandomPiecewisePolynomial(30)
        dc = createRandomPiecewisePolynomial(3)
        ddc = createRandomPiecewisePolynomial(3)
        L = createRandomPiecewisePolynomial(3)
        dL = createRandomPiecewisePolynomial(3)
        wrench = createRandomPiecewisePolynomial(6)
        zmp = createRandomPiecewisePolynomial(3)
        root = createRandomSE3Traj()
        coefs = np.random.rand(3, 7)  # degree 3
        c1 = polynomial(coefs, 0, 2)
        c2 = polynomial(coefs, 0, 2)
        # assign trajectories :
        cp1.q_t = q
        self.assertTrue(cp1 != cp2)
        cp2.q_t = q
        self.assertTrue(cp1 == cp2)
        cp1.dq_t = dq
        self.assertTrue(cp1 != cp2)
        cp2.dq_t = dq
        self.assertTrue(cp1 == cp2)
        cp1.ddq_t = ddq
        self.assertTrue(cp1 != cp2)
        cp2.ddq_t = ddq
        self.assertTrue(cp1 == cp2)
        cp1.tau_t = tau
        self.assertTrue(cp1 != cp2)
        cp2.tau_t = tau
        self.assertTrue(cp1 == cp2)
        cp1.c_t = c1
        self.assertTrue(cp1 != cp2)
        cp2.c_t = c2
        self.assertTrue(cp1 == cp2)
        cp1.dc_t = dc
        self.assertTrue(cp1 != cp2)
        dc2 = dc
        cp2.dc_t = dc2
        self.assertTrue(cp1 == cp2)
        cp1.ddc_t = ddc
        self.assertTrue(cp1 != cp2)
        cp2.ddc_t = ddc
        self.assertTrue(cp1 == cp2)
        cp1.L_t = L
        self.assertTrue(cp1 != cp2)
        cp2.L_t = L
        self.assertTrue(cp1 == cp2)
        cp1.dL_t = dL
        self.assertTrue(cp1 != cp2)
        cp2.dL_t = dL
        self.assertTrue(cp1 == cp2)
        cp1.wrench_t = wrench
        self.assertTrue(cp1 != cp2)
        cp2.wrench_t = wrench
        self.assertTrue(cp1 == cp2)
        cp1.zmp_t = zmp
        self.assertTrue(cp1 != cp2)
        cp2.zmp_t = zmp
        self.assertTrue(cp1 == cp2)
        cp1.root_t = root
        self.assertTrue(cp1 != cp2)
        cp2.root_t = root
        self.assertTrue(cp1 == cp2)

        # test contacts
        p = SE3()
        p.setRandom()
        patchRF = ContactPatch(p, 0.5)
        cp1.addContact("right-leg", patchRF)
        self.assertTrue(cp1 != cp2)
        cp2.addContact("right-leg2", patchRF)
        self.assertTrue(cp1 != cp2)
        cp2.addContact("right-leg", patchRF)
        self.assertTrue(cp1 != cp2)
        cp2.removeContact("right-leg2")
        self.assertTrue(cp1 == cp2)
        p = SE3()
        p.setRandom()
        patchLF = ContactPatch(p, 0.5)
        patchLF2 = ContactPatch(p)
        cp1.addContact("left-leg", patchLF)
        self.assertFalse(cp1 == cp2)
        cp2.addContact("left-leg", patchLF2)
        self.assertFalse(cp1 == cp2)
        cp2.removeContact("left-leg")
        cp2.addContact("left-leg", patchLF.copy())
        self.assertFalse(cp1 != cp2)

        # test force trajectories :
        fR = createRandomPiecewisePolynomial(12)
        fL = createRandomPiecewisePolynomial(12)
        fL2 = createRandomPiecewisePolynomial(12)
        cp1.addContactForceTrajectory("right-leg", fR)
        self.assertTrue(cp1 != cp2)
        cp2.addContactForceTrajectory("right-leg", fR)
        self.assertTrue(cp1 == cp2)
        cp1.addContactForceTrajectory("left-leg", fL)
        self.assertTrue(cp1 != cp2)
        cp2.addContactForceTrajectory("left-leg", fL2)
        self.assertTrue(cp1 != cp2)
        cp2.addContactForceTrajectory("left-leg", fL)
        self.assertTrue(cp1 == cp2)
        fR = createRandomPiecewisePolynomial(1)
        fL = createRandomPiecewisePolynomial(1)
        fL2 = createRandomPiecewisePolynomial(1)
        cp1.addContactNormalForceTrajectory("right-leg", fR)
        self.assertTrue(cp1 != cp2)
        cp2.addContactNormalForceTrajectory("right-leg", fR)
        self.assertTrue(cp1 == cp2)
        cp1.addContactNormalForceTrajectory("left-leg", fL)
        self.assertTrue(cp1 != cp2)
        cp2.addContactNormalForceTrajectory("left-leg", fL2)
        self.assertTrue(cp1 != cp2)
        cp2.addContactNormalForceTrajectory("left-leg", fL)
        self.assertTrue(cp1 == cp2)
        # test effector trajectories :
        fR = createRandomSE3Traj()
        fL = createRandomSE3Traj()
        fL2 = createRandomSE3Traj()
        cp1.addEffectorTrajectory("right-hand", fR)
        self.assertTrue(cp1 != cp2)
        cp2.addEffectorTrajectory("right-hand", fR)
        self.assertTrue(cp1 == cp2)
        cp1.addEffectorTrajectory("left-hand", fL)
        self.assertTrue(cp1 != cp2)
        cp2.addEffectorTrajectory("left-hand", fL2)
        self.assertTrue(cp1 != cp2)
        cp2.addEffectorTrajectory("left-hand", fL)
        self.assertTrue(cp1 == cp2)

    def test_copy_constructor(self):
        cp1 = buildRandomContactPhase(0., 2.)
        cp2 = ContactPhase(cp1)
        cp3 = cp1.copy()
        self.assertEqual(cp1, cp2)
        self.assertEqual(cp1, cp3)

    def test_contact_phase_serialization_no_timing(self):
        cp1 = ContactPhase()
        addRandomPointsValues(cp1)
        cp1.saveAsText("cp_test.txt")
        cp_txt = ContactPhase()
        cp_txt.loadFromText("cp_test.txt")
        self.assertEqual(cp1, cp_txt)
        cp1.saveAsBinary("cp_test")
        cp_bin = ContactPhase()
        cp_bin.loadFromBinary("cp_test")
        self.assertEqual(cp1, cp_bin)
        cp1.saveAsXML("cp_test.xml", 'ContactPhase')
        cp_xml = ContactPhase()
        cp_xml.loadFromXML("cp_test.xml", 'ContactPhase')
        self.assertEqual(cp1, cp_xml)
        cp_pickled = pickle.dumps(cp1)
        cp_from_pickle = pickle.loads(cp_pickled)
        self.assertEqual(cp1, cp_from_pickle)

    def test_contact_phase_serialization_full(self):
        cp1 = buildRandomContactPhase(0., 2.)
        cp1.saveAsText("cp_test_full.txt")
        cp_txt = ContactPhase()
        cp_txt.loadFromText("cp_test_full.txt")
        self.assertEqual(cp1, cp_txt)
        cp1.saveAsBinary("cp_test_full")
        cp_bin = ContactPhase()
        cp_bin.loadFromBinary("cp_test_full")
        self.assertEqual(cp1, cp_bin)
        cp1.saveAsXML("cp_test_full.xml", 'ContactPhase')
        cp_xml = ContactPhase()
        cp_xml.loadFromXML("cp_test_full.xml", 'ContactPhase')
        self.assertEqual(cp1, cp_xml)
        # TODO : check serialization from another file
        cp_pickled = pickle.dumps(cp1)
        cp_from_pickle = pickle.loads(cp_pickled)
        self.assertEqual(cp1, cp_from_pickle)

    def test_contact_phase_contacts_variation(self):
        # # contacts repositioned :
        cp1 = buildRandomContactPhase()
        cp2 = buildRandomContactPhase()
        repo = cp1.getContactsRepositioned(cp2)
        self.assertTrue(len(repo) == 2)
        self.assertTrue(repo[0] == "right-leg")
        self.assertTrue(repo[1] == "left-leg")
        repo1 = cp2.getContactsRepositioned(cp1)
        self.assertTrue(len(repo1) == 2)
        self.assertTrue(repo1[0] == "right-leg")
        self.assertTrue(repo1[1] == "left-leg")
        vars = cp1.getContactsVariations(cp2)
        self.assertTrue(len(vars) == 2)

        # # contacts broken :

        RH_placement = SE3.Identity()
        RH_placement.setRandom()
        RH_patch = ContactPatch(RH_placement)
        cp3 = ContactPhase()
        cp3.addContact("right-leg", RH_patch)
        p = SE3.Identity()
        p.setRandom()
        cp3.addContact("left-leg", ContactPatch(p))
        cp4 = ContactPhase()
        cp4.addContact("right-leg", RH_patch)
        broken = cp3.getContactsBroken(cp4)
        self.assertTrue(len(broken) == 1)
        self.assertTrue(broken[0] == "left-leg")
        broken1 = cp4.getContactsBroken(cp3)
        self.assertTrue(len(broken1) == 0)

        created = cp4.getContactsCreated(cp3)
        self.assertTrue(len(created) == 1)
        self.assertTrue(created[0] == "left-leg")
        created1 = cp3.getContactsCreated(cp4)
        self.assertTrue(len(created1) == 0)

        vars = cp3.getContactsVariations(cp4)
        self.assertTrue(len(vars) == 1)
        self.assertTrue(vars[0] == "left-leg")
        vars = cp4.getContactsVariations(cp3)
        self.assertTrue(len(vars) == 1)
        self.assertTrue(vars[0] == "left-leg")

    def test_com_trajectory_helper(self):
        N = 7
        points = array(random.rand(3, N))
        points_derivative = array(random.rand(3, N))
        points_second_derivative = array(random.rand(3, N))
        time_points = array(random.rand(1, N)).T
        time_points.sort(0)
        cp = ContactPhase()
        cp.setCOMtrajectoryFromPoints(points, points_derivative, points_second_derivative, time_points)
        self.assertEqual(cp.c_t.min(), time_points[0])
        self.assertEqual(cp.c_t.max(), time_points[-1])
        self.assertEqual(cp.dc_t.dim(), 3)
        for i in range(N):
            self.assertTrue(isclose(cp.c_t(time_points[i, 0]), points[:, i]).all())
            self.assertTrue(isclose(cp.dc_t(time_points[i, 0]), points_derivative[:, i]).all())
            self.assertTrue(isclose(cp.ddc_t(time_points[i, 0]), points_second_derivative[:, i]).all())

        cp.setAMtrajectoryFromPoints(points, points_derivative, time_points)
        for i in range(N):
            self.assertTrue(isclose(cp.L_t(time_points[i, 0]), points[:, i]).all())
            self.assertTrue(isclose(cp.dL_t(time_points[i, 0]), points_derivative[:, i]).all())

        cp.setJointsTrajectoryFromPoints(points, points_derivative, points_second_derivative, time_points)
        for i in range(N):
            self.assertTrue(isclose(cp.q_t(time_points[i, 0]), points[:, i]).all())
            self.assertTrue(isclose(cp.dq_t(time_points[i, 0]), points_derivative[:, i]).all())
            self.assertTrue(isclose(cp.ddq_t(time_points[i, 0]), points_second_derivative[:, i]).all())


class ContactSequenceTest(unittest.TestCase):
    def test_append(self):
        cs = ContactSequence(0)
        self.assertTrue(cs.size() == 0)
        cp0 = buildRandomContactPhase(0, 2)
        cp1 = buildRandomContactPhase(2, 4.)
        id = cs.append(cp0)
        self.assertTrue(cs.size() == 1)
        self.assertTrue(id == 0)
        self.assertTrue(cs.contactPhases[0] == cp0)
        id = cs.append(cp1)
        self.assertTrue(cs.size() == 2)
        self.assertTrue(id == 1)
        self.assertTrue(cs.contactPhases[0] == cp0)
        self.assertTrue(cs.contactPhases[1] == cp1)

    """ # test copied from c++, but the same behaviour cannot be obtained in python
  def test_accessor_phase_vector(self):
    cs = ContactSequence(0)
    cp0 = buildRandomContactPhase(0,2)
    cp1 = buildRandomContactPhase(2,4.)
    cs.append(cp0)
    cs.append(cp1)
    phases = cs.contactPhases()
    self.assertTrue(type(phases) is list)
    self.assertTrue(len(phases) == 2)
    self.assertTrue(phases[0] == cp0)
    self.assertTrue(phases[1] == cp1)

    # check that the accessor to contactPhases() create a copy :
    cp2 = buildRandomContactPhase(0,2)
    phases += [cp2]
    self.assertTrue(len(phases) == 3)
    self.assertTrue(cs.size() == 2 ) # original contact sequence should not be modified
    phases[1].duration = 3.
    self.assertTrue(cs.contactPhases[1) == cp1) # original contact sequence should not be modified
  """

    def test_accessor_phase_reference(self):
        cs = ContactSequence(0)
        cp0 = buildRandomContactPhase(0, 2)
        cp1 = buildRandomContactPhase(2, 4.)
        cp2 = buildRandomContactPhase(2, 4.)
        cs.append(cp0)
        cs.append(cp1)
        cs.contactPhases[1].timeFinal = 10.
        self.assertTrue(cs.contactPhases[1] != cp1)
        self.assertTrue(cs.contactPhases[1].timeFinal == 10.)

        cs.contactPhases[0] = cp2
        self.assertTrue(cs.contactPhases[0] == cp2)

        # try with a variable :
        cp_ref = cs.contactPhases[0]
        c_init = np.random.rand(3)
        cp_ref.c_init = c_init
        cp_ref.duration = 10
        self.assertTrue(cs.contactPhases[0].duration == 10)
        self.assertTrue(array_equal(cs.contactPhases[0].c_init, c_init))

    def test_constructor_with_size(self):
        cp_default = ContactPhase()
        cs = ContactSequence(3)
        self.assertTrue(cs.size() == 3)
        for i in range(3):
            self.assertTrue(cs.contactPhases[i] == cp_default)

        # try to modify the uninitialized contact phase inside the sequence from the reference
        cp_0 = cs.contactPhases[0]
        c_init = np.random.rand(3)
        cp_0.c_init = c_init
        cp_0.duration = 10
        self.assertTrue(cs.contactPhases[0] != cp_default)
        self.assertTrue(cs.contactPhases[0].duration == 10)
        self.assertTrue(array_equal(cs.contactPhases[0].c_init, c_init))

        cp1 = buildRandomContactPhase(2, 4.)
        cs.contactPhases[1] = cp1
        self.assertTrue(cs.contactPhases[1] == cp1)

    def test_resize(self):
        cp_default = ContactPhase()
        cs = ContactSequence(3)
        cp_0 = cs.contactPhases[0]
        c_init = np.random.rand(3)
        cp_0.c_init = c_init
        cp_0.duration = 10
        cp1 = buildRandomContactPhase(2, 4.)
        cs.contactPhases[1] = cp1
        # with smaller value than current :
        cs.resize(1)
        self.assertTrue(cs.size() == 1)
        self.assertTrue(cs.contactPhases[0].duration == 10)
        self.assertTrue(array_equal(cs.contactPhases[0].c_init, c_init))

        # check with greater size than current :
        cs.resize(4)
        self.assertTrue(cs.size() == 4)
        self.assertTrue(cs.contactPhases[0].duration == 10)
        self.assertTrue(array_equal(cs.contactPhases[0].c_init, c_init))
        for i in range(1, 4):
            self.assertTrue(cs.contactPhases[i] == cp_default)

    def test_operator_equal(self):
        cs3 = ContactSequence()
        cs4 = ContactSequence()

        self.assertTrue(cs3 == cs4)
        cp3_0 = buildRandomContactPhase(0., 2.)
        cs3.append(cp3_0)
        self.assertTrue(cs3 != cs4)
        self.assertFalse(cs3 == cs4)
        cs4.append(cp3_0)
        self.assertTrue(cs3 == cs4)
        cp3_1 = buildRandomContactPhase(0., 2.)
        cs3.append(cp3_1)
        self.assertTrue(cs3 != cs4)
        cs4.append(cp3_1)
        self.assertTrue(cs3 == cs4)
        cs4.contactPhases[1].duration = 10
        self.assertTrue(cs4.contactPhases[1] != cp3_1)
        self.assertTrue(cs3 != cs4)
        cs5 = ContactSequence(2)
        cs5.contactPhases[0] = cp3_0
        self.assertTrue(cs3 != cs5)
        cs5.contactPhases[1] = cp3_1
        self.assertTrue(cs3 == cs5)

    def test_copy_constructor(self):
        cs = ContactSequence()
        for i in range(10):
            cp = buildRandomContactPhase(0., 2.)
            cs.append(cp)
        self.assertTrue(cs.size() == 10)

        cs1 = ContactSequence(cs)
        self.assertTrue(cs == cs1)
        for i in range(10):
            self.assertTrue(cs.contactPhases[i] == cs1.contactPhases[i])

        # check that it's a copy and not the same object :
        cs.contactPhases[0].duration = 15.
        self.assertFalse(cs == cs1)
        self.assertFalse(cs.contactPhases[0] == cs1.contactPhases[0])

    def test_serialization(self):
        cs = ContactSequence()
        for i in range(10):
            cp = buildRandomContactPhase(0., 2.)
            cs.append(cp)

        cs.saveAsText("cs_test_full.txt")
        cs_txt = ContactSequence()
        cs_txt.loadFromText("cs_test_full.txt")
        self.assertEqual(cs, cs_txt)
        cs.saveAsBinary("cs_test_full")
        cs_bin = ContactSequence()
        cs_bin.loadFromBinary("cs_test_full")
        self.assertEqual(cs, cs_bin)
        cs.saveAsXML("cs_test_full.xml", 'ContactSequence')
        cs_xml = ContactSequence()
        cs_xml.loadFromXML("cs_test_full.xml", 'ContactPatch')
        self.assertEqual(cs, cs_xml)

    def test_contact_sequence_helpers(self):
        cs1 = ContactSequence()
        self.assertTrue(cs1.size() == 0)
        cp0 = buildRandomContactPhase(0, 2)
        cp1 = buildRandomContactPhase(2, 4.)
        cs1.append(cp0)
        cs1.append(cp1)
        # # test break contact :
        self.assertTrue(cs1.size() == 2)
        cs1.breakContact("left-leg")
        self.assertTrue(cs1.size() == 3)
        self.assertFalse(cs1.contactPhases[2].isEffectorInContact("left-leg"))
        self.assertTrue(
            cs1.contactPhases[1].timeFinal == 4.)  # time final of previous phase should not have been modified
        # check that the final value of the previous phase have been copied in the initial value of the new one
        self.assertTrue(array_equal(cs1.contactPhases[1].c_final, cs1.contactPhases[2].c_init))
        self.assertTrue(array_equal(cs1.contactPhases[1].dc_final, cs1.contactPhases[2].dc_init))
        self.assertTrue(array_equal(cs1.contactPhases[1].ddc_final, cs1.contactPhases[2].ddc_init))
        self.assertTrue(array_equal(cs1.contactPhases[1].L_final, cs1.contactPhases[2].L_init))
        self.assertTrue(array_equal(cs1.contactPhases[1].dL_final, cs1.contactPhases[2].dL_init))
        self.assertTrue(array_equal(cs1.contactPhases[1].q_final, cs1.contactPhases[2].q_init))
        self.assertTrue(cs1.contactPhases[1].timeFinal == cs1.contactPhases[2].timeInitial)
        # check that the other contactPatch have been copied :
        self.assertTrue(
            cs1.contactPhases[1].contactPatch("right-leg") == cs1.contactPhases[2].contactPatch("right-leg"))

        # # test create contact :
        placement_random = SE3.Identity()
        placement_random.setRandom()
        target = ContactPatch(placement_random)
        cs1.createContact("left-leg", target, 2.5)
        self.assertTrue(cs1.size() == 4)
        self.assertTrue(
            cs1.contactPhases[2].timeFinal == 6.5)  # time final of previous phase should have been modified
        self.assertTrue(cs1.contactPhases[3].contactPatch("left-leg") == target)
        # check that the final value of the previous phase have been copied in the initial value of the new one
        self.assertTrue(array_equal(cs1.contactPhases[2].c_final, cs1.contactPhases[3].c_init))
        self.assertTrue(array_equal(cs1.contactPhases[2].dc_final, cs1.contactPhases[3].dc_init))
        self.assertTrue(array_equal(cs1.contactPhases[2].ddc_final, cs1.contactPhases[3].ddc_init))
        self.assertTrue(array_equal(cs1.contactPhases[2].L_final, cs1.contactPhases[3].L_init))
        self.assertTrue(array_equal(cs1.contactPhases[2].dL_final, cs1.contactPhases[3].dL_init))
        self.assertTrue(array_equal(cs1.contactPhases[2].q_final, cs1.contactPhases[3].q_init))
        self.assertTrue(cs1.contactPhases[2].timeFinal == cs1.contactPhases[3].timeInitial)
        # check that the other contactPatch have been copied :
        self.assertTrue(
            cs1.contactPhases[2].contactPatch("right-leg") == cs1.contactPhases[3].contactPatch("right-leg"))

        # # test break with duration :
        cs1.breakContact("left-leg", 1.)
        self.assertTrue(cs1.size() == 5)
        self.assertFalse(cs1.contactPhases[4].isEffectorInContact("left-leg"))
        self.assertTrue(
            cs1.contactPhases[3].timeFinal == 7.5)  # time final of previous phase should have been modified

        # # test  create contact with no duration:
        cs1.contactPhases[4].duration = 1.
        self.assertTrue(
            cs1.contactPhases[4].timeFinal == 8.5)  # time final of previous phase should have been modified
        placement_random.setRandom()
        target = ContactPatch(placement_random)
        cs1.createContact("left-leg", target)
        self.assertTrue(cs1.size() == 6)
        self.assertTrue(
            cs1.contactPhases[4].timeFinal == 8.5)  # time final of previous phase should have been modified
        self.assertTrue(
            cs1.contactPhases[5].timeInitial == 8.5)  # time final of previous phase should have been modified

        # # test move effector to placement :
        target_placement = SE3.Identity()
        target_placement.setRandom()
        addRandomPointsValues(cs1.contactPhases[5])
        cs1.contactPhases[5].contactPatch("right-leg").friction = 2.
        cs1.moveEffectorToPlacement("right-leg", target_placement, 1., 1.5)
        self.assertTrue(cs1.size() == 8)
        self.assertFalse(cs1.contactPhases[6].isEffectorInContact("right-leg"))
        self.assertTrue(cs1.contactPhases[7].isEffectorInContact("right-leg"))
        self.assertTrue(cs1.contactPhases[7].contactPatch("right-leg").placement == target_placement)
        # check that previous patch have not been modified :
        self.assertTrue(cs1.contactPhases[5].contactPatch("right-leg").placement != target_placement)
        self.assertTrue(cs1.contactPhases[7].contactPatch("right-leg").friction == 2.)
        self.assertTrue(cs1.contactPhases[5].timeFinal == 9.5)
        self.assertTrue(cs1.contactPhases[6].timeInitial == 9.5)
        self.assertTrue(cs1.contactPhases[6].timeFinal == 11.)
        self.assertTrue(cs1.contactPhases[7].timeInitial == 11.)
        # check that the final value of the previous phase have been copied in the initial value of the new one
        self.assertTrue(array_equal(cs1.contactPhases[5].c_final, cs1.contactPhases[6].c_init))
        self.assertTrue(array_equal(cs1.contactPhases[5].dc_final, cs1.contactPhases[6].dc_init))
        self.assertTrue(array_equal(cs1.contactPhases[5].ddc_final, cs1.contactPhases[6].ddc_init))
        self.assertTrue(array_equal(cs1.contactPhases[5].L_final, cs1.contactPhases[6].L_init))
        self.assertTrue(array_equal(cs1.contactPhases[5].dL_final, cs1.contactPhases[6].dL_init))
        self.assertTrue(array_equal(cs1.contactPhases[5].q_final, cs1.contactPhases[6].q_init))
        # with MoveEffector, the middle phase should have the same initial and final point :
        self.assertTrue(array_equal(cs1.contactPhases[6].c_final, cs1.contactPhases[6].c_init))
        self.assertTrue(array_equal(cs1.contactPhases[6].dc_final, cs1.contactPhases[6].dc_init))
        self.assertTrue(array_equal(cs1.contactPhases[6].ddc_final, cs1.contactPhases[6].ddc_init))
        self.assertTrue(array_equal(cs1.contactPhases[6].L_final, cs1.contactPhases[6].L_init))
        self.assertTrue(array_equal(cs1.contactPhases[6].dL_final, cs1.contactPhases[6].dL_init))
        self.assertTrue(array_equal(cs1.contactPhases[6].q_final, cs1.contactPhases[6].q_init))
        # check that the final value of the previous phase have been copied in the initial value of the new one
        self.assertTrue(array_equal(cs1.contactPhases[6].c_final, cs1.contactPhases[7].c_init))
        self.assertTrue(array_equal(cs1.contactPhases[6].dc_final, cs1.contactPhases[7].dc_init))
        self.assertTrue(array_equal(cs1.contactPhases[6].ddc_final, cs1.contactPhases[7].ddc_init))
        self.assertTrue(array_equal(cs1.contactPhases[6].L_final, cs1.contactPhases[7].L_init))
        self.assertTrue(array_equal(cs1.contactPhases[6].dL_final, cs1.contactPhases[7].dL_init))
        self.assertTrue(array_equal(cs1.contactPhases[6].q_final, cs1.contactPhases[7].q_init))
        # check that the other contactPatch have been copied :
        self.assertTrue(cs1.contactPhases[5].contactPatch("left-leg") == cs1.contactPhases[6].contactPatch("left-leg"))
        self.assertTrue(cs1.contactPhases[6].contactPatch("left-leg") == cs1.contactPhases[7].contactPatch("left-leg"))

        # # test move effector of:
        target_transform = SE3.Identity()
        target_transform.setRandom()
        cs1.contactPhases[7].contactPatch("left-leg").friction = 10.
        cs1.moveEffectorOf("left-leg", target_transform, 1., 1.5)
        self.assertTrue(cs1.size() == 10)
        self.assertFalse(cs1.contactPhases[8].isEffectorInContact("left-leg"))
        self.assertTrue(cs1.contactPhases[9].isEffectorInContact("left-leg"))
        target_placement = target_transform.act(cs1.contactPhases[7].contactPatch("left-leg").placement)
        self.assertTrue(cs1.contactPhases[9].contactPatch("left-leg").placement == target_placement)
        self.assertTrue(cs1.contactPhases[9].contactPatch("left-leg").friction == 10.)
        # check that the other contactPatch have been copied :
        self.assertTrue(
            cs1.contactPhases[7].contactPatch("right-leg") == cs1.contactPhases[8].contactPatch("right-leg"))
        self.assertTrue(
            cs1.contactPhases[8].contactPatch("right-leg") == cs1.contactPhases[9].contactPatch("right-leg"))

    def test_contact_sequence_helpers_errors(self):
        cs1 = ContactSequence()
        self.assertTrue(cs1.size() == 0)
        cp0 = buildRandomContactPhase(0, 2)
        cp1 = buildRandomContactPhase(2, 4.)
        cp1.removeContact("left-leg")
        cs1.append(cp0)
        cs1.append(cp1)
        self.assertTrue(cs1.size() == 2)
        with self.assertRaises(ValueError):
            cs1.breakContact("left-leg")  # contact do not exist
        self.assertTrue(cs1.size() == 2)
        cp2 = buildRandomContactPhase()
        cs1.append(cp2)
        self.assertTrue(cs1.size() == 3)
        with self.assertRaises(ValueError):
            cs1.breakContact("left-leg", 1.5)  # time interval not defined for last phase
        self.assertTrue(cs1.size() == 3)

        # # check that create contact correctly throw error when needed :
        placement = SE3.Identity()
        placement.setRandom()

        with self.assertRaises(ValueError):
            cs1.createContact("left-leg", ContactPatch(placement))  # contact already exist
        self.assertTrue(cs1.size() == 3)
        cs1.breakContact("left-leg")
        self.assertTrue(cs1.size() == 4)
        with self.assertRaises(ValueError):
            cs1.createContact("left-leg", ContactPatch(placement), 2.)  # time interval not defined
        self.assertTrue(cs1.size() == 4)

    def test_is_consistent(self):
        cs1 = ContactSequence(0)
        cp0 = buildRandomContactPhase(0, 2)
        cp1 = buildRandomContactPhase(2, 4.)
        cs1.append(cp0)
        cs1.append(cp1)
        consistent = cs1.haveTimings()
        self.assertTrue(consistent)

        cs2 = ContactSequence(0)
        cp2 = buildRandomContactPhase(0, 2)
        cp3 = buildRandomContactPhase(1.5, 4.)
        cs2.append(cp2)
        cs2.append(cp3)
        consistent = cs2.haveTimings()
        self.assertFalse(consistent)

        cs3 = ContactSequence(0)
        cp4 = buildRandomContactPhase(0, 2)
        cp5 = buildRandomContactPhase()
        cs3.append(cp4)
        cs3.append(cp5)
        consistent = cs3.haveTimings()
        self.assertFalse(consistent)

        cs4 = ContactSequence(0)
        cp6 = buildRandomContactPhase()
        cp7 = buildRandomContactPhase(1, 3)
        cs4.append(cp6)
        cs4.append(cp7)
        consistent = cs4.haveTimings()
        self.assertFalse(consistent)

    def test_contact_sequence_have_contact_model(self):
        cs1 = ContactSequence(0)
        cp0 = buildRandomContactPhase(0, 2)
        cp1 = buildRandomContactPhase(2, 4.)
        cs1.append(cp0)
        cs1.append(cp1)
        self.assertFalse(cs1.haveContactModelDefined())

        mp1 = ContactModel(0.5, ContactType.CONTACT_PLANAR)
        pos = np.random.rand(3, 5)
        mp1.contact_points_positions = pos
        mp2 = ContactModel(1., ContactType.CONTACT_POINT)
        pos = np.random.rand(3, 5)
        mp1.contact_points_positions = pos

        cs1.contactPhases[0].contactPatch("right-leg").contact_model = mp1
        cs1.contactPhases[0].contactPatch("left-leg").contact_model = mp2
        cs1.contactPhases[1].contactPatch("right-leg").contact_model = mp1
        cs1.contactPhases[1].contactPatch("left-leg").contact_model = mp2
        self.assertTrue(cs1.haveContactModelDefined())

        cp2 = buildRandomContactPhase(6., 8.)
        cs1.append(cp2)
        self.assertFalse(cs1.haveContactModelDefined())
        mp3 = ContactModel(0.2)
        cs1.contactPhases[2].contactPatch("right-leg").contact_model = mp3
        cs1.contactPhases[2].contactPatch("left-leg").contact_model = mp2
        self.assertFalse(cs1.haveContactModelDefined())

        mp3.contact_type = ContactType.CONTACT_PLANAR  # do not change the contact model already in the seqence
        self.assertFalse(cs1.haveContactModelDefined())

        cs1.contactPhases[2].contactPatch("right-leg").contact_model.contact_type = ContactType.CONTACT_PLANAR
        self.assertTrue(cs1.haveContactModelDefined())

    def test_contact_sequence_concatenate_config_traj(self):
        cs1 = ContactSequence(0)
        cp0 = buildRandomContactPhase(0, 2)
        cp1 = buildRandomContactPhase(2, 4.)
        p0 = np.random.rand(35)
        p1 = np.random.rand(35)
        p2 = np.random.rand(35)
        t0 = 0.
        t1 = 2.
        t2 = 4.
        c1 = polynomial(p0, p1, t0, t1)
        c2 = polynomial(p1, p2, t1, t2)
        cp0.q_t = c1
        cp1.q_t = c2
        self.assertTrue(cp0.q_t.min() == 0.)
        self.assertTrue(cp0.q_t.max() == 2.)
        self.assertTrue(cp1.q_t.min() == 2.)
        self.assertTrue(cp1.q_t.max() == 4.)
        cs1.append(cp0)
        cs1.append(cp1)
        q_t = cs1.concatenateQtrajectories()
        self.assertTrue(q_t.min() == 0.)
        self.assertTrue(q_t.max() == 4.)
        self.assertTrue(array_equal(q_t(0), cp0.q_t(0)))
        self.assertTrue(array_equal(q_t(0.5), cp0.q_t(0.5)))
        self.assertTrue(array_equal(q_t(2.), cp0.q_t(2.)))
        self.assertTrue(array_equal(q_t(3), cp1.q_t(3)))
        self.assertTrue(array_equal(q_t(4.), cp1.q_t(4.)))

    def test_contact_sequence_concatenate_effector_traj(self):
        cs1 = ContactSequence(0)
        cp0 = ContactPhase(0, 2)
        cp1 = ContactPhase(2, 4.)
        cp2 = ContactPhase(4, 8.)
        p0 = SE3()
        p0.setRandom()
        p1 = SE3()
        p1.setRandom()
        p2 = SE3()
        p2.setRandom()

        traj_0 = SE3Curve(p0, p1, 0., 2.)
        traj_2 = SE3Curve(p1, p2, 4., 8.)
        cp0.addEffectorTrajectory("right_leg", traj_0)
        cp2.addEffectorTrajectory("right_leg", traj_2)
        cs1.append(cp0)
        cs1.append(cp1)
        cs1.append(cp2)

        traj = cs1.concatenateEffectorTrajectories("right_leg")
        self.assertTrue(traj.min() == 0.)
        self.assertTrue(traj.max() == 8.)
        self.assertTrue(np.isclose(traj(0.), traj_0(0.)).all())
        self.assertTrue(np.isclose(traj(1.5), traj_0(1.5)).all())
        self.assertTrue(np.isclose(traj(2.), traj_0(2.)).all())
        self.assertTrue(np.isclose(traj(4.), traj_2(4.)).all())
        self.assertTrue(np.isclose(traj(6.), traj_2(6.)).all())
        self.assertTrue(np.isclose(traj(8.), traj_2(8.)).all())
        self.assertTrue(np.isclose(traj(2.5), traj_0(2.)).all())
        self.assertTrue(np.isclose(traj(3.8), traj_0(2.)).all())

    def test_contact_sequence_concatenate_force_traj(self):
        cs1 = ContactSequence(0)
        cp0 = ContactPhase(0, 2)
        cp1 = ContactPhase(2, 4.)
        cp2 = ContactPhase(4, 8.)

        cp0.addContact("right_leg", ContactPatch())
        cp2.addContact("right_leg", ContactPatch())
        f_0 = createRandomPiecewisePolynomial(12, 0, 2)
        f_2 = createRandomPiecewisePolynomial(12, 4, 8)
        cp0.addContactForceTrajectory("right_leg", f_0)
        cp2.addContactForceTrajectory("right_leg", f_2)

        cs1.append(cp0)
        cs1.append(cp1)
        cs1.append(cp2)

        forces = cs1.concatenateContactForceTrajectories("right_leg")
        self.assertTrue(forces.min() == 0.)
        self.assertTrue(forces.max() == 8.)
        self.assertTrue(array_equal(forces(0.), f_0(0.)))
        self.assertTrue(array_equal(forces(1.5), f_0(1.5)))
        self.assertTrue(array_equal(forces(1.999), f_0(1.999)))
        self.assertTrue(array_equal(forces(4.), f_2(4.)))
        self.assertTrue(array_equal(forces(6.), f_2(6.)))
        self.assertTrue(array_equal(forces(8.), f_2(8.)))
        self.assertTrue(array_equal(forces(2.), np.zeros(12)))
        self.assertTrue(array_equal(forces(2.5), np.zeros(12)))
        self.assertTrue(array_equal(forces(3.8), np.zeros(12)))

    def test_contact_sequence_concatenate_normal_force_traj(self):
        cs1 = ContactSequence(0)
        cp0 = ContactPhase(0, 2)
        cp1 = ContactPhase(2, 4.)
        cp2 = ContactPhase(4, 8.)

        cp1.addContact("right_leg", ContactPatch())
        f_1 = createRandomPiecewisePolynomial(1, 2., 4.)
        cp1.addContactNormalForceTrajectory("right_leg", f_1)

        cs1.append(cp0)
        cs1.append(cp1)
        cs1.append(cp2)

        forces = cs1.concatenateNormalForceTrajectories("right_leg")
        self.assertTrue(forces.min() == 0.)
        self.assertTrue(forces.max() == 8.)
        self.assertTrue(array_equal(forces(2.), f_1(2.)))
        self.assertTrue(array_equal(forces(2.5), f_1(2.5)))
        self.assertTrue(array_equal(forces(3.999), f_1(3.999)))
        self.assertTrue(array_equal(forces(0.), np.zeros(1)))
        self.assertTrue(array_equal(forces(1.5), np.zeros(1)))
        self.assertTrue(array_equal(forces(4.), np.zeros(1)))
        self.assertTrue(array_equal(forces(7.5), np.zeros(1)))

    def test_contact_sequence_phase_at_time(self):
        cs1 = ContactSequence(0)
        cp0 = ContactPhase(0, 2)
        cp1 = ContactPhase(2, 4.)
        cp2 = ContactPhase(4, 8.)

        cs1.append(cp0)
        cs1.append(cp1)
        cs1.append(cp2)

        self.assertEqual(cs1.phaseIdAtTime(0.), 0)
        self.assertEqual(cs1.phaseIdAtTime(1.), 0)
        self.assertEqual(cs1.phaseIdAtTime(1.9), 0)
        self.assertEqual(cs1.phaseIdAtTime(2.), 1)
        self.assertEqual(cs1.phaseIdAtTime(3.5), 1)
        self.assertEqual(cs1.phaseIdAtTime(4.), 2)
        self.assertEqual(cs1.phaseIdAtTime(5.), 2)
        self.assertEqual(cs1.phaseIdAtTime(8.), 2)
        self.assertEqual(cs1.phaseIdAtTime(-0.5), -1)
        self.assertEqual(cs1.phaseIdAtTime(10.), -1)

        self.assertEqual(cs1.phaseAtTime(0.), cp0)
        self.assertEqual(cs1.phaseAtTime(1.), cp0)
        self.assertEqual(cs1.phaseAtTime(1.9), cp0)
        self.assertEqual(cs1.phaseAtTime(2.), cp1)
        self.assertEqual(cs1.phaseAtTime(3.5), cp1)
        self.assertEqual(cs1.phaseAtTime(4.), cp2)
        self.assertEqual(cs1.phaseAtTime(5.), cp2)
        self.assertEqual(cs1.phaseAtTime(8.), cp2)
        with self.assertRaises(ValueError):
            cs1.phaseAtTime(-0.5)
        with self.assertRaises(ValueError):
            cs1.phaseAtTime(10.)

    def test_pickle_contact_sequence(self):
        cs = ContactSequence()
        for i in range(10):
            cp = buildRandomContactPhase(0., 2.)
            cs.append(cp)
        cs_pickled = pickle.dumps(cs)
        cs_from_pickle = pickle.loads(cs_pickled)
        self.assertEqual(cs_from_pickle, cs)


if __name__ == '__main__':
    unittest.main()
