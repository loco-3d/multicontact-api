# Copyright (c) 2019, CNRS
# Authors: Pierre Fernbach <pfernbac@laas.fr>
import unittest

import numpy as np
from random import uniform
from math import sqrt,sin,cos
import multicontact_api
multicontact_api.switchToNumpyArray()

from multicontact_api import ContactModelPlanar,ContactPatch,ContactPhase
from pinocchio import SE3,Quaternion
import curves
from curves import SE3Curve,polynomial,bezier,piecewise,piecewise_SE3

def randomQuaternion():
  u1 = uniform(0.,1.)
  u2 = uniform(0.,2.*np.pi)
  u3 = uniform(0.,2.*np.pi)
  a = sqrt(1-u1)
  b = sqrt(u1)
  q = Quaternion(a * sin(u2), a * cos(u2), b * sin(u3), b * cos(u3))
  q.normalize()
  return q


class ContactModelTest(unittest.TestCase):

  def test_contact_model_planar(self):
    mu = 0.3
    zmp_radius = 0.01
    # constructor with both values
    mp1 = ContactModelPlanar(mu,zmp_radius)
    # test getter bindings
    self.assertEqual(mp1.mu,mu)
    self.assertEqual(mp1.ZMP_radius,zmp_radius)

    # copy constructor :
    mp2 = ContactModelPlanar(mp1)
    self.assertEqual(mp2.mu,mu)
    self.assertEqual(mp2.ZMP_radius,zmp_radius)

    # test operator ==
    self.assertTrue(mp1 == mp2)
    mp1.mu = 0.5
    self.assertTrue(mp1 != mp2)


class ContactPatchTest(unittest.TestCase):

  def test_default_constructor(self):
    cp = ContactPatch()
    self.assertEqual(cp.friction,-1.0)
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
    cp = ContactPatch(p,0.9)
    self.assertTrue(cp.friction == 0.9)
    self.assertTrue(cp.placement == p)


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
    cp1 = ContactPatch(p,0.9)
    cp2 = ContactPatch(p,0.9)
    self.assertTrue(cp1 == cp2)
    cp1.placement.setRandom()
    self.assertTrue(cp1 != cp2)


  def test_copy_constructor(self):
    p = SE3()
    p.setRandom()
    cp1 = ContactPatch(p,0.9)
    cp2 = ContactPatch(cp1)
    self.assertTrue(cp1 == cp2)
    cp1.placement.setRandom()
    self.assertTrue(cp1 != cp2)

  def test_serialization(self):
    #TODO
    pass

class ContactPhaseTest(unittest.TestCase):

  def test_default_constructor(self):
    cp = ContactPhase()
    self.assertEqual(cp.timeInitial,-1)
    self.assertEqual(cp.timeFinal,-1)
    self.assertEqual(cp.duration,0)
    self.assertEqual(cp.numContacts(),0)
    self.assertEqual(len(cp.effectorsInContact()),0)

  def test_constructor_with_arguments(self):
    cp = ContactPhase(1,5)
    self.assertEqual(cp.timeInitial,1)
    self.assertEqual(cp.timeFinal,5)
    self.assertEqual(cp.duration,4)
    self.assertEqual(cp.numContacts(),0)
    self.assertEqual(len(cp.effectorsInContact()),0)
    with self.assertRaises(ValueError):
      cp = ContactPhase(1,0.5)


  def test_timings_setter(self):
    cp = ContactPhase()
    cp.timeInitial = 1.5
    cp.timeFinal = 3.
    self.assertEqual(cp.timeInitial,1.5)
    self.assertEqual(cp.timeFinal,3.)
    self.assertEqual(cp.duration,1.5)
    cp.duration = 2.
    self.assertEqual(cp.timeInitial,1.5)
    self.assertEqual(cp.timeFinal,3.5)
    self.assertEqual(cp.duration,2.)
    with self.assertRaises(ValueError):
      cp.timeFinal = 1.
    with self.assertRaises(ValueError):
      cp.duration = -0.5

  def test_contact_methods(self):
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    patchRF = ContactPatch(p,0.5)
    new = cp.addContact("right-leg",patchRF)
    self.assertTrue(new)
    self.assertTrue(cp.isEffectorInContact("right-leg"))
    self.assertTrue("right-leg" in cp.effectorsInContact())
    self.assertEqual(patchRF,cp.contactPatch("right-leg"))
    self.assertEqual(cp.numContacts(),1)

    # add another contact :
    p = SE3()
    p.setRandom()
    patchLF = ContactPatch(p,0.5)
    new = cp.addContact("left-leg",patchLF)
    self.assertTrue(new)
    self.assertTrue(cp.isEffectorInContact("right-leg"))
    self.assertTrue("right-leg" in cp.effectorsInContact())
    self.assertEqual(patchRF,cp.contactPatch("right-leg"))
    self.assertTrue(cp.isEffectorInContact("left-leg"))
    self.assertTrue("left-leg" in cp.effectorsInContact())
    self.assertEqual(patchLF,cp.contactPatch("left-leg"))
    self.assertEqual(cp.numContacts(),2)
    # check that the patch can be overwritten:
    p = SE3()
    p.setRandom()
    patchRF2 = ContactPatch(p,0.5)
    new = cp.addContact("right-leg",patchRF2)
    self.assertFalse(new)

    # check deletion of contacts :
    exist = cp.removeContact("right-leg")
    self.assertTrue(exist)
    self.assertTrue(cp.isEffectorInContact("left-leg"))
    self.assertTrue("left-leg" in cp.effectorsInContact())
    self.assertEqual(patchLF,cp.contactPatch("left-leg"))
    self.assertEqual(cp.numContacts(),1)
    self.assertFalse(cp.isEffectorInContact("right-leg"))
    self.assertFalse("right-leg" in cp.effectorsInContact())
    with self.assertRaises(ValueError):
      cp.contactPatch("right-leg")
    exist = cp.removeContact("right-leg")
    self.assertFalse(exist)

    exist = cp.removeContact("left-leg")
    self.assertTrue(exist)
    self.assertFalse(cp.isEffectorInContact("left-leg"))
    self.assertFalse("left-leg" in cp.effectorsInContact())
    self.assertFalse(cp.isEffectorInContact("right-leg"))
    self.assertFalse("right-leg" in cp.effectorsInContact())
    self.assertEqual(cp.numContacts(),0)

  def test_contact_patch_access(self):
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    patchRF = ContactPatch(p,0.5)
    cp.addContact("right-leg",patchRF)
    # check that the contactPatch have been copied and it's not a pointer :
    patchRF.placement.setRandom()
    self.assertNotEqual(patchRF,cp.contactPatch("right-leg"))
    patchRF = ContactPatch(cp.contactPatch("right-leg"))
    self.assertEqual(patchRF,cp.contactPatch("right-leg"))
    patchRF.placement.translation += np.array([0,0.1,0])
    self.assertNotEqual(patchRF,cp.contactPatch("right-leg"))
    patchRF =  ContactPatch(cp.contactPatch("right-leg"))
    # check that the getter of contactPatch is a non const reference:
    cp.contactPatch('right-leg').placement.setRandom()
    self.assertNotEqual(patchRF,cp.contactPatch("right-leg"))
    patchRF =  ContactPatch(cp.contactPatch("right-leg"))
    cp.contactPatch('right-leg').friction = 0.7
    self.assertNotEqual(patchRF,cp.contactPatch("right-leg"))
    patchRF =  ContactPatch(cp.contactPatch("right-leg"))
    cp.contactPatch("right-leg").placement.translation += np.array([0,0.1,0])
    self.assertNotEqual(patchRF,cp.contactPatch("right-leg"))

    patchRF = cp.contactPatch("right-leg")
    self.assertEqual(patchRF,cp.contactPatch("right-leg"))
    patchRF.placement.translation += np.array([0,0.1,0])
    self.assertEqual(patchRF,cp.contactPatch("right-leg"))


  def test_effector_trajectory(self):
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    patchRF = ContactPatch(p,0.5)
    cp.addContact("right-leg",patchRF)



if __name__ == '__main__':
  unittest.main()

