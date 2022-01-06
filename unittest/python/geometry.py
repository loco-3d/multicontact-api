# Copyright (c) 2015-2019, CNRS
# Authors: Justin Carpentier <jcarpent@laas.fr>, Pierre Fernbach <pfernbac@laas.fr>
import unittest

import numpy as np

import multicontact_api

multicontact_api.switchToNumpyArray()


class GeometryTest(unittest.TestCase):
    def test_geom_soc6(self):
        A = np.identity(3)
        c = np.array([1., 1., 1.])

        e = multicontact_api.Ellipsoid3d(A, c)
        self.assertTrue((e.A == A).all())
        self.assertTrue((e.center == c).all())
        self.assertEqual(e.lhsValue(c), 0.0)

        Q = np.identity(6)
        direction = np.ones(6)

        C6D = multicontact_api.SOC6(Q, direction)
        self.assertTrue((C6D.Q == Q).all())
        self.assertTrue((C6D.direction == (direction / np.linalg.norm(direction))).all())


if __name__ == '__main__':
    unittest.main()
