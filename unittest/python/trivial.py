import unittest

import multicontact_api


class TrivialTest(unittest.TestCase):
    """ A test written by someone who has no idea what this software is about"""
    def test_trivial(self):
        comopla = multicontact_api.ContactModelPlanar()
        self.assertEqual(comopla.mu, -1.0)


if __name__ == '__main__':
    unittest.main()
