import unittest

import multicontact_api


class TrivialTest(unittest.TestCase):
    """A test written by someone who has no idea what this software is about"""

    def test_trivial(self):
        comopla = multicontact_api.ContactModel()
        epsilon = 0.00001
        value_wanted = -1.0
        self.assertTrue((comopla.mu - value_wanted) < epsilon)


if __name__ == "__main__":
    unittest.main()
