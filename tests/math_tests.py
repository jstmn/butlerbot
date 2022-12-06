import unittest
import sys
import os
sys.path.append(os.getcwd())

from hsrb_interface.geometry import Vector3, Quaternion
from supporting_types import Transform


class TestMath(unittest.TestCase):

    def test_Transform(self):
        position = Vector3(1, 2, 3)
        quat = Quaternion(0, 0, 0, 1)
        pose = Transform("test_tf", position, quat)


if __name__ == "__main__":
    unittest.main()