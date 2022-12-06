import unittest
import sys
import os
sys.path.append(os.getcwd())

from hsrb_interface.geometry import Vector3, Quaternion
from hsrb_interface.geometry import vector3
from src.supporting_types import Cuboid, Bottle, Transform


class TestBottle(unittest.TestCase):

    def test_in_area(self):
        
        cuboid = Cuboid(
            vector3(0.0, 0, 0),
            vector3(1.0, 1.0, 1.0),
        )
        bottle = Bottle(
            Transform("bottle_test", vector3(0.5, 0.5, 0.5), Quaternion(0, 0, 0, 1)),
            width=0.1,
            height=0.1
            )
        self.assertTrue(bottle.in_area(cuboid))


if __name__ == "__main__":
    unittest.main()