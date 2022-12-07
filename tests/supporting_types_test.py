import unittest
import sys
import os
sys.path.append(os.getcwd())

from hsrb_interface.geometry import Vector3, Quaternion
from hsrb_interface.geometry import vector3
from src.supporting_types import Cuboid, Bottle, Transform


class TestBottle(unittest.TestCase):

    def test_in_area(self):
        
        # First cuboid: unit cube at the origin
        cuboid = Cuboid(
            vector3(0.0, 0, 0),
            vector3(1.0, 1.0, 1.0),
        )

        bottle_position = vector3(0.5, 0.5, 0.0001)
        bottle = Bottle(
            Transform("bottle_test", bottle_position, Quaternion(0, 0, 0, 1)),
            diameter=0.1,
            height=0.1
            )
        self.assertTrue(bottle.in_area(cuboid))

        bottle_position = vector3(0.5, 0.5, 0.0)  
        bottle = Bottle(
            Transform("bottle_test", bottle_position, Quaternion(0, 0, 0, 1)),
            diameter=0.1,
            height=0.1
            )
        self.assertTrue(bottle.in_area(cuboid)) # in_area is equal or greater



if __name__ == "__main__":
    unittest.main()