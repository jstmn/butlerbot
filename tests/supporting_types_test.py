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
        bottle_diameter = 0.1
        bottle_height = 0.1
        bottle = Bottle(
            Transform("bottle_test", bottle_position, Quaternion(0, 0, 0, 1)),
            diameter=bottle_diameter,
            height=bottle_height
            )
        self.assertTrue(bottle.in_area(cuboid)) # in_area is equal or greater

        bottle_position = vector3(0.5, 0.5, -0.0001)
        bottle_diameter = 0.1
        bottle_height = 0.1
        bottle = Bottle(
            Transform("bottle_test", bottle_position, Quaternion(0, 0, 0, 1)),
            diameter=bottle_diameter,
            height=bottle_height
            )
        self.assertFalse(bottle.in_area(cuboid))

        bottle_position = vector3(0.5, 0.5, 0.0)
        bottle_diameter = 0.1
        bottle_height = 1.00001
        bottle = Bottle(
            Transform("bottle_test", bottle_position, Quaternion(0, 0, 0, 1)),
            diameter=bottle_diameter,
            height=bottle_height
            )
        self.assertFalse(bottle.in_area(cuboid))


    def test_distance_to_point(self):

        bottle_position = vector3(0.5, 0.0, 0.0)
        o_position = vector3(0.0, 0.0, 0.0)
        bottle = Bottle(
            Transform("bottle_test", bottle_position, Quaternion(0, 0, 0, 1)),
            diameter=0.1,
            height=0.1
        )
        distance_expected = 0.5
        distance_actual = bottle.distance_to_point(o_position)
        self.assertAlmostEqual(distance_expected, distance_actual)

        # 
        bottle_position = vector3(0.5, 0.0, 0.0)
        o_position = vector3(0.5, 0.0, 0.0)
        bottle = Bottle(
            Transform("bottle_test", bottle_position, Quaternion(0, 0, 0, 1)),
            diameter=0.1,
            height=0.1
        )
        distance_expected = 0.0
        distance_actual = bottle.distance_to_point(o_position)
        self.assertAlmostEqual(distance_expected, distance_actual)

        # 
        bottle_position = vector3(0.5, 0.0, 0.0)
        o_position      = vector3(0.5, 1.0, 1.0)
        bottle = Bottle(
            Transform("bottle_test", bottle_position, Quaternion(0, 0, 0, 1)),
            diameter=0.1,
            height=0.1
        )
        distance_expected = 1.41421356237 # sqrt(2)
        distance_actual = bottle.distance_to_point(o_position)
        self.assertAlmostEqual(distance_expected, distance_actual)



if __name__ == "__main__":
    unittest.main()