from typing import Tuple
import unittest
import sys
import os
sys.path.append(os.getcwd())

from hsrb_interface.geometry import Vector3, Quaternion
from hsrb_interface.geometry import vector3
from src.supporting_types import Cuboid, Bottle, Transform
from src.primitives import get_bottle_to_grasp


class MockedHsrbRobot:

    def __init__(self, xy: Tuple[float,float] ) -> None:
        self.xy = xy

    def base_xy_pose(self) -> Tuple[float, float]:
        return self.xy


class TestBottle(unittest.TestCase):

    def test_get_bottle_to_grasp(self):

        robot_xy = (0.5, 0.0, 0.0)
        robot = MockedHsrbRobot(robot_xy)

        # 
        closest_bottle_position = vector3(1.0, 0.0, 0.0)
        closest_bottle = Bottle(
            Transform("closest_bottle", closest_bottle_position, Quaternion(0, 0, 0, 1)))
        bottles = [
            closest_bottle,
            Bottle(Transform("closest_bottle", vector3(1.1, 0.0, 0.0), Quaternion(0, 0, 0, 1))),
            Bottle(Transform("closest_bottle", vector3(1.2, 0.0, 0.0), Quaternion(0, 0, 0, 1))),
        ]

        retuned_closest_bottle = get_bottle_to_grasp(robot, bottles)
        self.assertEqual(retuned_closest_bottle, closest_bottle)


class TestTransform(unittest.TestCase):

    def test_smoke_test(self):
        position = Vector3(1, 2, 3)
        quat = Quaternion(0, 0, 0, 1)
        pose = Transform("test_tf", position, quat)



if __name__ == "__main__":
    unittest.main()