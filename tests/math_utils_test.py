from typing import Tuple
import unittest
import sys
import os
sys.path.append(os.getcwd())

from hsrb_interface.geometry import Vector3, Quaternion
from hsrb_interface.geometry import quaternion, vector3
from src.math_utils import *

class TestBottle(unittest.TestCase):

    def assert_vector3s_equal(self, vec1: Vector3, vec2: Vector3):
        self.assertAlmostEqual(vec1.x, vec2.x, places=6)
        self.assertAlmostEqual(vec1.y, vec2.y, places=6)
        self.assertAlmostEqual(vec1.z, vec2.z, places=6)

    def test_quaternion_from_matrix(self):
        r = np.array([
            [0.0,  -1.0,  0.0],
            [0.0,   0.0, -1.0],
            [-1.0,  0.0,  0.0],
        ])
        q_returned = quaternion_from_matrix(r)
        print(r)
        print(q_returned)


    def test_rotate_vector(self):

        # Smoke test: Check that unit quaternion rotates vector to itself
        vec = vector3(1, 0, 0)
        rot = quaternion(x=0, y=0, z=0, w=1)
        vec_rotated = rotate_vector(vec, rot)
        vec_rotated = vector3(vec_rotated[0], vec_rotated[1], vec_rotated[2])
        self.assert_vector3s_equal(vec_rotated, vec)

        # rotation about z axis by 90 degrees
        vec = vector3(1, 0, 0)
        rot = quaternion(x=0, y=0, z=0.7071068, w=0.7071068)
        vec_rotated_expected = vector3(0, 1.0, 0)

        vec_rotated = rotate_vector(vec, rot)
        vec_rotated = vector3(vec_rotated[0], vec_rotated[1], vec_rotated[2])
        self.assert_vector3s_equal(vec_rotated, vec_rotated_expected)

        # Test rotation with 0
        vec = vector3(0.0, 0, 0)
        rot = quaternion()
        # rot = Quaternion(x=5.800216634196169e-06, y=4.469788972803005e-06, z=0.47942493058978675, w=0.8775828940193382)
        vec_rotated_expected = vector3(0, 0, 0)
        vec_rotated = rotate_vector(vec, rot)
        vec_rotated = vector3(vec_rotated[0], vec_rotated[1], vec_rotated[2])
        self.assert_vector3s_equal(vec_rotated, vec_rotated_expected)

        # 
        vec = Vector3(x=0.0, y=0.0, z=-0.055)
        rot = Quaternion(x=0.0, y=0.0, z=0.47942493058978675, w=0.8775828940193382)
        vec_rotated = rotate_vector(vec, rot)
        print(vec_rotated)


if __name__ == "__main__":
    unittest.main()