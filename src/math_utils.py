from typing import List, Tuple

from hsrb_interface.geometry import quaternion, vector3, Vector3
import tf

import numpy as np

ZERO_VECTOR3 = vector3(0, 0, 0)


def vec_addition(v1: Vector3, v2: Vector3) -> Vector3:
    return vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)


def vec_negation(v: Vector3) -> Vector3:
    return vector3(-v.x, -v.y, -v.z)


def vec_scaled(v: Vector3, alpha: float) -> Vector3:
    return vector3(v.x * alpha, v.y * alpha, v.z * alpha)


def vec_distance(v: Vector3) -> Vector3:
    return np.sqrt(v.x**2 + v.y**2 + v.z**2)


def distance_between_vector3s(v1: Vector3, v2: Vector3) -> float:
    return np.sqrt((v1.x - v2.x) ** 2 + (v1.y - v2.y) ** 2 + (v1.z - v2.z) ** 2)


# rotate vector v1 by quaternion q1
def _qv_mult(q1: List, v1: List) -> np.ndarray:
    # from https://answers.ros.org/question/196149/how-to-rotate-vector-by-quaternion-in-python/?answer=196155#post-id-196155
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2), tf.transformations.quaternion_conjugate(q1)
    )[:3]


def rotate_vector(v: vector3, q: quaternion) -> Tuple[float, float, float]:
    # 'Quaternions ix+jy+kz+w are represented as [x, y, z, w].'
    if distance_between_vector3s(v, ZERO_VECTOR3) < 1e-6:
        return (0, 0, 0)
    q_ros = [q.x, q.y, q.z, q.w]
    v_ros = np.array([v.x, v.y, v.z])
    return _qv_mult(q_ros, v_ros)
