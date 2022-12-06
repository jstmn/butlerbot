from dataclasses import dataclass

from hsrb_interface.geometry import Vector3, Quaternion

import numpy as np


@dataclass
class Cuboid:
    xyz_min: np.ndarray
    xyz_max: np.ndarray


@dataclass
class Transform:
    name: str
    position: Vector3
    quat: Quaternion

    def __str__(self):
        return f"<Transform[{self.name}] position: {self.position}, rotation: {self.quat}>"


class Bottle:
    def __init__(self, pose: Transform) -> None:
        self._pose = pose

    @property
    def pose(self) -> Transform:
        return self._pose

    def in_area(self, area: Cuboid) -> bool:
        raise NotImplementedError()
