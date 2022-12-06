from dataclasses import dataclass

from src.constants import *

from hsrb_interface.geometry import Vector3, Quaternion
from geometry_msgs.msg import Point as RosPoint
from geometry_msgs.msg import Quaternion as RosQuaternion
from geometry_msgs.msg import (
    Pose as RosPose,
)  # renamed to `RosPose` to avoid confusion with hsrb_interface.geometry.Pose


@dataclass
class Cuboid:
    xyz_min: Vector3
    xyz_max: Vector3

    def widths(self) -> Vector3:
        return Vector3(
            x=self.xyz_max.x - self.xyz_min.x, y=self.xyz_max.y - self.xyz_min.y, z=self.xyz_max.z - self.xyz_min.z
        )

    def midpoint(self) -> Vector3:
        return Vector3(
            x=(self.xyz_min.x + self.xyz_max.x) / 2,
            y=(self.xyz_min.y + self.xyz_max.y) / 2,
            z=(self.xyz_min.z + self.xyz_max.z) / 2,
        )

    def __post_init__(self):
        assert self.xyz_min.x <= self.xyz_max.x
        assert self.xyz_min.y <= self.xyz_max.y
        assert self.xyz_min.z <= self.xyz_max.z


@dataclass
class Transform:
    name: str
    position: Vector3
    quat: Quaternion

    def position_rosformat(self) -> RosPoint:
        return RosPoint(self.position.x, self.position.y, self.position.z)

    def orientation_rosformat(self) -> RosQuaternion:
        return RosQuaternion(x=self.quat.x, y=self.quat.y, z=self.quat.z, w=self.quat.w)

    def pose_rosformat(self) -> RosPose:
        return RosPose(position=self.position_rosformat(), orientation=self.orientation_rosformat())

    def __str__(self):
        return f"[Transform[{self.name}] position: {self.position}, rotation: {self.quat}]"


class Bottle:
    """The reference frame of the bottle is attached to ___"""

    def __init__(self, tf: Transform, width: float = SODA_CAN_WIDTH, height: float = SODA_CAN_HEIGHT) -> None:
        self._tf = tf
        self._width = width
        self._height = height

    @property
    def tf(self) -> Transform:
        return self._tf

    # TODO(@jstm): Implement
    def in_area(self, area: Cuboid) -> bool:
        return False
