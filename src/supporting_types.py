from dataclasses import dataclass

from src.constants import *
from src.math_utils import distance_between_vector3s

from hsrb_interface.geometry import Vector3, Quaternion
from hsrb_interface.geometry import Pose as HsrbPose
from geometry_msgs.msg import Point as RosPoint
from geometry_msgs.msg import Quaternion as RosQuaternion
from geometry_msgs.msg import (
    Pose as RosPose,
)  # renamed to `RosPose` to avoid confusion with hsrb_interface.geometry.Pose


@dataclass
class Cuboid:
    xyz_min: Vector3
    xyz_max: Vector3

    @property
    def widths(self) -> Vector3:
        return Vector3(
            x=self.xyz_max.x - self.xyz_min.x, y=self.xyz_max.y - self.xyz_min.y, z=self.xyz_max.z - self.xyz_min.z
        )

    @property
    def midpoint(self) -> Vector3:
        return Vector3(
            x=(self.xyz_min.x + self.xyz_max.x) / 2,
            y=(self.xyz_min.y + self.xyz_max.y) / 2,
            z=(self.xyz_min.z + self.xyz_max.z) / 2,
        )

    def enclosed_in(self, other: "Cuboid") -> bool:
        return (
            self.xyz_min.x >= other.xyz_min.x
            and self.xyz_min.y >= other.xyz_min.y
            and self.xyz_min.z >= other.xyz_min.z
            and self.xyz_max.x <= other.xyz_max.x
            and self.xyz_max.y <= other.xyz_max.y
            and self.xyz_max.z <= other.xyz_max.z
        )

    def __post_init__(self):
        assert self.xyz_min.x <= self.xyz_max.x
        assert self.xyz_min.y <= self.xyz_max.y
        assert self.xyz_min.z <= self.xyz_max.z


@dataclass
class Transform:
    name: str
    position: Vector3
    rotation: Quaternion

    def position_rosformat(self) -> RosPoint:
        return RosPoint(self.position.x, self.position.y, self.position.z)

    def orientation_rosformat(self) -> RosQuaternion:
        return RosQuaternion(x=self.rotation.x, y=self.rotation.y, z=self.rotation.z, w=self.rotation.w)

    def pose_rosformat(self) -> RosPose:
        return RosPose(position=self.position_rosformat(), orientation=self.orientation_rosformat())

    def pose_hsrb_format(self) -> HsrbPose:
        return HsrbPose(self.position, self.rotation)

    def __str__(self):
        return f"[Transform[{self.name}] position: {self.position}, rotation: {self.rotation}]"


class Bottle:
    """The reference frame of the bottle is attached to the bottom of the can in the center, with +z pointing up."""

    def __init__(self, tf: Transform, diameter: float = SODA_CAN_WIDTH, height: float = SODA_CAN_HEIGHT) -> None:
        self._tf = tf
        self._diameter = diameter
        self._height = height

        self._bounding_cube = Cuboid(
            xyz_min=Vector3(
                x=self._tf.position.x - (diameter / 2), y=self._tf.position.y - (diameter / 2), z=self._tf.position.z
            ),
            xyz_max=Vector3(
                x=self._tf.position.x + (diameter / 2),
                y=self._tf.position.y + (diameter / 2),
                z=self._tf.position.z + height,
            ),
        )

    @property
    def tf(self) -> Transform:
        return self._tf

    def in_area(self, cube: Cuboid) -> bool:
        return self._bounding_cube.enclosed_in(cube)

    def distance_to_point(self, point: Vector3) -> float:
        return distance_between_vector3s(self._tf.position, point)

    def __str__(self):
        return f"<Bottle[] tf: '{self.tf}'>"
