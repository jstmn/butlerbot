from typing import Union, Tuple
from dataclasses import dataclass

from src.constants import *
from src.math_utils import vector_distance

from hsrb_interface.geometry import vector3, Vector3, Quaternion
from hsrb_interface.geometry import Pose as HsrbPose
from geometry_msgs.msg import Point as RosPoint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion as RosQuaternion
from geometry_msgs.msg import (
    Pose as RosPose,
)  # renamed to `RosPose` to avoid confusion with hsrb_interface.geometry.Pose
import rospy


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

    # TODO(@jstm): Will this update the padding multiple times? That would be bad.
    def add_x_padding(self, padding: float):
        self.xyz_min = vector3(x=self.xyz_min.x - padding, y=self.xyz_min.y, z=self.xyz_min.z)
        self.xyz_max = vector3(x=self.xyz_max.x + padding, y=self.xyz_max.y, z=self.xyz_max.z)

    def add_y_padding(self, padding: float):
        self.xyz_min = vector3(x=self.xyz_min.x, y=self.xyz_min.y - padding, z=self.xyz_min.z)
        self.xyz_max = vector3(x=self.xyz_max.x, y=self.xyz_max.y + padding, z=self.xyz_max.z)

    def add_z_padding(self, padding: float):
        self.xyz_min = vector3(x=self.xyz_min.x, y=self.xyz_min.y, z=self.xyz_min.z - padding)
        self.xyz_max = vector3(x=self.xyz_max.x, y=self.xyz_max.y, z=self.xyz_max.z + padding)

    def get_ros_marker(
        self, frame_id: str = "map", id: int = 0, color: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.2)
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 1
        marker.id = id

        # Set the scale of the marker
        widths = self.widths
        marker.scale.x = widths.x
        marker.scale.y = widths.y
        marker.scale.z = widths.z

        # Set the color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        # Set the pose of the marker
        midpoint = self.midpoint
        marker.pose.position.x = midpoint.x
        marker.pose.position.y = midpoint.y
        marker.pose.position.z = midpoint.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        return marker

    def enclosed_in(self, other: Union["Cuboid", "Sphere"]) -> bool:
        if isinstance(other, Sphere):
            sphere_square_approx = Cuboid(
                xyz_min=vector3(
                    x=other.center.x - other.radius, y=other.center.y - other.radius, z=other.center.z - other.radius
                ),
                xyz_max=vector3(
                    x=other.center.x + other.radius, y=other.center.y + other.radius, z=other.center.z + other.radius
                ),
            )
            return self.enclosed_in(sphere_square_approx)

        elif isinstance(other, Cuboid):
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
        assert isinstance(self.xyz_min, Vector3)
        assert isinstance(self.xyz_max, Vector3)


@dataclass
class Sphere:
    center: Vector3
    radius: float

    def __post_init__(self):
        assert isinstance(self.center, Vector3)
        assert isinstance(self.radius, float)
        assert self.radius >= 0


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

    def __post_init__(self):
        assert isinstance(self.position, Vector3)
        assert isinstance(self.rotation, Quaternion)


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

    def xy_coords(self) -> Tuple[float, float]:
        return self._tf.position.x, self._tf.position.y

    def in_area(self, cube: Union[Cuboid, Sphere]) -> bool:
        return self._bounding_cube.enclosed_in(cube)

    def distance_to_point(self, point: Vector3) -> float:
        return vector_distance(self._tf.position, point)

    def __str__(self):
        return f"<Bottle[] tf: '{self.tf}'>"
