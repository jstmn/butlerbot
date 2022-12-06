from typing import List
from src.constants import *
from src.supporting_types import Transform, Cuboid

import rospy

from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from hsrb_interface.geometry import quaternion, vector3


def gazebo_bottle_poses_callback(data, apply_offset=True):
    """

    Notes:
        1. This function is only used in GAZEBO_MODE
        2. Gazebo coordinates are shifted by (5, 6.6, 0) relative to the map frame. With `apply_offset` set to True,
            this function will apply the offset `GAZEBO_TO_RVIZ_XY_OFFSET`
    """
    assert GAZEBO_MODE, "This function should only be called if GAZEBO_MODE is enabled"

    offset = (0.0, 0.0)
    if apply_offset:
        offset = GAZEBO_TO_RVIZ_XY_OFFSET

    poses = []
    for i in range(len(data.name)):
        ref_frame = data.name[i]
        if "TOGRASP" in ref_frame:
            tf = Transform(
                name=data.name[i],
                position=vector3(
                    data.pose[i].position.x + offset[0], data.pose[i].position.y + offset[1], data.pose[i].position.z
                ),
                quat=quaternion(
                    x=data.pose[i].orientation.x,
                    y=data.pose[i].orientation.y,
                    z=data.pose[i].orientation.z,
                    w=data.pose[i].orientation.w,
                ),
            )
            poses.append(tf)
    return poses


# Code example from https://answers.ros.org/question/373802/minimal-working-example-for-rviz-marker-publishing/
rospy.init_node("rviz_marker")
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
tf_pub = rospy.Publisher("/visualization_tf", PoseArray, queue_size=2)


def visualize_cuboid_in_rviz(cube: Cuboid):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 1
    marker.id = 0

    # Set the scale of the marker
    widths = cube.widths()
    marker.scale.x = widths.x
    marker.scale.y = widths.y
    marker.scale.z = widths.z

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.5

    # Set the pose of the marker
    midpoint = cube.midpoint()
    marker.pose.position.x = midpoint.x
    marker.pose.position.y = midpoint.y
    marker.pose.position.z = midpoint.z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker_pub.publish(marker)
    print(f"visualize_cuboid_in_rviz(): publishing cube {cube}")


def visualize_tfs_in_rviz(tfs: List[Transform]):
    stamped_tfs_ros = PoseArray(poses=[tf.pose_rosformat() for tf in tfs])
    stamped_tfs_ros.header.frame_id = "map"
    stamped_tfs_ros.header.stamp = rospy.Time.now()
    tf_pub.publish(stamped_tfs_ros)
    print(f"visualize_tfs_in_rviz(): publishing tfs")


# if __name__ == "__main__":
#     CLEAN_AREA = Cuboid(vector3(CLEAN_AREA_MIN_XY[0], CLEAN_AREA_MIN_XY[1], 0.0), vector3(CLEAN_AREA_MAX_XY[0], CLEAN_AREA_MAX_XY[1], 1.5))
#     visualize_cuboid_in_rviz(CLEAN_AREA)
