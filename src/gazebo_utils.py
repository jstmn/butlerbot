from typing import List
from src.constants import *
from src.supporting_types import Transform, Cuboid
from src.math_utils import rotate_vector

import rospy

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from hsrb_interface.geometry import quaternion, vector3

CAN_TF_OFFSET = vector3(x=GAZEBO_TO_RVIZ_CAN_OFFSET[0], y=GAZEBO_TO_RVIZ_CAN_OFFSET[1], z=GAZEBO_TO_RVIZ_CAN_OFFSET[2])


def gazebo_bottle_poses_callback(data, apply_offset=True) -> List[Transform]:
    """

    Notes:
        1. This function is only used in GAZEBO_MODE
        2. Gazebo coordinates are shifted by (5, 6.6, 0) relative to the map frame. With `apply_offset` set to True,
            this function will apply the offset `GAZEBO_TO_RVIZ_XY_OFFSET`
    """
    assert GAZEBO_MODE, "This function should only be called if GAZEBO_MODE is enabled"

    xy_offset = (0.0, 0.0)
    if apply_offset:
        xy_offset = GAZEBO_TO_RVIZ_XY_OFFSET

    poses = []
    for i in range(len(data.name)):
        ref_frame = data.name[i]
        if "TOGRASP" in ref_frame:
            rotation = quaternion(
                x=data.pose[i].orientation.x,
                y=data.pose[i].orientation.y,
                z=data.pose[i].orientation.z,
                w=data.pose[i].orientation.w,
            )
            rotated_tf_rotation = rotate_vector(CAN_TF_OFFSET, rotation)
            tf = Transform(
                name=data.name[i],
                position=vector3(
                    data.pose[i].position.x + xy_offset[0] + rotated_tf_rotation[0],
                    data.pose[i].position.y + xy_offset[1] + rotated_tf_rotation[1],
                    data.pose[i].position.z + rotated_tf_rotation[2],
                ),
                rotation=rotation,
            )
            poses.append(tf)
    return poses


# Code example from https://answers.ros.org/question/373802/minimal-working-example-for-rviz-marker-publishing/
rospy.init_node("rviz_marker")
marker_pubs = {"clean_area": rospy.Publisher("/clean_area", Marker, queue_size=2)}
marker_set_pubs = {"obstacle_world": rospy.Publisher("/obstacle_world", MarkerArray, queue_size=2)}

tf_pub = {
    "grasp_pose_tf": rospy.Publisher("/grasp_pose_tf", PoseStamped, queue_size=2),
}

tf_sets_pub = {"can_tfs": rospy.Publisher("/can_tfs", PoseArray, queue_size=2)}


def visualize_cuboids_in_rviz(alias: str, cubes: List[Cuboid]):
    assert alias in marker_set_pubs, f"Invalid alias: {alias}"
    markers = [cube.get_ros_marker(frame_id="map") for cube in cubes]
    marker_array = MarkerArray(markers=markers)
    marker_set_pubs[alias].publish(marker_array)


def visualize_cuboid_in_rviz(alias: str, cube: Cuboid):
    marker = cube.get_ros_marker(frame_id="map")
    marker_pubs[alias].publish(marker)


def visualize_tf_in_rviz(topic_name: str, tf: Transform):
    assert topic_name in tf_pub, f"Invalid topic name: {topic_name}"
    tf_ros = PoseStamped(pose=tf.pose_rosformat())
    tf_ros.header.frame_id = "map"
    tf_ros.header.stamp = rospy.Time.now()
    tf_pub[topic_name].publish(tf_ros)


def visualize_tf_set_in_rviz(topic_name: str, tfs: List[Transform]):
    assert topic_name in tf_sets_pub, f"Invalid topic name: {topic_name}"
    stamped_tfs_ros = PoseArray(poses=[tf.pose_rosformat() for tf in tfs])
    stamped_tfs_ros.header.frame_id = "map"
    stamped_tfs_ros.header.stamp = rospy.Time.now()
    tf_sets_pub[topic_name].publish(stamped_tfs_ros)
