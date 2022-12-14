from typing import List

from src.supporting_types import Bottle, Transform
from src.hsrb_robot import HsrbRobot
from src.gazebo_utils import visualize_tf_set_in_rviz

from hsrb_interface.geometry import vector3, quaternion
import rospy
import tf

# rospy.init_node('ar_marker_listener')
listener = tf.TransformListener()


def _merge_bottles(bottles1: List[Bottle], bottles2: List[Bottle]) -> List[Bottle]:
    merged_bottles = bottles1
    b1_names = [b1.tf.name for b1 in bottles1]
    for b2 in bottles2:
        if b2.tf.name not in b1_names:
            merged_bottles.append(b2)
    return merged_bottles


def scan_for_bottles(robot: HsrbRobot, print_header: bool = False) -> List[Bottle]:
    if print_header:
        print("\n--------------------------------", flush=True)
        print("  --- Scanning for bottles ---  \n", flush=True)

    robot.move_to_go()

    bottle_tf_names = ["ar_marker/1", "ar_marker/2", "ar_marker/4000"]
    # bottle_tf_names = ['ar_marker/0001', 'ar_marker/0002', 'ar_marker/4000']
    bottles = []

    def get_visible_bottles():
        for bottle_tf_name in bottle_tf_names:
            try:
                trans, rot = listener.lookupTransform("map", bottle_tf_name, rospy.Time(0))
                qx, qy, qz, qw = rot
                bottle_tf = Transform(
                    bottle_tf_name,
                    position=vector3(*trans),
                    rotation=quaternion(x=qx, y=qy, z=qz, w=qw),
                )
                bottle_tf.shift_z(0.02)
                bottles.append(Bottle(tf=bottle_tf))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        return bottles

    all_bottles = []

    vp1 = vector3(x=3.3, y=-0.51, z=0.7)
    vp2 = vector3(x=2.87, y=-0.51, z=0.7)
    print("  looking at viewpoint 1 (vector3(x=3.3, y=-0.51, z=0.7))", flush=True)
    robot.whole_body.gaze_point(point=vp1, ref_frame_id="map")
    rospy.sleep(1.0)
    visible = get_visible_bottles()
    all_bottles = _merge_bottles(all_bottles, visible)
    print("  found {} bottles".format(len(visible)), flush=True)

    print("  looking at viewpoint 2", flush=True)
    robot.whole_body.gaze_point(point=vp2, ref_frame_id="map")
    rospy.sleep(1.0)
    visible = get_visible_bottles()
    all_bottles = _merge_bottles(all_bottles, visible)
    print("  found {} bottles".format(len(visible)), flush=True)

    visualize_tf_set_in_rviz("can_tfs", [bottle.tf for bottle in bottles])
    return bottles
