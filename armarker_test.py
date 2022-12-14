from typing import List
from time import sleep, time

from src.gazebo_utils import (
    gazebo_bottle_poses_callback,
    visualize_cuboid_in_rviz,
    visualize_tf_set_in_rviz,
)
from src.supporting_types import Bottle, Cuboid, Transform
from src.hsrb_robot import HsrbRobot
from src.primitives import (
    get_bottle_to_grasp,
    get_bottle_place_location,
)
from src.perception import scan_for_bottles
from src.constants import *

from hsrb_interface.geometry import vector3
import hsrb_interface
import rospy
import gazebo_msgs.msg

#
_robot = hsrb_interface.Robot()
robot = HsrbRobot(_robot, Cuboid(xyz_min=vector3(100, 100, 100), xyz_max=vector3(101, 101, 101)))


#
def scan_for_bottles_wrapper(robot: HsrbRobot, print_header: bool = False) -> List[Bottle]:
    if print_header:
        print("\n--------------------------------")
        print("  --- Scanning for bottles ---  ")
    if not GET_BOTTLE_POSES_FROM_GAZEBO:
        return scan_for_bottles(robot)
    else:
        assert False


""" Usage

# Terminal 1:
roslaunch hsrb_gazebo_launch hsrb_butler_bot_world.launch

# After every new gazebo roslaunch, run 
python3 armarker_test.py
"""

if __name__ == "__main__":
    robot.say("Hello. My name is Mr. Butler. I will now clean the table")

    # Move to the table
    sleep(0.5)

    while True:
        # Find all bottles
        bottles = scan_for_bottles_wrapper(robot, print_header=True)

        sleep(5.0)
