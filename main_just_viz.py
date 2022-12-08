from typing import List
from time import sleep, time
import threading

from src.gazebo_utils import (
    gazebo_bottle_poses_callback,
    visualize_cuboid_in_rviz,
    visualize_tf_set_in_rviz,
    visualize_tf_in_rviz,
)
from src.supporting_types import Bottle, Cuboid, Transform
from src.hsrb_robot import HsrbRobot
from src.perception import scan_for_bottles
from src.constants import *

from hsrb_interface.geometry import vector3
import hsrb_interface
import rospy
import gazebo_msgs.msg


_robot = hsrb_interface.Robot()
robot = HsrbRobot(_robot)

# Clean area is where the tray is
CLEAN_AREA = Cuboid(
    vector3(CLEAN_AREA_MIN_XY[0], CLEAN_AREA_MIN_XY[1], DESK_HEIGHT - 0.1),
    vector3(CLEAN_AREA_MAX_XY[0], CLEAN_AREA_MAX_XY[1], DESK_HEIGHT + SODA_CAN_HEIGHT + 0.1),
)
visualize_cuboid_in_rviz("clean_area", CLEAN_AREA)  # pubs to 'visualization_marker'

# Update the ground truth bottle poses (from gazebo) at a fixed frequency
BOTTLE_POSES_GT: List[Transform] = []
if GAZEBO_MODE:
    bottle_tf_update_frequency = 0.5
    last_published = None

    def callback_wrapper(data):
        global last_published, BOTTLE_POSES_GT
        if (
            last_published
            and bottle_tf_update_frequency > 0.0
            and time() - last_published <= 1.0 / bottle_tf_update_frequency
        ):
            return
        BOTTLE_POSES_GT = gazebo_bottle_poses_callback(data)
        last_published = time()
        visualize_tf_set_in_rviz("can_tfs", BOTTLE_POSES_GT)

    rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, callback_wrapper)

#
def scan_for_bottles_wrapper(robot: HsrbRobot) -> List[Bottle]:
    if not GAZEBO_MODE:
        return scan_for_bottles(robot)
    else:
        return [Bottle(pose) for pose in BOTTLE_POSES_GT]


""" Usage

# Terminal 1:
roslaunch hsrb_gazebo_launch hsrb_butler_bot_world.launch

# Terminal 2:
rostopic echo /clicked_point    # then 'Publish Point' in Rviz

# Terminal 3:
python3 main_just_viz.py
"""

if __name__ == "__main__":
    print("Sleeping for 1 seconds to allow the robot to initialize", flush=True)
    rospy.sleep(1.0)
    robot.say("Hello. My name is Mr. Butler. I will now clean the table")

    robot.move_to_neural()

    while True:
        sleep(0.01)
