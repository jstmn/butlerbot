from typing import List
from time import sleep, time

from src.gazebo_utils import gazebo_bottle_poses_callback, visualize_cuboid_in_rviz, visualize_tfs_in_rviz
from src.supporting_types import Bottle, Cuboid
from src.hsrb_robot import HsrbRobot
from src.primitives import (
    get_bottle_to_grasp,
    grasp_bottle,
    am_currently_holding_a_bottle,
    place_currently_grasped_bottle_on_tray,
    reset_to_neutral,
)
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
visualize_cuboid_in_rviz(CLEAN_AREA)

# Update the ground truth bottle poses (from gazebo) at a fixed frequency
BOTTLE_POSES_GT = []
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
        visualize_tfs_in_rviz(BOTTLE_POSES_GT)

    rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, callback_wrapper)


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


"""

if __name__ == "__main__":
    print("Sleeping for 1 seconds to allow the robot to initialize", flush=True)
    rospy.sleep(1.0)
    robot.say("Hello. My name is Mr. Butler. I will now clean the table")

    # Move to the table
    robot.move_base_to(DESK_TARGET_POSE)

    while True:
        # Find all bottles
        bottles = scan_for_bottles_wrapper(robot)
        robot.say(f"Located {len(bottles)} bottles")

        # print("sleeping for 30 seconds", flush=True)
        # sleep(30)

        # Find all bottles that are out of place
        out_of_place_bottles = [bottle for bottle in bottles if not bottle.in_area(CLEAN_AREA)]
        cleaned_bottles = [bottle for bottle in bottles if bottle.in_area(CLEAN_AREA)]
        assert len(out_of_place_bottles) + len(cleaned_bottles) == len(bottles)

        # Mission complete if there are no bottles out of place
        if len(out_of_place_bottles) == 0:
            robot.say("All bottles are in now in place. Mission complete")
            break

        bottle_to_grasp = get_bottle_to_grasp(out_of_place_bottles)

        # Grasp the bottle
        grasp_successful = grasp_bottle(robot, bottle_to_grasp)
        if not grasp_successful:
            robot.say("I was unable to grasp the bottle")
            # TODO: What to do in this case
            break

        # Verify that we are holding a bottle
        holding_a_bottle = am_currently_holding_a_bottle(robot)
        if not holding_a_bottle:
            robot.say("I have detected that I am not holding a bottle, even though I believed my grasp was successful")
            # TODO: What to do in this case
            break

        place_currently_grasped_bottle_on_tray(robot, CLEAN_AREA, cleaned_bottles)
        reset_to_neutral(robot)

        # Verify that we are holding a bottle
        holding_a_bottle = am_currently_holding_a_bottle(robot)
        if holding_a_bottle:
            robot.say("I have detected I am holding a bottle, which I should not be")
            # TODO: What to do in this case
            break
