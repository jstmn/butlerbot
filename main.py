from typing import List
from time import sleep, time

from src.gazebo_utils import gazebo_bottle_poses_callback
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

# from hsrb_interface import geometry
import hsrb_interface
import rospy
import numpy as np
import gazebo_msgs.msg


PI = np.pi
GAZEBO_MODE = True

_robot = hsrb_interface.Robot()
robot = HsrbRobot(_robot)
clean_area = Cuboid(np.zeros(3), np.zeros(3))  # TODO: fill in the values


DESK_TARGET_POSE = (2.75, 0.25, 3 / 2 * PI)  # (x, y, yaw)
# DESK_TARGET_POSE = (1.0, 0, 0) # (x, y, yaw)
# DESK_TARGET_POSE = (8.0, 6.7, -1.567) # (x, y, yaw)


BOTTLE_POSES_GT = []
bottle_tf_update_frequency = 10.0
last_published = None

if GAZEBO_MODE:
    def callback_wrapper(data):
        global last_published, BOTTLE_POSES_GT
        if last_published and bottle_tf_update_frequency > 0.0 and time() - last_published <= 1.0 / bottle_tf_update_frequency:
            return
        BOTTLE_POSES_GT = gazebo_bottle_poses_callback(data)
        last_published = time()

    rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, callback_wrapper)


def scan_for_bottles_wrapper(robot: HsrbRobot) -> List[Bottle]:
    if not GAZEBO_MODE:
        return scan_for_bottles(robot)
    else:
        return [Bottle(pose) for pose in BOTTLE_POSES_GT]


"""
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

        # Find all bottles that are out of place
        out_of_place_bottles = [bottle for bottle in bottles if not bottle.in_area(clean_area)]
        cleaned_bottles = [bottle for bottle in bottles if bottle.in_area(clean_area)]
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

        place_currently_grasped_bottle_on_tray(robot, clean_area, cleaned_bottles)
        reset_to_neutral(robot)

        # Verify that we are holding a bottle
        holding_a_bottle = am_currently_holding_a_bottle(robot)
        if holding_a_bottle:
            robot.say("I have detected I am holding a bottle, which I should not be")
            # TODO: What to do in this case
            break
