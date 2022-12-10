from typing import List
from time import sleep, time

from src.gazebo_utils import (
    gazebo_bottle_poses_callback,
    visualize_cuboid_in_rviz,
    visualize_tf_set_in_rviz,
)
from src.supporting_types import Bottle, Cuboid, Transform, Sphere
from src.hsrb_robot import HsrbRobot
from src.primitives import (
    get_bottle_to_grasp,
    am_currently_holding_a_bottle,
    get_bottle_place_location,
)
from src.perception import scan_for_bottles
from src.constants import *

from hsrb_interface.geometry import vector3
import hsrb_interface
import rospy
import gazebo_msgs.msg


# Clean area is where the trash can is
_TABLE_AREA = Cuboid(
    xyz_min=vector3(DESK_MIN_XY[0], DESK_MIN_XY[1], DESK_HEIGHT - 2 * SODA_CAN_HEIGHT),
    xyz_max=vector3(DESK_MAX_XY[0], DESK_MAX_XY[1], DESK_HEIGHT + 2 * SODA_CAN_HEIGHT),
)
_BASKET_AREA = Cuboid(
    xyz_min=vector3(BASKET_XY[0] - BASKET_RADIUS, BASKET_XY[1] - BASKET_RADIUS, DESK_HEIGHT),
    xyz_max=vector3(BASKET_XY[0] + BASKET_RADIUS, BASKET_XY[1] + BASKET_RADIUS, DESK_HEIGHT + BASKET_HEIGHT),
)
_BASKET_AREA.add_x_padding(0.05)
_BASKET_AREA.add_y_padding(0.05)
CLEAN_AREA = _BASKET_AREA
visualize_cuboid_in_rviz("clean_area", CLEAN_AREA)  # pubs to 'visualization_marker'

#
_robot = hsrb_interface.Robot()
robot = HsrbRobot(_robot, _BASKET_AREA)

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
def scan_for_bottles_wrapper(robot: HsrbRobot, print_header: bool = False) -> List[Bottle]:
    if print_header:
        print("\n--------------------------------")
        print("  --- Scanning for bottles ---  ")
    if not GAZEBO_MODE:
        return scan_for_bottles(robot)
    else:
        return [Bottle(pose) for pose in BOTTLE_POSES_GT]


""" Usage

# Terminal 1:
roslaunch hsrb_gazebo_launch hsrb_butler_bot_world.launch

# Terminal 2:
rostopic echo /clicked_point    # then 'Publish Point' in Rviz

# After every new gazebo roslaunch, run 
python3 initialize_robot.py

# After initialize_robot.py finishes:
python3 main.py
"""

if __name__ == "__main__":
    print("Sleeping for 1 seconds to allow the robot to initialize", flush=True)
    rospy.sleep(1.0)
    robot.say("Hello. My name is Mr. Butler. I will now clean the table")

    # Move to the table
    robot.move_base_to(DESK_TARGET_POSE, print_header=True)

    while True:
        # Find all bottles
        bottles = scan_for_bottles_wrapper(robot, print_header=True)

        # Find all bottles that are out of place
        out_of_place_bottles = [
            bottle for bottle in bottles if (bottle.in_area(_TABLE_AREA) and not bottle.in_area(_BASKET_AREA))
        ]
        cleaned_bottles = [bottle for bottle in bottles if bottle.in_area(CLEAN_AREA)]
        dropped_bottles = [
            bottle for bottle in bottles if (not bottle.in_area(CLEAN_AREA) and not bottle.in_area(_TABLE_AREA))
        ]
        robot.say(
            f"Located {len(cleaned_bottles)} clean bottles, {len(out_of_place_bottles)} out of place bottles on the"
            f" table, and {len(dropped_bottles)} bottles elsewhere"
        )

        # Mission complete if there are no bottles out of place
        if len(out_of_place_bottles) == 0:
            robot.say("All bottles are in now in place. Mission complete")
            break

        bottle_to_grasp = get_bottle_to_grasp(robot, out_of_place_bottles, print_header=True)

        # Look at the bottle. Not necessary, but makes it easier to see what the robot is doing and is cool
        robot.look_at(bottle_to_grasp.tf.position)

        # Grasp the bottle
        grasp_successful = robot.grasp_bottle(bottle_to_grasp, print_header=True)
        if not grasp_successful:
            # TODO: What to do in this case
            break

        bottle_place_location = get_bottle_place_location(CLEAN_AREA, cleaned_bottles)
        print(f"Bottle place location: {bottle_place_location}", flush=True)

        bottled_placed = robot.place_currently_grasped_bottle(bottle_place_location, cleaned_bottles, print_header=True)
        if not bottled_placed:
            print("Failed to place bottle, aborting", flush=True)
            break

    robot.say("Mission complete")


# Verify that we are holding a bottle
# holding_a_bottle = am_currently_holding_a_bottle(robot)
# if not holding_a_bottle:
#     robot.say("I have detected that I am not holding a bottle, even though I believed my grasp was successful")
#     # TODO: What to do in this case
#     break

# Verify that we are holding a bottle
# holding_a_bottle = am_currently_holding_a_bottle(robot)
# if holding_a_bottle:
#     robot.say("I have detected I am holding a bottle, which I should not be")
#     # TODO: What to do in this case
#     break
