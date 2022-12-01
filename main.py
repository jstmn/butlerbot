import hsrb_interface
import rospy
import sys

from supporting_types import HsrbRobotWrapper, Cuboid
from primitives import (
    move_to_table, 
    get_bottle_to_grasp,
    grasp_bottle,
    am_currently_holding_a_bottle,
    place_currently_grasped_bottle_on_tray,
    reset_to_neutral
)
from perception import scan_for_bottles

import numpy as np

robot = hsrb_interface.Robot()
robot_wr = HsrbRobotWrapper(robot)
clean_area = Cuboid(np.zeros(3), np.zeros(3)) # TODO: fill in the values


if __name__ == "__main__":

    rospy.sleep(5.0)
    robot_wr.say('Hello. My name is Mr. Butler. I will now clean the table')

    move_to_table(robot_wr)

    while True:

        # Find all bottles
        bottles = scan_for_bottles(robot_wr)
        
        # Find all bottles that are out of place
        out_of_place_bottles = [bottle for bottle in bottles if not bottle.in_area(clean_area)]
        cleaned_bottles = [bottle for bottle in bottles if bottle.in_area(clean_area)]
        assert len(out_of_place_bottles) + len(cleaned_bottles) == len(bottles)

        # Mission complete if there are no bottles out of place
        if len(out_of_place_bottles) == 0:
            robot_wr.say("All bottles are in now in place. Mission complete")
            break

        bottle_to_grasp = get_bottle_to_grasp(out_of_place_bottles)

        # Grasp the bottle
        grasp_successful = grasp_bottle(robot_wr, bottle_to_grasp)
        if not grasp_successful:
            robot_wr.say("I was unable to grasp the bottle")
            # TODO: What to do in this case
            break

        # Verify that we are holding a bottle
        holding_a_bottle = am_currently_holding_a_bottle(robot_wr) 
        if not holding_a_bottle:
            robot_wr.say("I have detected that I am not holding a bottle, even though I believed my grasp was successful")
            # TODO: What to do in this case
            break
        
        place_currently_grasped_bottle_on_tray(robot_wr, clean_area, cleaned_bottles)
        reset_to_neutral(robot_wr)

        # Verify that we are holding a bottle
        holding_a_bottle = am_currently_holding_a_bottle(robot_wr) 
        if holding_a_bottle:
            robot_wr.say("I have detected I am holding a bottle, which I should not be")
            # TODO: What to do in this case
            break
