from typing import List

from src.constants import *
from src.supporting_types import Sphere, Bottle, Cuboid
from src.hsrb_robot import HsrbRobot
from src.math_utils import vec_addition

from hsrb_interface.geometry import vector3, Vector3, Quaternion

USE_MIN_DISTANCE = False
USE_MAX_X = True


def get_bottle_to_grasp(robot: HsrbRobot, bottles: List[Bottle], print_header=True) -> Bottle:
    """Return the closest bottle to the robot"""
    if print_header:
        print("\n-----------------------------------")
        print("  --- Getting bottle to grasp ---  \n")

    if USE_MIN_DISTANCE:
        robot_xy = robot.base_xy_pose()
        robot_point = Vector3(x=robot_xy[0], y=robot_xy[1], z=DESK_HEIGHT)
        distances = [bottle.distance_to_point(robot_point) for bottle in bottles]
        min_idx = distances.index(min(distances))
        print(f"  returning the closest bottle to the robot ({round(min(distances), 4)} m away)")
        return bottles[min_idx]

    elif USE_MAX_X:
        xs = [bottle.tf.position.x for bottle in bottles]
        max_idx = xs.index(max(xs))
        print(f"  returning the bottle that is furthest down the table (x={round(max(xs), 2)})")
        return bottles[max_idx]


def am_currently_holding_a_bottle(robot: HsrbRobot) -> bool:
    raise NotImplementedError()


# TODO: Lower the z height
def get_bottle_place_location(clean_area: Cuboid, bottles: List[Bottle]) -> Vector3:
    """Return the location to place the bottle"""
    clean_area_midpoint = clean_area.midpoint
    extra_z = 0.05
    offset = vector3(x=0, y=-0.05, z=SODA_CAN_HEIGHT + extra_z)
    return vec_addition(vector3(clean_area_midpoint.x, clean_area_midpoint.y, DESK_HEIGHT + BASKET_HEIGHT), offset)
    # return vec_addition(clean_area.center, vector3(0, 0, SODA_CAN_HEIGHT))
    # if len(bottles) == 0:
    #     clean_area_midpoint = clean_area.midpoint
    #     return vector3(clean_area_midpoint.x, clean_area_midpoint.y, DESK_HEIGHT)
    # else:
    #     raise NotImplementedError()
