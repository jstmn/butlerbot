from typing import List

from src.constants import *
from src.supporting_types import Cuboid, Bottle
from src.hsrb_robot import HsrbRobot

from hsrb_interface.geometry import vector3, Vector3, Quaternion


def get_bottle_to_grasp(robot: HsrbRobot, bottles: List[Bottle]) -> Bottle:
    """Return the closest bottle to the robot"""
    robot_xy = robot.base_xy_pose()
    robot_point = Vector3(x=robot_xy[0], y=robot_xy[1], z=DESK_HEIGHT)
    distances = [bottle.distance_to_point(robot_point) for bottle in bottles]
    min_idx = distances.index(min(distances))
    return bottles[min_idx]


def am_currently_holding_a_bottle(robot: HsrbRobot) -> bool:
    raise NotImplementedError()


def get_bottle_place_location(clean_area: Cuboid, bottles: List[Bottle]) -> Vector3:
    """Return the location to place the bottle"""
    if len(bottles) == 0:
        clean_area_midpoint = clean_area.midpoint
        return vector3(clean_area_midpoint.x, clean_area_midpoint.y, DESK_HEIGHT)
    else:
        raise NotImplementedError()
