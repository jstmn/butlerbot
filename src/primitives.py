from typing import List

from src.constants import *
from src.supporting_types import Cuboid, Bottle
from src.hsrb_robot import HsrbRobot

from hsrb_interface.geometry import Vector3, Quaternion


def get_bottle_to_grasp(robot: HsrbRobot, bottles: List[Bottle]) -> Bottle:
    """Return the closest bottle to the robot"""
    robot_xy = robot.base_xy_pose()
    robot_point = Vector3(x=robot_xy[0], y=robot_xy[1], z=DESK_HEIGHT)
    distances = [bottle.distance_to_point(robot_point) for bottle in bottles]
    min_idx = distances.index(min(distances))
    return bottles[min_idx]


def am_currently_holding_a_bottle(robot: HsrbRobot) -> bool:
    raise NotImplementedError()


def place_currently_grasped_bottle_on_tray(robot: HsrbRobot, clean_area: Cuboid, cleaned_bottles: List[Bottle]):
    raise NotImplementedError()


def reset_to_neutral(robot: HsrbRobot):
    raise NotImplementedError()
