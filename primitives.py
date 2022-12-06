from typing import List

from supporting_types import Cuboid, Bottle
from hsrb_robot import HsrbRobot


def get_bottle_to_grasp(bottles: List[Bottle]) -> Bottle:
    raise NotImplementedError()


def grasp_bottle(robot: HsrbRobot, bottle: Bottle) -> bool:
    raise NotImplementedError()


def am_currently_holding_a_bottle(robot: HsrbRobot) -> bool:
    raise NotImplementedError()


def place_currently_grasped_bottle_on_tray(robot: HsrbRobot, clean_area: Cuboid, cleaned_bottles: List[Bottle]):
    raise NotImplementedError()


def reset_to_neutral(robot: HsrbRobot):
    raise NotImplementedError()
