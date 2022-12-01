from typing import List

from supporting_types import HsrbRobotWrapper, Cuboid, Bottle

def move_to_table(robot_wr: HsrbRobotWrapper):
    raise NotImplementedError()

def get_bottle_to_grasp(bottles: List[Bottle]) -> Bottle:
    raise NotImplementedError()

def grasp_bottle(robot_wr: HsrbRobotWrapper, bottle: Bottle) -> bool:
    raise NotImplementedError()

def am_currently_holding_a_bottle(robot_wr: HsrbRobotWrapper) -> bool:
    raise NotImplementedError()

def place_currently_grasped_bottle_on_tray(robot_wr: HsrbRobotWrapper, clean_area: Cuboid, cleaned_bottles: List[Bottle]):
    raise NotImplementedError()

def reset_to_neutral(robot_wr: HsrbRobotWrapper):
    raise NotImplementedError()