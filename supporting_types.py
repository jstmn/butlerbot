from dataclasses import dataclass

from hsrb_interface import Robot
from hsrb_interface.mobile_base import MobileBase
from hsrb_interface.joint_group import JointGroup
from hsrb_interface.end_effector import Gripper
# from hsrb_interface.geometry import Vector3, Quaternion

import rospy
import sys

import numpy as np


class HsrbRobotWrapper:

    def __init__(self, robot: Robot) -> None:
        self._robot = robot
        self._omni_base = robot.get('omni_base')
        self._whole_body = robot.get('whole_body')
        self._gripper = robot.get('gripper')
        self._speaker = robot.get('default_tts')

    @property
    def robot(self) -> Robot:
        return self._robot

    @property
    def omni_base(self) -> MobileBase:
        return self._omni_base

    @property
    def whole_body(self) -> JointGroup:
        return self._whole_body

    @property
    def gripper(self) -> Gripper:
        return self._gripper

    def say(self, text: str) -> None:
        self._speaker.say(text)



@dataclass
class Cuboid:
    xyz_min: np.ndarray
    xyz_max: np.ndarray


@dataclass
class PoseNp:
    position: np.ndarray
    quat: np.ndarray

    def __str__(self):
        return f'position: {self.position}, rotation: {self.quat}'


class Bottle:
    def __init__(self, pose: PoseNp) -> None:
        self._pose = pose

    @property
    def pose_np(self) -> PoseNp:
        return self._pose
    
    def in_area(self, area: Cuboid) -> bool:
        raise NotImplementedError()
        return False