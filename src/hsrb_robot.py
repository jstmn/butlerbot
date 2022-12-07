from typing import Tuple

from src.supporting_types import Cuboid, Bottle

from hsrb_interface import Robot
from hsrb_interface.mobile_base import MobileBase
from hsrb_interface.joint_group import JointGroup
from hsrb_interface.end_effector import Gripper
from hsrb_interface.geometry import Vector3, Quaternion

_MOVE_TIMEOUT = 60.0


class HsrbRobot:
    def __init__(self, robot: Robot) -> None:
        self._robot = robot
        self._omni_base = robot.get("omni_base")
        self._whole_body = robot.get("whole_body")
        self._gripper = robot.get("gripper")
        self._speaker = robot.get("default_tts")

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

    def base_xy_pose(self) -> Tuple[float, float]:
        base_pose, _ = self.omni_base.get_pose()
        return base_pose.x, base_pose.y

    # ------------------------------------------------------------------------------------------------------------------
    #                                               Actions

    def grasp_bottle(self, bottle: Bottle) -> bool:
        raise NotImplementedError()

    def move_base_to(self, pose: Tuple[float, float, float]):
        print("HsrbRobot.move_base_to() - Moving to pose:", pose, flush=True)
        print("                         - Current pose:", self.omni_base.get_pose(), flush=True)
        print("                         - Moving to go state", flush=True)
        try:
            self.whole_body.move_to_go()
        except Exception as e:
            print("HsrbRobot.move_base_to() - failed to move_to_go():", e, flush=True)
            raise e

        self.omni_base.go_abs(x=pose[0], y=pose[1], yaw=pose[2], timeout=_MOVE_TIMEOUT)

    def say(self, text: str) -> None:
        print(f"HsrbRobot.say(): '{text}'")
        self._speaker.say(text)
