from typing import Tuple

from src.supporting_types import Cuboid, Bottle, Transform
from src.gazebo_utils import visualize_tf_in_rviz
from src.constants import *

from hsrb_interface import Robot
from hsrb_interface.mobile_base import MobileBase
from hsrb_interface.joint_group import JointGroup
from hsrb_interface.end_effector import Gripper
from hsrb_interface.collision_world import CollisionWorld
from hsrb_interface.geometry import Vector3, vector3, quaternion
from hsrb_interface.geometry import Pose as HsrbPose


_MOVE_TIMEOUT = 30.0

_END_EFF_FLAT_ROTATION = quaternion(
    x=-0.5474032817174194, y=0.44796588933801107, z=-0.547040587681599, w=-0.44768605582883053
)


def get_collision_world() -> CollisionWorld:
    if GAZEBO_MODE:
        table_surface_c = Cuboid(
            # xyz_min=vector3(DESK_MIN_XY[0], DESK_MIN_XY[1], DESK_HEIGHT - 0.05),
            xyz_min=vector3(DESK_MIN_XY[0], DESK_MIN_XY[1], 0.0),
            xyz_max=vector3(DESK_MAX_XY[0], DESK_MAX_XY[1], DESK_HEIGHT),
        )
        obstacles = [table_surface_c]
        obstacle_names = ["table_surface"]

        # Create and instantiate the collision world
        cw = CollisionWorld(
            "global_collision_world"
        )  # Needs to be 'global_collision_world' for some reason. See L181 in /opt/ros/noetic/lib/python3/dist-packages/hsrb_interface/settings.py
        for obstacle, name in zip(obstacles, obstacle_names):
            # visualize_cuboid_in_rviz(obstacle)
            widths = obstacle.widths
            cw.add_box(
                x=widths.x,
                y=widths.y,
                z=widths.z,
                pose=HsrbPose(obstacle.midpoint, quaternion()),
                name=name,
                frame_id="map",
            )
        return cw
    else:
        raise NotImplementedError()
        return CollisionWorld()


def _hsrb_pose_to_transform(pose: HsrbPose) -> Transform:
    return Transform(name="unnamed_pose", position=pose.pos, rotation=pose.ori)


class HsrbRobot:
    def __init__(self, robot: Robot) -> None:
        self._robot = robot
        self._omni_base = robot.get("omni_base")
        self._whole_body = robot.get("whole_body")

        # CollisionWorld
        # self._collision_world = get_collision_world()

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

    def get_end_effector_pose(self) -> Transform:
        # end_effector_frame = self.whole_body.end_effector_frame # hand_palm_link
        end_eff_pose = self.whole_body.get_end_effector_pose(ref_frame_id="map")
        tf = Transform(name="hsrb_end_effector", position=end_eff_pose.pos, rotation=end_eff_pose.ori)
        return tf

    # ------------------------------------------------------------------------------------------------------------------
    #                                               Actions

    def grasp_bottle(self, bottle: Bottle) -> bool:
        print("\n------------------------------------------", flush=True)
        print("HsrbRobot.grasp_bottle() - Grasping bottle", flush=True)
        self.move_to_neural()
        print(f"current end effector pose: {self.get_end_effector_pose()}")
        self.open_gripper()
        print("Setting collision_world")

        # TODO: It seems like collision world is not being accounted for. Inflate the cw more. Then .. ?
        self.whole_body.collision_world = (
            get_collision_world()
        )  # Note: collision_world needs to be set every time for some reason

        print("                         - Moving end effector to grasp pose", flush=True)

        bottle_pose = bottle.tf.pose_hsrb_format()
        grasp_position = vector3(
            x=bottle_pose.pos.x, y=bottle_pose.pos.y, z=bottle_pose.pos.z + SODA_CAN_HEIGHT / 2.0
        )  # Grasp at the mid height of the can
        grasp_rotation = _END_EFF_FLAT_ROTATION
        grasp_pose = HsrbPose(grasp_position, grasp_rotation)
        visualize_tf_in_rviz("grasp_pose_tf", _hsrb_pose_to_transform(grasp_pose))

        self.whole_body.move_end_effector_pose(grasp_pose, ref_frame_id="map")
        # gripper.apply_force(0.5, delicate=True)

    def open_gripper(self):
        """From the docs: "The hand uses slightly different commands than the other joints. The command function is used
        to control the opening of the gripper angle [rad]. Roughly 1.2[rad] is the open position and 0.0 [rad] is the
        closed position"

        See https://docs.hsr.io/hsr_develop_manual_en/python_interface/arm_python_interface.html#id4 for further details
        """
        print("HsrbRobot.open_gripper() - Opening gripper", flush=True)
        self.gripper.command(1.2)

    def close_gripper(self):
        """See open_gripper() for details"""
        print("HsrbRobot.open_gripper() - Closing gripper", flush=True)
        self.gripper.command(0.0)

    def move_to_neural(self) -> None:
        print("HsrbRobot.move_to_neutral() - Moving to neutral pose", flush=True)
        self.whole_body.move_to_neutral()

    def look_at(self, position: Vector3) -> None:
        """Look at a point. Will not self-collide"""
        self.whole_body.gaze_point(point=position, ref_frame_id="base_link")

    def move_base_to(self, pose: Tuple[float, float, float]):
        print("HsrbRobot.move_base_to() - Moving to pose:", pose, flush=True)
        print("                         - Current pose:", self.omni_base.get_pose(), flush=True)
        try:
            self.close_gripper()
            print("                         - Moving to go state", flush=True)
            self.whole_body.move_to_go()  # Ensures that the hand does not collide during the movement.
        except Exception as e:
            print("HsrbRobot.move_base_to() - failed to move_to_go():", e, flush=True)
            raise e

        print("                         - Moving with go_abs()", flush=True)
        self.omni_base.go_abs(x=pose[0], y=pose[1], yaw=pose[2], timeout=_MOVE_TIMEOUT)
        print("HsrbRobot.move_base_to() - Done", flush=True)

    def say(self, text: str) -> None:
        print(f"HsrbRobot.say(): '{text}'")
        self._speaker.say(text)
