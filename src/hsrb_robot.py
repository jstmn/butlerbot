from typing import List, Tuple
from time import sleep

from src.math_utils import vec_addition, vec_negation, vec_scaled, vec_length, vector_distance
from src.supporting_types import Cuboid, Bottle, Transform
from src.gazebo_utils import visualize_tf_in_rviz, visualize_cuboids_in_rviz
from src.constants import *

from hsrb_interface import Robot
from hsrb_interface.mobile_base import MobileBase
from hsrb_interface.joint_group import JointGroup
from hsrb_interface.end_effector import Gripper
from hsrb_interface.collision_world import CollisionWorld
from hsrb_interface.geometry import Vector3, vector3, quaternion
from hsrb_interface.geometry import Pose as HsrbPose
from hsrb_interface.geometry import pose as hsrb_pose
from hsrb_interface.exceptions import FollowTrajectoryError, MotionPlanningError
import rospy

_MOVE_TIMEOUT = 30.0

_END_EFF_FLAT_ROTATION = quaternion(
    x=-0.5474032817174194, y=0.44796588933801107, z=-0.547040587681599, w=-0.44768605582883053
)


def _hsrb_pose_to_transform(pose: HsrbPose) -> Transform:
    return Transform(name="unnamed_pose", position=pose.pos, rotation=pose.ori)


class HsrbRobot:
    def __init__(self, robot: Robot, basket_volume: Cuboid) -> None:
        self._robot = robot
        self._omni_base = robot.get("omni_base")
        self._whole_body = robot.get("whole_body")
        self.whole_body.collision_world = CollisionWorld("global_collision_world")
        self._gripper = robot.get("gripper")
        self._speaker = robot.get("default_tts")
        self._basket_volume = basket_volume

        # Create and fill in the collision world
        self._initialize_collision_world()

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

    def _initialize_collision_world(self):
        self.whole_body.collision_world = CollisionWorld("global_collision_world")
        self.whole_body.collision_world.remove_all()

        if GAZEBO_MODE:
            table_surface_c = Cuboid(
                xyz_min=vector3(DESK_MIN_XY[0], DESK_MIN_XY[1], 0.0),
                xyz_max=vector3(DESK_MAX_XY[0], DESK_MAX_XY[1], DESK_HEIGHT),
            )
            table_surface_c.add_x_padding(0.03)
            table_surface_c.add_y_padding(0.03)
            table_surface_c.add_z_padding(0.02)

            obstacles = [table_surface_c, self._basket_volume]
            obstacle_names = ["table_surface", "basket"]
            visualize_cuboids_in_rviz("obstacle_world", obstacles)

            # Create and instantiate the collision world
            for obstacle, name in zip(obstacles, obstacle_names):
                widths = obstacle.widths
                self.whole_body.collision_world.add_box(
                    x=widths.x,
                    y=widths.y,
                    z=widths.z,
                    pose=HsrbPose(obstacle.midpoint, quaternion()),
                    name=name,
                    frame_id="map",
                )
        else:
            raise NotImplementedError()

    def initialize_manipulation(self, print_header: bool = False):
        """Note: This function is a giant hack to get the software into a 'good' state: the state where the motion
        planner actually avoids obstacles
        """
        if print_header:
            print("\n-------------------------------------", flush=True)
            print("  --- Initializing manipulation ---  \n", flush=True)

        collision_world = CollisionWorld("global_collision_world")
        curr_x, curr_y = self.omni_base.get_pose()[0].x, self.omni_base.get_pose()[0].y

        def reset(curr_x, curr_y):
            collision_world.remove_all()
            self.whole_body.move_to_neutral()
            self.omni_base.go_abs(curr_x, curr_y, 0)

        def add_box(curr_x, curr_y):
            collision_world.add_box(
                x=0.5, y=0.75, z=0.7, pose=hsrb_pose(x=curr_x + 1.0, y=curr_y, z=0.35), frame_id="map"
            )
            self.whole_body.collision_world = collision_world

        non_collision_pose = HsrbPose(
            pos=vector3(x=curr_x + 1.0, y=curr_y, z=0.8), ori=quaternion(x=0.7071068, y=0, z=0.7071068, w=0)
        )
        #
        reset(curr_x, curr_y)
        add_box(curr_x, curr_y)
        self.whole_body.move_end_effector_pose(pose=non_collision_pose, ref_frame_id="map")
        print("sleeping to allow motion to complete")

        error = vector_distance(self.get_end_effector_pose().position, non_collision_pose.pos)
        if error < 0.05:
            print(
                "Action completed successfully which indicates the robot software is in the good state - meaning the"
                " motion planner is avoiding obstacles. exiting function early"
            )
            reset(curr_x, curr_y)
            self._initialize_collision_world()
            return

        while True:
            error = vector_distance(self.get_end_effector_pose().position, non_collision_pose.pos)
            if error < 0.05:
                break
            print("end effector error: ", round(error, 4), " - sleeping", flush=True)
            sleep(0.5)

        print("Done sleeping. repeating action - this time it should avoid the box")
        reset(curr_x, curr_y)
        add_box(curr_x, curr_y)
        self.whole_body.move_end_effector_pose(pose=non_collision_pose, ref_frame_id="map")
        reset(curr_x, curr_y)
        print("Done")
        self._initialize_collision_world()

    def get_bottle_to_robot_vector(self, bottle: Bottle) -> Vector3:
        bottle_pose = bottle.tf.pose_hsrb_format()
        robot_xy = self.base_xy_pose()
        robot_position = vector3(x=robot_xy[0], y=robot_xy[1], z=DESK_HEIGHT)
        return vec_addition(robot_position, vec_negation(bottle_pose.pos))

    def _get_grasp_pose(self, bottle: Bottle) -> Tuple[HsrbPose, float]:
        """Get the pose the end effector should move to in order to grasp the bottle. Assumes there will be a scooping
        motion to collect the bottle.

        Args:
            bottle (Bottle): The bottle to grasp
        """
        bottle_pose = bottle.tf.pose_hsrb_format()
        alpha = 0.1

        # Grasp at the mid height of the can
        adjusted_bottle_position = vec_addition(bottle_pose.pos, vector3(z=SODA_CAN_HEIGHT / 2.0))
        bottle_to_robot_vec = self.get_bottle_to_robot_vector(bottle)
        adjusted_bottle_position = vec_addition(adjusted_bottle_position, vec_scaled(bottle_to_robot_vec, alpha))
        grasp_rotation = _END_EFF_FLAT_ROTATION  # TODO: Rotate quaternion about z-axis by the approach angle
        grasp_pose = HsrbPose(adjusted_bottle_position, grasp_rotation)
        return grasp_pose, vec_length(bottle_to_robot_vec) * alpha

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

    def grasp_bottle(self, bottle: Bottle, print_header: bool = False) -> bool:
        if print_header:
            print("\n---------------------------", flush=True)
            print("  --- Grasping bottle ---  \n", flush=True)

        self.move_to_neural()
        self.open_gripper()

        # Find grasp pose
        grasp_pose, dist_to_can = self._get_grasp_pose(bottle)
        visualize_tf_in_rviz("grasp_pose_tf", _hsrb_pose_to_transform(grasp_pose))

        # Move to grasp pose
        print("  moving end effector to grasp pose", flush=True)
        try:
            self.whole_body.move_end_effector_pose(grasp_pose, ref_frame_id="map")
        except FollowTrajectoryError:
            print("  failed to move to grasp pose. Aborting grasp", flush=True)
            return False

        # Move the gripper forward by 5cm to help scoop the can
        print("  doing scoop", flush=True)
        try:
            self.whole_body.move_end_effector_by_line((0, 0, 1), dist_to_can)
        except MotionPlanningError as e:
            print("  failed to do scoop. Aborting grasp", flush=True)
            print(f"  error: '{e}'")
            return False

        # Grab object
        print("  closing gripper", flush=True)
        self.gripper.apply_force(0.5, delicate=True)

        # Return to base pose
        print("  grasped bottle. moving back to base pose", flush=True)
        self.move_base_to(DESK_TARGET_POSE)
        print("done")
        return True

    def place_currently_grasped_bottle(
        self, place_location: Vector3, cleaned_bottles: List[Bottle], print_header: bool = False
    ) -> bool:
        if print_header:
            print("\n--------------------------------------------", flush=True)
            print("  --- Placing currently grasped bottle ---  \n", flush=True)

        self.move_base_to(DESK_CLEAN_TARGET_POSE)

        # Get target pose
        target_rotation = _END_EFF_FLAT_ROTATION
        target_pose = HsrbPose(place_location, target_rotation)
        visualize_tf_in_rviz("grasp_pose_tf", _hsrb_pose_to_transform(target_pose))

        # Move to target pose
        try:
            self.whole_body.move_end_effector_pose(target_pose, ref_frame_id="map")
        except FollowTrajectoryError:
            print("  Failed to move to target pose ('hsrb_interface.exceptions.FollowTrajectoryError')", flush=True)
            return False

        print("  Opening gripper", flush=True)
        self.open_gripper()
        print("  Placed bottle. moving back to base pose", flush=True)
        self.move_base_to(DESK_CLEAN_TARGET_POSE)
        return True

    def open_gripper(self):
        """From the docs: "The hand uses slightly different commands than the other joints. The command function is used
        to control the opening of the gripper angle [rad]. Roughly 1.2[rad] is the open position and 0.0 [rad] is the
        closed position"

        See https://docs.hsr.io/hsr_develop_manual_en/python_interface/arm_python_interface.html#id4 for further details
        """
        self.gripper.command(1.2)

    def close_gripper(self):
        """See open_gripper() for details"""
        self.gripper.command(0.0)

    def move_to_neural(self) -> None:
        self.whole_body.move_to_neutral()

    def look_at(self, position: Vector3) -> None:
        """Look at a point. Will not self-collide"""
        self.whole_body.gaze_point(point=position, ref_frame_id="base_link")

    def move_base_to(self, pose: Tuple[float, float, float], print_header: bool = False) -> None:
        if print_header:
            print("\n--------------------------", flush=True)
            print("  --- Moving to pose ---  \n", flush=True)

        try:
            print("  moving to go state", flush=True)
            self.whole_body.move_to_go()  # Ensures that the hand does not collide during the movement.
        except Exception as e:
            print("move_base_to() - failed to move_to_go():", e, flush=True)
            raise e

        print("  moving with go_abs()", flush=True)
        self.omni_base.go_abs(x=pose[0], y=pose[1], yaw=pose[2], timeout=_MOVE_TIMEOUT)
        print("  ^-- done", flush=True)

    def say(self, text: str) -> None:
        print(f"  robot: '{text}'")
        self._speaker.say(text)


"""
def print_diffs(old, new):
            names = self.whole_body.joint_state.name
            for i in range(len(old)):
                if abs(old[i] - new[i]) > 0.003:
                    print(f"{names[i]}: {round(old[i], 4)} -> {round(new[i],4)}")

        print("  moving to 0deg")
        curr_x, curr_y = self.base_xy_pose()
        self.move_base_to((curr_x, curr_y, 0))

        # Set the collision world
        print("  moving to neutral")
        self.whole_body.move_to_neutral()
        print("  moving to target pose")
        # Needs to be 'global_collision_world' for some reason. See L181 in /opt/ros/noetic/lib/python3/dist-packages/hsrb_interface/settings.py
        self.whole_body.collision_world = CollisionWorld("global_collision_world")
        self.whole_body.collision_world.add_box(
            x=1.0, y=0.75, z=0.7, pose=hsrb_pose(x=curr_x + 1, y=curr_y, z=0.35), frame_id="map"
        )
        target_pose = HsrbPose(
            pos=vector3(x=curr_x + 1.0, y=curr_y, z=0.5), ori=quaternion(x=0.7071068, y=0, z=0.7071068, w=0)
        )
        self.whole_body.move_end_effector_pose(pose=target_pose, ref_frame_id="map")
        print("  moving back to neutral")
        self.whole_body.move_to_neutral()

        self.whole_body.collision_world.remove_all()
        self.update_collision_world(self.whole_body.collision_world)

        current_posv = np.array(self.whole_body.joint_state.position)
        print(f"  testing  'arm_lift_joint' movement")
        self.whole_body.move_to_joint_positions({"arm_lift_joint": 0.2})
        rospy.sleep(3.0)

        print("  joint differences:")
        new_posv = np.array(self.whole_body.joint_state.position)
        print_diffs(current_posv, new_posv)
        self.whole_body.move_to_neutral()
        print("done")
"""
