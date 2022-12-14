from typing import List, Tuple
from time import time, sleep

from src.math_utils import *
from src.supporting_types import Cuboid, Bottle, Transform
from src.gazebo_utils import visualize_tf_in_rviz, visualize_cuboids_in_rviz
from src.constants import *

from hsrb_interface import Robot
from hsrb_interface.mobile_base import MobileBase
from hsrb_interface.joint_group import JointGroup
from hsrb_interface.end_effector import Gripper
from hsrb_interface.collision_world import CollisionWorld
from hsrb_interface.geometry import Pose as HsrbPose
from hsrb_interface.geometry import pose as hsrb_pose
from hsrb_interface.exceptions import FollowTrajectoryError, MotionPlanningError

_MOVE_TIMEOUT = 30.0
APPROACH_ALPHA = 0.07
_THETA_OFFSET = np.deg2rad(-45)
OMNI_BASE_GRASP_OFFSET = 0.2


def R_x(theta: float) -> np.ndarray:
    return np.array(
        [
            [1.0, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)],
        ]
    )


def R_z(theta: float) -> np.ndarray:
    return np.array(
        [
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1],
        ]
    )


def _hsrb_pose_to_transform(pose: HsrbPose) -> Transform:
    return Transform(name="unnamed_pose", position=pose.pos, rotation=pose.ori)


def _vec3_str(vec3: Vector3):
    return "<{}, {}, {}>".format(round(vec3.x, 3), round(vec3.y, 3), round(vec3.z, 3))


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
            table_surface_c.add_z_padding(0.01)

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

        # Remove all obstacles from the CollisionWorld and move the robot to its starting position in the neutral pose
        def reset(curr_x, curr_y):
            collision_world.remove_all()
            self.whole_body.move_to_neutral()
            self.omni_base.go_abs(curr_x, curr_y, 0)

        # Add a box to the CollisionWorld and provide it to the motion planner
        def add_box(curr_x, curr_y):
            collision_world.add_box(
                x=0.5, y=0.75, z=0.7, pose=hsrb_pose(x=curr_x + 1.0, y=curr_y, z=0.35), frame_id="map"
            )
            self.whole_body.collision_world = collision_world

        non_collision_pose = HsrbPose(
            pos=vector3(x=curr_x + 1.0, y=curr_y, z=0.8), ori=quaternion(x=0.7071068, y=0, z=0.7071068, w=0)
        )
        reset(curr_x, curr_y)
        add_box(curr_x, curr_y)
        # Expected behavior: the end effector enters the box ( its target is above the box). It then waits there for 70
        # seconds, before slowly moving to the actual target. After the robot completes its motion, the system should be
        # in a good state
        self.whole_body.move_end_effector_pose(pose=non_collision_pose, ref_frame_id="map")
        print("sleeping to allow motion to complete")
        t0 = time()

        while True:
            error = vector_distance(self.get_end_effector_pose().position, non_collision_pose.pos)
            if error < 0.05:
                print(
                    "Action completed successfully which indicates the robot software is in the good state - the motion"
                    " planner should avoid obstacles"
                )
                reset(curr_x, curr_y)
                self._initialize_collision_world()
                return
            print("end effector error: ", round(error, 4), f" - sleeping ({round(time()-t0, 2)}s elapsed)", flush=True)
            sleep(1.0)

    def _get_grasp_rotation(self, bottle: Bottle) -> Quaternion:
        visualize_tf_in_rviz("robot_pose_tf", _hsrb_pose_to_transform(self.omni_base.get_pose()))

        # _THETA_OFFSET = np.deg2rad(-45)

        # xy_bottle = bottle.xy_coords()
        # xy_robot = self.base_xy_pose()
        # theta = np.arctan2(xy_bottle[1] - xy_robot[1], xy_bottle[0] - xy_robot[0]) - (3*np.pi/2)
        theta = _THETA_OFFSET

        R = np.eye(3)
        R = np.matmul(R, R_x(np.deg2rad(90)))
        R = np.matmul(R, R_z(np.deg2rad(90)))
        R = np.matmul(R, R_x(theta))
        # R = np.matmul(R, R_x(-theta + _THETA_OFFSET))
        print(f"  theta: {theta} / {np.rad2deg(theta)}")
        # Sanity check the rotation
        np.testing.assert_almost_equal(np.linalg.inv(R), np.transpose(R))
        return quaternion_from_matrix(R)

    def get_bottle_to_robot_vector(self, bottle: Bottle) -> Vector3:
        robot_xy = self.base_xy_pose()
        robot_position = vector3(x=robot_xy[0], y=robot_xy[1], z=DESK_HEIGHT)
        return vec_addition(robot_position, vec_negation(bottle.tf.position))

    def _get_grasp_pose(self, bottle: Bottle) -> Tuple[HsrbPose, float]:
        """Get the pose the end effector should move to in order to grasp the bottle. Assumes there will be a scooping
        motion to collect the bottle.

        Args:
            bottle (Bottle): The bottle to grasp
        """
        if GET_BOTTLE_POSES_FROM_GAZEBO:
            grasp_height_offset = (SODA_CAN_HEIGHT / 2.0) + 0.02
        else:
            grasp_height_offset = 0.01

        # Grasp at the mid height of the can
        bottle_to_robot_vec = self.get_bottle_to_robot_vector(bottle)
        scaled_bottle_to_robot_vec = vec_scaled(bottle_to_robot_vec, APPROACH_ALPHA)
        grasp_position = vec_addition(bottle.tf.position, scaled_bottle_to_robot_vec)
        # print(f"              robot:            {self.base_xy_pose()}")
        # print(f"bottle_to_robot_vec:            {_vec3_str(bottle_to_robot_vec)}")
        # print(f"scaled bottle_to_robot_vec:     {_vec3_str(scaled_bottle_to_robot_vec)}")
        # print(f"    bottle_position:            {_vec3_str(bottle.tf.position)}")
        # print(f"grasp_position:                 {_vec3_str(grasp_position)}")
        # Add +z offset
        height_adjusted_grasp_position = vec_addition(grasp_position, vector3(z=grasp_height_offset))
        # print(f"height_adjusted_grasp_position: {_vec3_str(height_adjusted_grasp_position)}")
        grasp_rotation = self._get_grasp_rotation(bottle)
        grasp_pose = HsrbPose(height_adjusted_grasp_position, grasp_rotation)
        return grasp_pose, vec_length(bottle_to_robot_vec) * APPROACH_ALPHA

    def base_xy_pose(self) -> Tuple[float, float]:
        base_pose, _ = self.omni_base.get_pose()
        return base_pose.x, base_pose.y

    def get_end_effector_pose(self) -> Transform:
        # end effector link is 'hand_palm_link'
        end_eff_pose = self.whole_body.get_end_effector_pose(ref_frame_id="map")
        return Transform(name="hsrb_end_effector", position=end_eff_pose.pos, rotation=end_eff_pose.ori)

    # ------------------------------------------------------------------------------------------------------------------
    #                                               Actions

    def grasp_bottle(self, bottle: Bottle, print_header: bool = False) -> bool:
        if print_header:
            print("\n---------------------------", flush=True)
            print("  --- Grasping bottle ---  \n", flush=True)

        base_pose = (bottle.xy_coords()[0] + OMNI_BASE_GRASP_OFFSET, DESK_TARGET_POSE[1], DESK_TARGET_POSE[2])
        self.move_base_to(base_pose)
        self.move_to_neural()
        self.open_gripper()

        # TODO: Update rotate grasp pose along +z axis by the approach angle
        # Find grasp pose
        grasp_pose, dist_to_can = self._get_grasp_pose(bottle)
        visualize_tf_in_rviz("grasp_pose_tf", _hsrb_pose_to_transform(grasp_pose))

        # sleep(3.0)
        # exit()

        # Move to grasp pose
        print("  moving end effector to grasp pose", flush=True)
        try:
            self.whole_body.move_end_effector_pose(grasp_pose, ref_frame_id="map")
            sleep(1.0)
        except FollowTrajectoryError:
            print("  failed to move to grasp pose. Aborting grasp", flush=True)
            return False

        # Grab object
        print("  closing gripper", flush=True)
        self.gripper.apply_force(0.5, delicate=True)  # blocking

        # Return to base pose
        print("  grasped bottle. moving back to base pose", flush=True)
        print("done")
        return True

    def place_currently_grasped_bottle(
        self, place_location: Vector3, cleaned_bottles: List[Bottle], print_header: bool = False
    ) -> bool:
        if print_header:
            print("\n--------------------------------------------", flush=True)
            print("  --- Placing currently grasped bottle ---  \n", flush=True)

        # Get target pose
        R = np.eye(3)
        R = np.matmul(R, R_x(np.deg2rad(90)))
        R = np.matmul(R, R_z(np.deg2rad(90)))
        target_rotation = quaternion_from_matrix(R)
        target_pose = HsrbPose(place_location, target_rotation)
        visualize_tf_in_rviz("grasp_pose_tf", _hsrb_pose_to_transform(target_pose))

        # Move to target pose
        try:
            self.whole_body.move_end_effector_pose(target_pose, ref_frame_id="map")
        except FollowTrajectoryError:
            print(f"  Failed to move to target pose: ('{e}')", flush=True)
            return False
        except MotionPlanningError as e:
            if "START_STATE_IN_COLLISION" in str(e):
                print(
                    "  Failed to move to target pose ('hsrb_interface.exceptions.MotionPlanningError:"
                    " START_STATE_IN_COLLISION')",
                    flush=True,
                )
                print("  going to try moving backwards and trying again")
                self.omni_base.go_rel(x=-0.25)
                self.whole_body.move_end_effector_pose(target_pose, ref_frame_id="map")
                return False
            print(f"  Failed to move to target pose with unhandled motion planning error ('{e}')", flush=True)
            return False

        print("  Opening gripper", flush=True)
        self.open_gripper()

        print("  moving backwards by x=-0.25", flush=True)
        self.omni_base.go_rel(x=-0.25)

        print("done", flush=True)
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

    def move_to_go(self) -> None:
        self.whole_body.move_to_go()

    def look_at(self, position: Vector3) -> None:
        """Look at a point. Will not self-collide"""
        self.whole_body.gaze_point(point=position, ref_frame_id="base_link")

    def move_base_to(self, pose: Tuple[float, float, float], print_header: bool = False) -> None:
        if print_header:
            print("\n--------------------------", flush=True)
            print("  --- Moving to pose ---  \n", flush=True)

        if np.linalg.norm(np.array(self.base_xy_pose()) - np.array(pose[:2])) < 0.05:
            print("  already close to target pose. skipping move", flush=True)
            return

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
        print(f"robot: '{text}'", flush=True)
        self._speaker.say(text)
