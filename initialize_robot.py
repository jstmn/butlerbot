from src.supporting_types import Bottle, Cuboid, Transform, Sphere
from src.hsrb_robot import HsrbRobot
from src.constants import *

from hsrb_interface.geometry import vector3
import hsrb_interface
import rospy

_robot = hsrb_interface.Robot()
robot = HsrbRobot(_robot, Cuboid(xyz_min=vector3(100, 100, 100), xyz_max=vector3(101, 101, 101)))

""" Usage

# Terminal 1:
roslaunch hsrb_gazebo_launch hsrb_butler_bot_world.launch

# Terminal 2:
python3 initialize_robot.py
"""

if __name__ == "__main__":
    rospy.sleep(1.0)
    robot.initialize_manipulation(print_header=True)
    print("Done")
