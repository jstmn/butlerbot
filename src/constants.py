import numpy as np

PI = np.pi
GAZEBO_MODE = True

SODA_CAN_WIDTH = 0.0662  # m
SODA_CAN_HEIGHT = 0.1246  # m

if GAZEBO_MODE:
    DESK_HEIGHT = 0.6973
    CLEAN_AREA_MIN_XY = (2.2714, -0.6963)
    CLEAN_AREA_MAX_XY = (2.5979, -0.3967)
    DESK_TARGET_POSE = (2.75, 0.25, 3 / 2 * PI)  # (x, y, yaw)

    # See /opt/ros/noetic/share/hsrb_gazebo_launch/launch/hsrb_butler_bot_world.launch
    GAZEBO_TO_RVIZ_XY_OFFSET = (-5.0, -6.6)