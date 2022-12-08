import numpy as np

PI = np.pi
GAZEBO_MODE = True

SODA_CAN_WIDTH = 0.0662  # m
SODA_CAN_HEIGHT = 0.1246  # m

if GAZEBO_MODE:
    DESK_HEIGHT = 0.6973
    DESK_MIN_XY = (2.2197, -0.902)
    DESK_MAX_XY = (3.4167, -0.3168)

    CLEAN_AREA_MIN_XY = (2.2714, -0.6963)
    CLEAN_AREA_MAX_XY = (2.5979, -0.3967)
    DESK_TARGET_POSE = (2.75, 0.25, 3 / 2 * PI)  # (x, y, yaw)

    # See /opt/ros/noetic/share/hsrb_gazebo_launch/launch/hsrb_butler_bot_world.launch
    GAZEBO_TO_RVIZ_XY_OFFSET = (-5.0, -6.6)

    # Gazebo defines the reference frame of the soda cans in a weird place - these offsets fix the tf the specified format
    GAZEBO_TO_RVIZ_CAN_OFFSET = (0.0, 0.0, -0.05)
    # GAZEBO_TO_RVIZ_CAN_OFFSET = (0.01, -0.035, -0.055) # it seems like the pose changed. These used to be good. I probably calibrated when the robot was mislocalized
