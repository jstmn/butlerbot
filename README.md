# ButlerBot


## Setup

First, follow HSR Development manual: https://docs.hsr.io/hsrc_user_manual_en/howto/pc_install.html#pc

Next, copy ros configs to `/opt/ros/noetic/...`:
```
cp ros_configs/hsrb_butler_bot_world.launch /opt/ros/noetic/share/hsrb_gazebo_launch/launch/

# NOTE: First update `hsrb_gazebo_common.xml` L67 with the appropriate filepath (currently its `/home/jstm/Documents/csci513/butler-bot/rviz_configs/rviz_config.rviz`)
cp ros_configs/hsrb_gazebo_common.xml /opt/ros/noetic/share/hsrb_gazebo_launch/launch/include/
cp ros_configs/butler_bot_world.world /opt/ros/noetic/share/tmc_gazebo_worlds/worlds/
```


## Development


```
# Terminal 1:
roslaunch hsrb_gazebo_launch hsrb_butler_bot_world.launch

# Terminal 2:
rostopic echo /clicked_point    # then 'Publish Point' in Rviz

# Terminal 3:
python3 main.py
```





## Reference frames

- **cans/bottles:** The reference frame of a can or bottle is attached to the bottom of the can/bottle in the center, with +z pointing up.
