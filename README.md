# ButlerBot

The code in this repo controls the Human Support Robot (HSR) to execute the task of picking up all visible soda cans off of a table and placing them in a trash can. To simplify the problem, several assumptions were made:
- The cans would be located on top of a single desk
- The cans would all be upright
- The size and location of the desk would be known ahead of time
- The size and location of the trash can would be known ahead of time

This project was done as the final project of [CSCI 513: Cyber Physical Systems](https://jdeshmukh.github.io/teaching/cs513-autocps-fall-2022/index.html).

Demo:
![alt text](https://github.com/jstmn/butlerbot/blob/master/media/fsm.png?raw=true)


This code is an implementation of the following finite state machine:
![alt text](https://github.com/jstmn/butlerbot/blob/master/media/ButlerBot-Demo-2.gif?raw=true)



## Setup

First, follow HSR Development manual: https://docs.hsr.io/hsrc_user_manual_en/howto/pc_install.html#pc

Next, copy ros configs to `/opt/ros/noetic/...`:
```

# NOTE: First update `hsrb_gazebo_common.xml` L67 with the appropriate filepath (currently its `/home/jstm/Documents/csci513/butler-bot/rviz_configs/rviz_config.rviz`)
cp ros_configs/hsrb_gazebo_common.xml /opt/ros/noetic/share/hsrb_gazebo_launch/launch/include/

sudo cp ros_configs/butler_bot_world.world /opt/ros/noetic/share/tmc_gazebo_worlds/worlds/
sudo cp ros_configs/butler_bot_armarker_world.world /opt/ros/noetic/share/tmc_gazebo_worlds/worlds/

sudo cp ros_configs/hsrb_butler_bot_world.launch /opt/ros/noetic/share/hsrb_gazebo_launch/launch/
sudo cp ros_configs/hsrb_butler_bot_armarker_world.launch /opt/ros/noetic/share/hsrb_gazebo_launch/launch/

sudo cp -r gazebo_models/*/  /opt/ros/noetic/share/tmc_gazebo_worlds/models/
```


## Development


```
# Terminal 1:
roslaunch hsrb_gazebo_launch hsrb_butler_bot_world.launch
roslaunch hsrb_gazebo_launch hsrb_butler_bot_armarker_world.launch

# Terminal 2:
rostopic echo /clicked_point    # then 'Publish Point' in Rviz

# Terminal 3:
python3 main.py
```




## Reference frames

- **cans/bottles:** The reference frame of a can or bottle is attached to the bottom of the can/bottle in the center, with +z pointing up.
