# ButlerBot

Pseudo-code

``` python
move_to_table()

while True:
  bottle_poses = get_poses_of_all_bottles()
  out_of_place_bottles = get_out_of_place_subset(bottle_poses)

  if len(out_of_place_bottles) == 0:
    say("No more bottles to pickup, cleaning mission complete")	
    return

  bottle_to_grasp = choose_bottle_to_grasp(out_of_place_bottles)
  grasp_bottle(bottle_to_grasp)

  bottle_grasped = am_currently_holding_bottle()
  if bottle_grasped:
    say("successfully picked up bottle")
  else:
      raise error

  place_bottle_on_tray()
```


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
