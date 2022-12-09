


- trash_can from https://github.com/osrf/gazebo_models. This is a huge trash can, not appropriate
sudo cp -r gazebo_models/first_2015_trash_can /opt/ros/noetic/share/tmc_gazebo_worlds/models/


https://data.nvision2.eecs.yorku.ca/3DGEMS/
- recycle_bin_green: too large
- recycle_bin_red: too large
- trash_bin: this works well. (But it was originally two separate cans that weren't stuck to one another?!)
- yellow_bin: too large

sudo cp -r */  /opt/ros/noetic/share/tmc_gazebo_worlds/models/