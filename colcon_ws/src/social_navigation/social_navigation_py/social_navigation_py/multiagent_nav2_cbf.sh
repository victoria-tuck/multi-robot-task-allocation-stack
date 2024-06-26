#!/bin/bash
# Connect aliases
shopt -s expand_aliases
source ~/.bash_aliases
# Create list of robot names
for ((i = 1; i <= $1; i++))
do
  robot_list="["
  for ((j = 1; j <= $1; j++))
  do
    if [[ $i -eq $j ]]; then
      robot_name="robot${i}"
    else
      robot_list=${robot_list}\'robot${j}\',
    fi
  done
  robot_list=${robot_list::-1}\]
  echo $robot_list
  # Call robot controller for each robot
  ros2 run social_navigation_py nav2_cbf --ros-args -p use_sim_time:=True -r robot_controller:__node:=robot_controller_${i} -r basic_navigator:__node:=basic_navigator -p "robot_name:='${robot_name}'" -p "robot_list:=${robot_list}" &
done