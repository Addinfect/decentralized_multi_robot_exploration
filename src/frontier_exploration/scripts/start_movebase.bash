#!/bin/bash

# just print this out
echo "Start add trajectories"

for ((c=0; c<$1; c++))
do
    echo "Start trajectory for Robot_"$c
    ROBOT_NAME=robot_$c roslaunch frontier_exploration add_trajectory.launch
    roslaunch frontier_exploration navigation.launch robot_name:="robot_$c" &>/dev/null &
done


#roslaunch frontier_exploration multi_robot_navigation.launch

# exit gracefully by returning a status 
exit 0