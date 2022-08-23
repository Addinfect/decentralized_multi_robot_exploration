#!/bin/bash

# just print this out
echo "Start add trajectories"

ROBOT_NAME=robot_0 roslaunch frontier_exploration add_trajectory.launch

ROBOT_NAME=robot_1 roslaunch frontier_exploration add_trajectory.launch

ROBOT_NAME=robot_2 roslaunch frontier_exploration add_trajectory.launch

roslaunch frontier_exploration multi_robot_navigation.launch

# exit gracefully by returning a status 
exit 0