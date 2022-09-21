#!/bin/bash

#stop script wioth ctrl + c
trap "exit" INT

# just print this out

#for j in {1..1}
#do
#    for i in {2..6..1}
#    do
#        rostest tests run_percentage.test world:=big_office number_robots:=$i explore_algorithm:=Stupid
# 	    rostest tests run_percentage.test world:=big_office number_robots:=$i explore_algorithm:=Hungarian
#        rostest tests run_percentage.test world:=big_office number_robots:=$i explore_algorithm:=Auction
#    done
#done


for i in {1..20}
do
    n_robots=$(((1+$RANDOM % 3)+3)) #between 2 and 6
    #n_robots=2
    assigner=$((1+$RANDOM % 3))
    assigner=2
    echo $n_robots

    if [[ $assigner -eq 1 ]]
    then
        algorithm="Stupid"
    elif [[ $assigner -eq 2 ]]
    then
        algorithm="Hungarian"
    elif [[ $assigner -eq 3 ]]
    then
        algorithm="Auction"
    fi
    echo $algorithm
    
    rostest tests run_percentage.test world:=hospital number_robots:=$n_robots explore_algorithm:=$algorithm
done

cp ~/.ros/1_result.csv ~/decentralized_multi_robot_exploration/.
# exit gracefully by returning a status 
exit 0
