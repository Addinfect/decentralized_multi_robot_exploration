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


for i in {1..2}
do
    n_robots=$(((1+$RANDOM % 5)+1))
    n_robots=6
    assigner=$((1+$RANDOM % 3))
    assigner=3
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
    
    rostest tests run_percentage.test world:=big_office number_robots:=$n_robots explore_algorithm:=$algorithm
done
# exit gracefully by returning a status 
exit 0
