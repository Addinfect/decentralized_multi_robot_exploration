#!/bin/bash

# just print this out
for j in {1..3}
do
    for i in {2..6..1}
    do
        rostest tests run_percentage.test world:=office number_robots:=$i explore_algorithm:=Simple
	rostest tests run_percentage.test world:=office number_robots:=$i explore_algorithm:=Hungarian
        rostest tests run_percentage.test world:=office number_robots:=$i explore_algorithm:=Auction
    done
done

# exit gracefully by returning a status 
exit 0
