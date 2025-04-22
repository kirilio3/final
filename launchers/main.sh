#!/bin/bash

source /environment.sh

# initialize the launcher
roslaunch my_package both_nodes.launch tagNumber:=4

# rosrun my_package camara_node.py
# wait for both to finish (joins all BG processes)
dt-launchfile-join 
