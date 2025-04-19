#!/bin/bash

source /environment.sh

# initialize the launcher
roslaunch my_package both_nodes.launch

# wait for both to finish (joins all BG processes)
dt-launchfile-join 
