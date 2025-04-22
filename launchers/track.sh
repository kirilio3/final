#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch publisher
rosrun my_package test.py
# roslaunch my_package lane_test.launch

# wait for app to end
dt-launchfile-join