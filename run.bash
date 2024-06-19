#!/bin/bash

( cd ~/ros2_ws/ && colcon build --packages-select value_iteration2)

source ~/.bashrc
ros2 run value_iteration2 vi_node --ros-args --params-file $(dirname $0)/config/params.yaml
