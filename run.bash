#!/bin/bash

( cd ~/ros2_ws/ && colcon build --packages-select value_iteration2)

source ~/.bashrc
ros2 launch value_iteration2 turtle.launch.py
