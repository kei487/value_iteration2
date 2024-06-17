#!/bin/bash

( cd ~/ros2_ws/ && colcon build --packages-select value_iteration2)

source ~/.bashrc
