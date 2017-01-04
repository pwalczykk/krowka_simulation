#!/bin/bash

rostopic pub --once /krowka/grip_0_position_controller/command std_msgs/Float64 "data: -0.1" &
rostopic pub --once /krowka/grip_1_position_controller/command std_msgs/Float64 "data: -0.1" &
