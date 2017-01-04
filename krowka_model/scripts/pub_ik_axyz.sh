#!/bin/bash

rostopic pub --once /krowka/inverse_kinematics/a std_msgs/Float64 "data: $1" &
rostopic pub --once /krowka/inverse_kinematics/x std_msgs/Float64 "data: $2" &
rostopic pub --once /krowka/inverse_kinematics/y std_msgs/Float64 "data: $3" &
rostopic pub --once /krowka/inverse_kinematics/z std_msgs/Float64 "data: $4" &
