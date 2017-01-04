#!/bin/bash

echo "Starting!"

./pub_ik_axyz.sh 3.14 100 100 300
sleep 3

./pub_grip_open.sh
sleep 1

./pub_ik_axyz.sh 3.14 100 100 100
sleep 2

./pub_ik_axyz.sh 3.14 100 100 50
sleep 1

./pub_grip_close.sh
sleep 1

./pub_ik_axyz.sh 3.14 100 100 200
sleep 3

./pub_ik_axyz.sh 3.14 -200 -200 200
sleep 8

./pub_grip_open.sh
sleep 0.5

./pub_grip_zero.sh

./pub_ik_axyz.sh 4.7 0 0 600
sleep 3

echo "Finished!"
