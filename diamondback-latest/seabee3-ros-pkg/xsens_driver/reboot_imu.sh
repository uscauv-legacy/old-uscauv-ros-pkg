#!/bin/bash

while true
do
DEV=`ls /dev/seabee | grep imu`
DRV=`rostopic list | grep xsens`

if [ -z "$DEV"] || [ -z "$DRV"]; then
    echo "imu  not found"
    roslaunch xsens_driver xsens_driver.launch &
else
    echo "imu found"
fi

sleep 3s
    
done