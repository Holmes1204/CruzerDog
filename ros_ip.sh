#!/bin/sh
export ROS_IP=$1
export ROS_HOSTNAME=$1
export ROS_MASTER_URI=http://$2:11311
echo $ROS_IP
echo $ROS_HOSTNAME
echo $ROS_MASTER_URI