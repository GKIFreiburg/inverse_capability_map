#!/bin/bash

if [ -z "$1" ]; then
	echo "Usage: ./create_isrm <polygon.poly> <resolution> <theta_resolution>"
	exit 1
fi

if [ -z "$2" ]; then
	echo "Usage: ./create_isrm <polygon.poly> <resolution> <theta_resolution>"
	exit 1
fi

if [ -z "$3" ]; then
	echo "Usage: ./create_isrm <polygon.poly> <resolution> <theta_resolution>"
	exit 1
fi

polygon=$1
resolution=$2
theta_resolution=$3

echo "Looking for IOCRM for right arm"
path=$(rospack find inverse_capability_map_generator)/maps/inv_cap_pr2_right_arm_$resolution\m_$theta_resolution\t.icpm

if [ -f $path ]; then
    echo "IOCRM for right arm exists"
else 
    echo "Need to compute IOCRM for the right arm"
	echo roslaunch inverse_capability_map_generator inverse_capability_pr2.launch group_name:=right_arm r:=$resolution t:=$theta_resolution
fi




