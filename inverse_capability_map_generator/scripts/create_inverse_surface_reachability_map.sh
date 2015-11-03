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

echo "Looking for IOCRM for both arms"
roscd inverse_capability_map_generator/maps
ll
if [ $? != 0 ]; then
	echo "ERROR IN MOVE_GROUP"
	exit 1
fi




