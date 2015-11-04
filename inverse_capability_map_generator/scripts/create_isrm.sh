#!/bin/bash

##### Verify input #####
if [ -z "$1" ]; then
	echo "Usage: ./create_isrm <polygon_name> <resolution> <theta_resolution>"
	exit 1
else
	path=$(rospack find primitive_to_polygon)/polygons/$1\.poly
	if [ ! -f $path ]; then
		echo "Error: Could not find: $path"
		exit 1
	fi		 
fi

if [ -z "$2" ]; then
	echo "Usage: ./create_isrm <polygon_name> <resolution> <theta_resolution>"
	exit 1
else
	if ! [[ "$2" =~ ^[-+]?[0-9]+\.?[0-9]*$ ]] ;
		then exec >&2; echo "Error: Parameter 2 is not a number"; exit 1
	fi
fi

if [ -z "$3" ]; then
	echo "Usage: ./create_isrm <polygon_name> <resolution> <theta_resolution>"
	exit 1
else
	if ! [[ "$3" =~ ^[0-9]+$ ]] ;
		then exec >&2; echo "Error: Parameter 3 is not a number"; exit 1
	fi
fi

polygon=$1
resolution=$2
theta_resolution=$3

##### IOCRM #####
echo "Looking for/Computing IOCRM for both arms"
declare -a arms=("right_arm" "left_arm")

for arm_group in ${arms[@]}
do
	path=$(rospack find inverse_capability_map_generator)/maps/inv_cap_pr2_$arm_group\_$resolution\m_$theta_resolution\t.icpm

	if [ -f $path ]; then
    	echo "IOCRM for $arm_group exists"
	else 
    	echo "Need to compute IOCRM for $arm_group"
		roslaunch inverse_capability_map_generator inverse_capability_pr2.launch group_name:=$arm_group r:=$resolution t:=$theta_resolution
	fi
done

##### ISRM #####
echo "Looking for/Computing ISRM for both arms"

for arm_group in ${arms[@]}
do
	path=$(rospack find inverse_capability_map_generator)/maps/inv_cap_$polygon\_pr2_$arm_group\_$resolution\m_$theta_resolution\t.icpm

	if [ -f $path ]; then
		echo "ISRM for $arm_group for $polygon exists"
	else
		echo "Need to compute ISRM for $arm_group for $polygon"
		roslaunch inverse_capability_map_generator inverse_capability_polygon_pr2.launch poly_name:=$polygon group_name:=$arm_group r:=$resolution t:=$theta_resolution
	fi
done

##### CISRM #####
echo "Computing Combined ISRM"
	path=$(rospack find inverse_capability_map_generator)/maps/inv_cap_$polygon\_pr2_combined_$resolution\m_$theta_resolution\t.icpm

echo $path

if [ -f $path ]; then
	echo "Combined ISRM for $polygon exists"
else
	echo "Need to compute combined ISRM for $polygon"
	roslaunch inverse_capability_map_generator inverse_capability_combined_pr2.launch poly_name:=$polygon r:=$resolution t:=$theta_resolution
fi

##### DONE #####
echo ""
path=$(rospack find inverse_capability_map_generator)/maps/
echo "All computed maps can be found in: $path"

