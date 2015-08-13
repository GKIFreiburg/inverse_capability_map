HOW TO: Create an inverse reachability map for a surface
========================================================

COMPUTE INVERSE REACHABILITY MAPS FOR PR2-ROBOT, can be ported to other robot systems, but this needs code adaption!

1.) Create a polygon, you may use the 'primitive_to_polygon' tool to create a polygon file (only for rectangles and circle at the moment)
	Note it is also possible to create polygon file by hand, take care that the points of the polygon are centered around the origin O(0,0)
		--> adapt launch file 'primitive_to_polygon.launch'
			- choose name for surface
			- select rectangle or circle
			- enter values for parameters
		--> Execute: roslaunch primitive_to_polygon primitive_to_polygon.launch
			- a file is created in primitive_to_polygon/maps/name.poly  

2.) Now create the inverse object-centered reachability map for your robot (you can skip this skep one these have been built for your robot)
	- first create reachability map with package 'capability_map_generator'
	- adapt launch file in 'inverse_capability_map_generator/inverse_capability_pr2.launch'
	- execute: roslaunch inverse_capability_map_generator inverse_capability_pr2.launch

3.) Next create the inverse surface reachability map
	- adapt launch file in 'inverse_capability_map_generator/inverse_capability_polygon_pr2.launch'
	- first enter name of polygon (created early) without extension
	- select arm_group
	- resolution and angular resolution should be the same as for the capability map
	- leave collision_checks to true
	- execute: roslaunch inverse_capability_map_generator inverse_capability_polygon_pr2.launch

		--> repeat the same, this time for the other arm group

4.) The combination of both inverse surface reachability maps
	- adapt launch file in 'inverse_capability_map_generator/inverse_capability_combined_pr2.launch'
	- execute: roslaunch inverse_capability_map_generator inverse_capability_combined_pr2.launch

This is it, you just created an inverse reachability map.

You can check your result using the launch scipts in 'inverse_capability_map/launch'

