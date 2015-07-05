#include "inverse_capability_map/InverseCapabilityOcTree.h"
#include "inverse_capability_map_utils/visualization_utils.h"
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <tclap/CmdLine.h>
#include <string>
#include <vector>

using namespace inverse_capability_map_utils;

int main(int argc, char** argv )
{
    ros::init(argc, argv, "inverse_capability_visualization");

    // arguments
    TCLAP::CmdLine cmd("Visualizes the inverse capability map given by argument", ' ', "1.0");

    TCLAP::ValueArg<std::string> pathNameArg("p", "path", "Path and filename of the inverse capability map to be visualized.\n\
                                             Example: -p mydir/mysubdir/filename.icpm", true, "./inverse_capability_map.icpm", "string");

    std::string msg;
    msg = "Specifies the region in x-direction to be visualized.\n\
           If no x-value is given, depending on y- and z-values, all stored capabilities in x-direction are displayed.\n\
           If only one x-value is given, a slice (or a point) at this position depending on y- and z-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(x1, x2, ...) to max(x1, x2, ...).\n\
           Example: -x -0.1 -x 2.3";
    TCLAP::MultiArg<double> xArg("x", "x-pos", msg, false, "floating point");

    msg = "Specifies the region in y-direction to be visualized.\n\
           If no y-value is given, depending on x- and z-values, all stored capabilities in y-direction are displayed.\n\
           If only one y-value is given, a slice (or a point) at this position depending on x- and z-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(y1, y2, ...) to max(y1, y2, ...).\n\
           Example: -y -0.1 -y 2.3";
    TCLAP::MultiArg<double> yArg("y", "y-pos", msg, false, "floating point");

    msg = "Specifies the region in z-direction to be visualized.\n\
           If no z-value is given, depending on x- and y-values, all stored capabilities in z-direction are displayed.\n\
           If only one z-value is given, a slice (or a point) at this position depending on x- and y-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(z1, z2, ...) to max(z1, z2, ...).\n\
           Example: -z -0.1 -z 2.3";
    TCLAP::MultiArg<double> zArg("z", "z-pos", msg, false, "floating point");

    msg = "The width of the table in m.\n"
           "Example: -w 1.0";
    TCLAP::ValueArg<double> widthArg("w", "width", msg, true, 1.0, "floating point");

    msg = "The length of the table in m.\n"
           "Example: -l 1.0";
    TCLAP::ValueArg<double> heightArg("l", "length", msg, true, 1.0, "floating point");

    msg = "If set, shows a color table from blue = best to red = worst";
    TCLAP::SwitchArg colorArg("c", "colortable", msg, false);

    msg = "If set, drawing 3d arrows, else 2d arrows are drawn";
    TCLAP::SwitchArg dimensionArg("d", "3darrowdimension", msg, false);

    cmd.add(zArg);
    cmd.add(yArg);
    cmd.add(xArg);
    cmd.add(widthArg);
    cmd.add(heightArg);
    cmd.add(colorArg);
    cmd.add(dimensionArg);
    cmd.add(pathNameArg);

    // parse arguments with TCLAP
    try
    {
        cmd.parse(argc, argv);
    }
    catch (TCLAP::ArgException &e)  // catch any exceptions
    {
        ROS_ERROR("Error: %s for argument %s", e.error().c_str(), e.argId().c_str());
        ros::shutdown();
        exit(1);
    }

    bool showColorTable = colorArg.getValue();
    bool draw3D = dimensionArg.getValue();

    std::string pathName = pathNameArg.getValue();
    InverseCapabilityOcTree* tree = InverseCapabilityOcTree::readFile(pathName);

    if (tree == NULL)
    {
        ROS_ERROR("Error: Inverse capability map could not be loaded.\n");
        ros::shutdown();
        exit(1);
    }

    std::string frame = tree->getBaseName();
    unsigned int theta_resolution = tree->getThetaResolution();

    ROS_INFO("Group name is: %s", tree->getGroupName().c_str());
    ROS_INFO("Base frame is: %s", frame.c_str());
    ROS_INFO("Tip frame is: %s", tree->getTipName().c_str());
    ROS_INFO("Resolution is: %g", tree->getResolution());
    ROS_INFO("Theta resolution is: %d\n", theta_resolution);

    // get the table width and height
    double width = widthArg.getValue();
    double height = heightArg.getValue();

    // get x, y and z values and sort them
    std::vector<double> xValues = xArg.getValue();
    std::vector<double> yValues = yArg.getValue();
    std::vector<double> zValues = zArg.getValue();

    std::sort(xValues.begin(), xValues.end());
    std::sort(yValues.begin(), yValues.end());
    std::sort(zValues.begin(), zValues.end());

    bool xIsSet = xValues.size() > 0 ? true : false;
    bool yIsSet = yValues.size() > 0 ? true : false;
    bool zIsSet = zValues.size() > 0 ? true : false;

    double startX = 0.0, endX = 0.0, startY = 0.0, endY = 0.0, startZ = 0.0, endZ = 0.0;

    // get and adjust the boundaries for iteration (add a small value to end due to floating point precision)
    if (xIsSet)
    {
        startX = tree->getAlignment(xValues[0]);
        endX = tree->getAlignment(xValues[xValues.size() - 1]) + tree->getResolution()/100.0;
    }
    if (yIsSet)
    {
        startY = tree->getAlignment(yValues[0]);
        endY = tree->getAlignment(yValues[yValues.size() - 1]) + tree->getResolution()/100.0;
    }
    if (zIsSet)
    {
        startZ = tree->getAlignment(zValues[0]);
        endZ = tree->getAlignment(zValues[zValues.size() - 1]) + tree->getResolution()/100.0;
    }

    ROS_INFO("Bounding Box\n"
    		"x values: [%lf, %lf]\n"
    		"y values: [%lf, %lf]\n"
    		"z values: [%lf, %lf]",
    		startX, endX, startY, endY, startZ, endZ );

    ros::NodeHandle n;
    ros::Rate r(1.0);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("inverse_capability_marker_array", 1, true);

    // remember the greatest extent in y-direction to properly set the color table outside of the capabilities
    double maxY = 0.0;

	unsigned int count = 0;
	visualization_msgs::MarkerArray markerArray;

	visualization_msgs::Marker tableMarker;
	tableMarker.header.frame_id = "table";
	tableMarker.header.stamp = ros::Time(0);
	tableMarker.action = visualization_msgs::Marker::ADD;
	tableMarker.lifetime = ros::Duration();
	tableMarker.ns = "table";
	tableMarker.id = 0;
	tableMarker.type = visualization_msgs::Marker::CUBE;
	tableMarker.pose.position.x = 0;
	tableMarker.pose.position.y = 0;
	tableMarker.pose.position.z = 0;
	tableMarker.pose.orientation.x = 0.0;
	tableMarker.pose.orientation.y = 0.0;
	tableMarker.pose.orientation.z = 0.0;
	tableMarker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	tableMarker.scale.x = width;
	tableMarker.scale.y = height;
	tableMarker.scale.z = 0.02;

	tableMarker.color.r = 0.67;
	tableMarker.color.g = 0.33;
	tableMarker.color.b = 0.0;
	tableMarker.color.a = 1.0;

	markerArray.markers.push_back(tableMarker);

	double min_percent, max_percent, minimum_percent;
	min_percent = HUGE_VAL;
	max_percent = 0.0;

	ros::NodeHandle nhPriv("~");
	nhPriv.param("minimum_percent", minimum_percent, 0.0);

	// loop through all inverse capabilities
	for (InverseCapabilityOcTree::leaf_iterator it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
	{
		// if not inside boundaries, skip actual capability
		double eps = 0.000001;
		if (xIsSet && (it.getX() + eps < startX || it.getX() - eps > endX))
		{
			continue;
		}
		if (yIsSet && (it.getY() + eps < startY || it.getY() - eps > endY))
		{
			continue;
		}
		if (zIsSet && (it.getZ() + eps < startZ || it.getZ() - eps > endZ))
		{
			continue;
		}

		visualization_msgs::Marker marker;

		marker.header.frame_id = "table";
		marker.header.stamp = ros::Time(0);

		marker.ns = tree->getGroupName();

		marker.action = visualization_msgs::Marker::ADD;
		marker.lifetime = ros::Duration();

		InverseCapability cap = it->getValue();

		// skip empty capabilities
		if (cap.getThetasPercent().size() == 0)
			continue;

		// start and end points of arrow
		geometry_msgs::Point start, end;

		double size = it.getSize();
		double angle;


		std::map<double, double>::const_iterator mit;
		const std::map<double, double>& alias = cap.getThetasPercent();

		for (mit = alias.begin(); mit != alias.end(); mit++)
		{
			// store min and max percent
			if (mit->second < min_percent)
				min_percent = mit->second;
			if (mit->second > max_percent)
				max_percent = mit->second;

			// All value below minimum_percent should not be printed
			if (mit->second < minimum_percent)
				continue;

			marker.points.clear();
			marker.id = count++;

			// compute start and end point of arrow
			angle = mit->first;

			start.x = it.getX();
			start.y = it.getY();
			start.z = it.getZ();

			end.x = it.getX() + (size / 2.0) * cos(angle);
			end.y = it.getY() + (size / 2.0) * sin(angle);
			end.z = it.getZ();

			// scale.x is the shaft diameter,
			// and scale.y is the head diameter.
			// If scale.z is not zero, it specifies the head length.
			marker.scale.x = 0.002;
			double arrow_length = hypot(start.x - end.x, start.y - end.y);
			marker.scale.y =  (arrow_length / 4) * ( 16 / theta_resolution);
			marker.scale.z = arrow_length / 2;

			if (draw3D)
				drawArrow3D(marker, start, end);
			else
				drawArrow2D(marker, start, end, marker.scale);

			// red = low, blue = high
			setMarkerColor(marker, (1.0 - mit->second / 100.0));
			// red = high, blue = low
			// setMarkerColor(&marker, mit->second / 100.0);

			markerArray.markers.push_back(marker);
		}

		// get maximal extent in y-direction (needed for positioning color table)
		if (maxY < it.getY())
		{
			maxY = it.getY();
		}
	}

	if (showColorTable)
	{
		for (size_t i = 0; i < 21; ++i)
		{
			visualization_msgs::Marker marker;

			marker.header.frame_id = frame;
			marker.header.stamp = ros::Time(0);

			marker.ns = "color_table";
			marker.id = count++;

			marker.action = visualization_msgs::Marker::ADD;
			marker.lifetime = ros::Duration();

			marker.type = visualization_msgs::Marker::CUBE;

			marker.pose.position.x = 0.05 * (double)i;
			marker.pose.position.y = maxY + 0.5;
			marker.pose.position.z = 0.0;

			marker.scale.x = 0.05;
			marker.scale.y = 0.05;
			marker.scale.z = 0.05;

			// set color according to reachable surface
			setMarkerColor(marker, 0.05 * (double)i);

			markerArray.markers.push_back(marker);
		}
	}

	ROS_INFO("Number of markers: %lu", markerArray.markers.size());
	// Publish the marker
	marker_pub.publish(markerArray);
	ROS_INFO("Minimum percent of inverse capability: %lf", min_percent);
	ROS_INFO("Maximum percent of inverse capability: %lf", max_percent);

    ros::spin();

    if (!ros::ok())
    	delete tree;
}

