#include "inverse_capability_map/InverseCapabilityOcTree.h"
#include "inverse_capability_map_utils/visualization_utils.h"
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <tclap/CmdLine.h>
#include <string>
#include <vector>

#include <inverse_capability_map_utils/polygon_utils.h>
#include <shape_msgs/Mesh.h>

using namespace inverse_capability_map_utils;

void showGrid(visualization_msgs::MarkerArray& markerArray, const geometry_msgs::Polygon* poly, const double resolution)
{
	polygon::bbox bbox = polygon::computeBoundingBox(*poly);
	double widthBbox, lengthBbox;
	widthBbox = bbox.xmax - bbox.xmin;
	lengthBbox = bbox.ymax - bbox.ymin;
	polygon::center center = polygon::computeBoundingBoxCenter(bbox);

	unsigned int width_cells, length_cells, grid_cells;
	width_cells  = round(widthBbox / resolution);
	length_cells = round(lengthBbox / resolution);
	grid_cells   = width_cells * length_cells;

	visualization_msgs::Marker marker;
	std::string frame = "map";
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time(0);
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration();
	marker.ns = "grid";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;

	marker.color.r = (float)255/255;
	marker.color.g = (float)0/255;
	marker.color.b = (float)0/255;
	marker.color.a = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.02;
	marker.scale.y = 0.02;
	marker.scale.z = 0.02;
	double x = center.x - widthBbox / 2;
	double y = center.y - lengthBbox / 2;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0.02;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	for (unsigned int l = 0; l <= length_cells; l++)
	{
		for (unsigned int w = 0; w <= width_cells; w++)
		{
			marker.id += 1;
			marker.pose.position.x = x + w * resolution;
			marker.pose.position.y = y + l * resolution;
			markerArray.markers.push_back(marker);
		}
	}
}

int main(int argc, char** argv )
{
    ros::init(argc, argv, "inverse_capability_polygon_visualization");

    // arguments
    TCLAP::CmdLine cmd("Visualizes the inverse capability map given by argument", ' ', "1.0");

    TCLAP::ValueArg<std::string> path_name_arg("p", "path", "Path and filename of the inverse capability map to be visualized.\n\
                                             Example: -p mydir/mysubdir/filename.icpm", true, "./inverse_capability_map.icpm", "string");

    std::string msg;
    msg = "Filename and path to the polygon file. \nExample: -c mydir/mysubdir/polygon.poly";
    TCLAP::ValueArg<std::string> path_poly_arg("i", "path-polygon", msg, true, "./polygon.poly", "string");

    msg = "Specifies the region in x-direction to be visualized.\n\
           If no x-value is given, depending on y- and z-values, all stored capabilities in x-direction are displayed.\n\
           If only one x-value is given, a slice (or a point) at this position depending on y- and z-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(x1, x2, ...) to max(x1, x2, ...).\n\
           Example: -x -0.1 -x 2.3";
    TCLAP::MultiArg<double> x_arg("x", "x-pos", msg, false, "floating point");

    msg = "Specifies the region in y-direction to be visualized.\n\
           If no y-value is given, depending on x- and z-values, all stored capabilities in y-direction are displayed.\n\
           If only one y-value is given, a slice (or a point) at this position depending on x- and z-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(y1, y2, ...) to max(y1, y2, ...).\n\
           Example: -y -0.1 -y 2.3";
    TCLAP::MultiArg<double> y_arg("y", "y-pos", msg, false, "floating point");

    msg = "Specifies the region in z-direction to be visualized.\n\
           If no z-value is given, depending on x- and y-values, all stored capabilities in z-direction are displayed.\n\
           If only one z-value is given, a slice (or a point) at this position depending on x- and y-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(z1, z2, ...) to max(z1, z2, ...).\n\
           Example: -z -0.1 -z 2.3";
    TCLAP::MultiArg<double> z_arg("z", "z-pos", msg, false, "floating point");

    msg = "If set, shows a color table from blue = best to red = worst";
    TCLAP::SwitchArg color_arg("c", "colortable", msg, false);

    msg = "If set, drawing 3d arrows, else 2d arrows are drawn";
    TCLAP::SwitchArg dimension_arg("d", "3darrowdimension", msg, false);

    cmd.add(x_arg);
    cmd.add(y_arg);
    cmd.add(z_arg);
    cmd.add(color_arg);
    cmd.add(dimension_arg);
    cmd.add(path_name_arg);
    cmd.add(path_poly_arg);

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

    bool showColorTable = color_arg.getValue();
    bool draw3D = dimension_arg.getValue();

    std::string pathName = path_name_arg.getValue();
    InverseCapabilityOcTree* tree = InverseCapabilityOcTree::readFile(pathName);
    std::string path_poly = path_poly_arg.getValue();

    if (tree == NULL)
    {
        ROS_ERROR("Error: Inverse capability map could not be loaded.\n");
        ros::shutdown();
        exit(1);
    }

    unsigned int theta_resolution = tree->getThetaResolution();

    ROS_INFO("Group name is: %s", tree->getGroupName().c_str());
    ROS_INFO("Base frame is: %s", tree->getBaseName().c_str());
    ROS_INFO("Tip frame is: %s", tree->getTipName().c_str());
    ROS_INFO("Resolution is: %g", tree->getResolution());
    ROS_INFO("Theta resolution is: %d", theta_resolution);

	ros::NodeHandle nhPriv("~");
	double minimum_percent;
	nhPriv.param("minimum_percent", minimum_percent, 0.0);
	std::string poly_name;
	nhPriv.param<std::string>("poly_name", poly_name, "#UNDEFINED");
	ROS_INFO("Polygon name is: %s\n", poly_name.c_str());
	bool show_grid;
	nhPriv.param("show_grid", show_grid, false);
	double arrow_shaft_diameter;
	nhPriv.param("arrow_shaft_diameter", arrow_shaft_diameter, 0.002);

    // get x, y and z values and sort them
    std::vector<double> xValues = x_arg.getValue();
    std::vector<double> yValues = y_arg.getValue();
    std::vector<double> zValues = z_arg.getValue();

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
	std::string frame = "map";
	tableMarker.header.frame_id = frame;
	tableMarker.header.stamp = ros::Time(0);
	tableMarker.action = visualization_msgs::Marker::ADD;
	tableMarker.lifetime = ros::Duration();
	tableMarker.ns = "table";
	tableMarker.id = 0;
	tableMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;

	ROS_INFO("Markers are published in frame: %s", frame.c_str());

	geometry_msgs::Polygon* poly = polygon::loadPolygon(path_poly);
	shape_msgs::Mesh mesh = polygon::createMeshFromPolygon(*poly, 0.0, 0.05);
	for (size_t i = 0; i < mesh.triangles.size(); i++)
	{
		shape_msgs::MeshTriangle tri = mesh.triangles[i];
		unsigned int p0 = tri.vertex_indices[0];
		unsigned int p1 = tri.vertex_indices[1];
		unsigned int p2 = tri.vertex_indices[2];

		ROS_ASSERT(p0 < mesh.vertices.size());
		ROS_ASSERT(p1 < mesh.vertices.size());
		ROS_ASSERT(p2 < mesh.vertices.size());
		tableMarker.points.push_back(mesh.vertices[p0]);
		tableMarker.points.push_back(mesh.vertices[p1]);
		tableMarker.points.push_back(mesh.vertices[p2]);

//		tableMarker.color.r = 0.67;
//		tableMarker.color.g = 0.33;
//		tableMarker.color.b = 0.0;
		tableMarker.color.r = (float)205/255;
		tableMarker.color.g = (float)102/255;
		tableMarker.color.b = (float)29/255 *0;
		tableMarker.color.a = 1.0;
	}

//	tableMarker.type = visualization_msgs::Marker::CUBE;
	tableMarker.pose.position.x = 0;
	tableMarker.pose.position.y = 0;
	tableMarker.pose.position.z = 0;
//	tableMarker.pose.position.z = 0.02;
	tableMarker.pose.orientation.x = 0.0;
	tableMarker.pose.orientation.y = 0.0;
	tableMarker.pose.orientation.z = 0.0;
	tableMarker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	tableMarker.scale.x = 1.0;
	tableMarker.scale.y = 1.0;
	tableMarker.scale.z = 0.6;

	markerArray.markers.push_back(tableMarker);

	if (show_grid)
		showGrid(markerArray, poly, tree->getResolution());

	double min_percent, max_percent;
	min_percent = HUGE_VAL;
	max_percent = 0.0;

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

		marker.header.frame_id = frame;
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

//			start.x = it.getX() - tree->getResolution() / 2;
//			start.y = it.getY() - tree->getResolution() / 2;
			start.x = it.getX();
			start.y = it.getY();
			start.z = it.getZ();

			end.x = start.x + (size / 2.0) * cos(angle);
			end.y = start.y + (size / 2.0) * sin(angle);
			end.z = start.z;

			// scale.x is the shaft diameter,
			// and scale.y is the head diameter.
			// If scale.z is not zero, it specifies the head length.
			marker.scale.x = arrow_shaft_diameter;
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

			marker.pose.position.x = 0.05 * (double)i - 0.5;
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

