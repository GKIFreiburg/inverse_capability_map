#include <inverse_capability_map/InverseCapabilityOcTree.h>
#include <inverse_capability_map_utils/path_utils.h>
#include <inverse_capability_map_utils/polygon_utils.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Polygon.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <tclap/CmdLine.h>
#include <string>
#include <math.h>

using namespace inverse_capability_map_utils;

struct Input {
    double resolution;
    std::string path_object;
    std::string path_name;
    std::string path_poly;
    bool loggingEnabled;
};

InverseCapabilityOcTree* object_tree = NULL;
geometry_msgs::Polygon* poly = NULL;

Input verifyInput(int argc, const char * const * argv)
{
    std::string msg = "Generates the inverse capability map of the specified table.";
    TCLAP::CmdLine cmd(msg, ' ', "1.0");

    msg = "Distance between two voxels in meter, default is 0.1 m.";
    TCLAP::ValueArg<double> resolution_arg("r", "resolution", msg, false, 0.1, "floating point");

//    msg = "Resolution indicating how many different angles should be computed for the base.";
//    TCLAP::ValueArg<unsigned int> theta_resolution_arg("t", "theta-resolution", msg, false, 16, "integer");

    msg = "Filename and path to the inverse capability object map. \nExample: -o mydir/mysubdir/filename.icpm";
    TCLAP::ValueArg<std::string> path_object_arg("o", "path-inverse-capability-object-map", msg, true, "./inverse_capability_map.icpm", "string");

    msg = "Filename and path where the inverse capability table map should be stored. \nExample: -c mydir/mysubdir/filename.icpm";
    TCLAP::ValueArg<std::string> path_name_arg("p", "path-inverse-capability-table-map", msg, true, "./inverse_capability_map.icpm", "string");

    msg = "Filename and path to the polygon file. \nExample: -c mydir/mysubdir/polygon.poly";
    TCLAP::ValueArg<std::string> path_poly_arg("i", "path-polygon", msg, true, "./polygon.poly", "string");

//    msg = "If set, writes a log file containing time required and number of computed capabilities to map_name.cpm.build_log";
//    TCLAP::SwitchArg log_arg("l", "log", msg, false);

    cmd.add(resolution_arg);
    cmd.add(path_object_arg);
    cmd.add(path_name_arg);
    cmd.add(path_poly_arg);
//    cmd.add(log_arg);

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

    // get values from arguments
    Input input;
    input.resolution  = resolution_arg.getValue();
    input.path_object = path_object_arg.getValue();
    input.path_name   = path_name_arg.getValue();
    input.path_poly   = path_poly_arg.getValue();
//    input.loggingEnabled = log_arg.getValue();

    // load inverse capability object map
    object_tree = InverseCapabilityOcTree::readFile(input.path_object);
    if (object_tree == NULL)
    {
        ROS_ERROR("Could not load inverse capability object map file %s", input.path_object.c_str());
        ros::shutdown();
        exit(1);
    }

    // load polygon
    poly = polygon::loadPolygon(input.path_poly);
    if (poly == NULL)
    {
        ROS_ERROR("Could not load polygon from file %s", input.path_poly.c_str());
        ros::shutdown();
        exit(1);
    }

    // Verify path of inverse capability surface map
    verifyPath(input.path_name, ".icpm");

    if (input.resolution <= 0.0)
    {
        ROS_ERROR("Error: resolution must be positive and greater than 0.0");
        ros::shutdown();
        exit(1);
    }

	return input;
}

bool robotPositionInTableContour(const tf::Point& robot, const double& width, const double& length)
{
	return -width/2 <= robot.getX() && robot.getX() <= width/2 &&
			-length/2 <= robot.getY() && robot.getY() <= length/2;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inverse_capability_object_generator");

    Input input = verifyInput(argc, argv);
    ros::NodeHandle nhPriv("~");

    // aliases
    const double& resolution       = input.resolution;
    polygon::bbox bbox = polygon::computeCenteredBoundingBox(*poly);
    double widthBbox, lengthBbox;
    widthBbox = bbox.xmax - bbox.xmin;
    lengthBbox = bbox.ymax - bbox.ymin;
    ROS_INFO("Bounding box width : %lf", widthBbox);
    ROS_INFO("Bounding box length: %lf", lengthBbox);
    polygon::center center = polygon::computeBoundingBoxCenter(bbox);
    ROS_INFO("Bounding box center: (%lf, %lf)", center.x, center.y);

    InverseCapabilityOcTree table_tree(resolution);
    table_tree.setGroupName(object_tree->getGroupName());
    table_tree.setBaseName(object_tree->getBaseName());
    table_tree.setTipName(object_tree->getTipName());
    table_tree.setThetaResolution(object_tree->getThetaResolution());
    ROS_INFO("Group name is: %s", table_tree.getGroupName().c_str());
    ROS_INFO("Base frame is: %s", table_tree.getBaseName().c_str());
    ROS_INFO("Tip frame is: %s", table_tree.getTipName().c_str());
    ROS_INFO("Resolution is: %g", table_tree.getResolution());
    ROS_INFO("Theta resolution is: %d\n", table_tree.getThetaResolution());

    unsigned int width_cells, length_cells, grid_cells;
    width_cells  = round(widthBbox / resolution);
    length_cells = round(lengthBbox / resolution);
    grid_cells   = width_cells * length_cells;
    ROS_INFO("Number of width  cells: %d", width_cells);
    ROS_INFO("Number of length cells: %d", length_cells);
    ROS_INFO("Number of grid   cells: %d", grid_cells);

    double numCapsToCompute = width_cells * length_cells * object_tree->size();
    double numCapsComputed = 0.0;

    ROS_INFO("Number of inverse capabilities to compute: %d", (unsigned int) numCapsToCompute);

    // start pose at left lower corner of table, but in center of grid cell
    geometry_msgs::Pose start_pose;
    start_pose.position.x = center.x - widthBbox / 2 + resolution / 2;
    start_pose.position.y = center.y - lengthBbox / 2 + resolution / 2;
    start_pose.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    tf::quaternionTFToMsg(q, start_pose.orientation);

//    //  add a small value to end due to floating point precision
//    endX += resolution/100.0;
//    endY += resolution/100.0;
//    endZ += resolution/100.0;

    // progress in percent
    double progress = 0.0;
    double progressLimiter = 0.0;

//    ROS_WARN_STREAM(*poly);
//    double x = 0;
//    double y = 0;
//    int c = polygon::pointInPolygon(*poly, x, y);
//
//    if (c % 2 == 0)
//    	ROS_INFO("point outside polygon, %d", c);
//    else
//    	ROS_INFO("point inside polygon, %d", c);

	geometry_msgs::Point object_in_table_frame, robot_in_object_frame;
	for (unsigned int l = 0; l != length_cells; l++)
    {
		for (unsigned int w = 0; w != width_cells; w++)
    	{
    		// update object_pose
			object_in_table_frame = start_pose.position;
			object_in_table_frame.x = start_pose.position.x + w * resolution;
			object_in_table_frame.y = start_pose.position.y + l * resolution;

    		// loop through all inverse capabilities
    		for (InverseCapabilityOcTree::leaf_iterator it = object_tree->begin_leafs(); it != object_tree->end_leafs(); ++it)
    		{
    			// Printing progress bar
                numCapsComputed += 1.0;
                progress = 100.0 * numCapsComputed / numCapsToCompute;
                if (progress > progressLimiter)
                {
                    progressLimiter = progress + 0.1;
                    printf("progress: %3.2f%%\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b", progress);
                    fflush(stdout);
                }

    			robot_in_object_frame.x = it.getX();
    			robot_in_object_frame.y = it.getY();
    			robot_in_object_frame.z = it.getZ();

    			// translate robot pose into table frame, rotation of robot stays the same
    			tf::Point robot_in_obj_frame;
                tf::Point obj_in_table_frame;
                tf::Point robot_in_table_frame;

                tf::pointMsgToTF(robot_in_object_frame, robot_in_obj_frame);
                tf::pointMsgToTF(object_in_table_frame, obj_in_table_frame);
                robot_in_table_frame = robot_in_obj_frame + obj_in_table_frame;

                // check if robot position is inside polygon, meaning invalid position
                // if return value of pointInPolygon is odd then position is in polygon
                if (polygon::pointInPolygon(*poly, robot_in_table_frame.getX(), robot_in_table_frame.getY()) % 2 != 0)
                {
//                	ROS_INFO("Position: (%lf, %lf) is in Polygon", robot_in_table_frame.getX(), robot_in_table_frame.getY());
                	continue;
                }

                InverseCapability inv_obj = it->getValue();

                // look in current tree, if inverse capability already exists, if not an empty InverseCapability is return
                InverseCapability inv_table = table_tree.getNodeInverseCapability(robot_in_table_frame.getX(), robot_in_table_frame.getY(), robot_in_table_frame.getZ());

                InverseCapability new_inv_cap = inv_obj + inv_table;

                // add new computed InverseCapability to table tree
                table_tree.setNodeInverseCapability(robot_in_table_frame.getX(), robot_in_table_frame.getY(), robot_in_table_frame.getZ(), new_inv_cap);
    		}
    	}
    }

	// normalize nodes
	for (InverseCapabilityOcTree::leaf_iterator it = table_tree.begin_leafs(); it != table_tree.end_leafs(); ++it)
		it->normalize((double) grid_cells);
//		it->normalize(1.0);


    printf("done              \n");

    if (!table_tree.writeFile(input.path_name))
    {
        ROS_ERROR("Error: Could not write to file %s\n", input.path_name.c_str());
        ros::shutdown();
        exit(1);
    }
    else
    {
        ROS_INFO("Inverse Capability map written to file %s", input.path_name.c_str());
    }
}
