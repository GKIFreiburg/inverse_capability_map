#include <inverse_capability_map/InverseCapabilityOcTree.h>
#include <inverse_capability_map_utils/path_utils.h>
#include <inverse_capability_map_utils/polygon_utils.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Polygon.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>
#include <tclap/CmdLine.h>
#include <string>
#include <math.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace inverse_capability_map_utils;

struct Input {
    double resolution;
    std::string path_map1;
    std::string path_map2;
    std::string path_name;
    InverseCapabilityOcTree* map1_tree;
    InverseCapabilityOcTree* map2_tree;
};


Input verifyInput(int argc, const char * const * argv)
{
    std::string msg = "Generates the inverse capability map of the specified table.";
    TCLAP::CmdLine cmd(msg, ' ', "1.0");

    msg = "Distance between two voxels in meter, default is 0.1 m.";
    TCLAP::ValueArg<double> resolution_arg("r", "resolution", msg, false, 0.1, "floating point");

    msg = "Filename and path to the inverse capability map1. \nExample: -c mydir/mysubdir/map1.icmp";
    TCLAP::ValueArg<std::string> path_map1_arg("i", "path-inverse-capability-map1", msg, true, "./inverse_capability_map1.icmp", "string");

    msg = "Filename and path to the inverse capability map2. \nExample: -o mydir/mysubdir/map2.icpm";
    TCLAP::ValueArg<std::string> path_map2_arg("o", "path-inverse-capability-map2", msg, true, "./inverse_capability_map2.icpm", "string");

    msg = "Filename and path where the new inverse capability table map should be stored. \nExample: -c mydir/mysubdir/filename.icpm";
    TCLAP::ValueArg<std::string> path_name_arg("p", "path-inverse-capability-map", msg, true, "./inverse_capability_map.icpm", "string");

    cmd.add(resolution_arg);
    cmd.add(path_map1_arg);
    cmd.add(path_map2_arg);
    cmd.add(path_name_arg);

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
    input.path_map1   = path_map1_arg.getValue();
    input.path_map2   = path_map2_arg.getValue();
    input.path_name   = path_name_arg.getValue();
//    input.loggingEnabled = log_arg.getValue();

    // load inverse capability object map
    input.map1_tree = InverseCapabilityOcTree::readFile(input.path_map1);
    if (input.map1_tree == NULL)
    {
        ROS_ERROR("Could not load inverse capability map file %s", input.path_map1.c_str());
        ros::shutdown();
        exit(1);
    }

    // load inverse capability object map
    input.map2_tree = InverseCapabilityOcTree::readFile(input.path_map2);
    if (input.map2_tree == NULL)
    {
        ROS_ERROR("Could not load inverse capability map file %s", input.path_map2.c_str());
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "inverse_capability_combined_generator");

	Input input = verifyInput(argc, argv);
	ros::NodeHandle nhPriv("~");

	// parameters and aliases
	const double& resolution = input.resolution;
	const InverseCapabilityOcTree* const map1 = input.map1_tree;
	const InverseCapabilityOcTree* const map2 = input.map2_tree;
//	ROS_ASSERT(map1->size() == map2->size());

	ROS_ASSERT(map1->getResolution() == map2->getResolution());
	InverseCapabilityOcTree combined_tree(resolution);
	combined_tree.setGroupName("arms");
	ROS_ASSERT(map1->getBaseName() == map2->getBaseName());
	combined_tree.setBaseName(map1->getBaseName());
	combined_tree.setTipName("*_wrist_roll_link");
	ROS_ASSERT(map1->getThetaResolution() == map2->getThetaResolution());
	combined_tree.setThetaResolution(map1->getThetaResolution());
	ROS_INFO("Group name is: %s", combined_tree.getGroupName().c_str());
	ROS_INFO("Base frame is: %s", combined_tree.getBaseName().c_str());
	ROS_INFO("Tip frame is: %s", combined_tree.getTipName().c_str());
	ROS_INFO("Resolution is: %g", combined_tree.getResolution());
	ROS_INFO("Theta resolution is: %d\n", combined_tree.getThetaResolution());

	double numCapsToCompute = map1->size();
	double numCapsComputed = 0.0;

	ROS_INFO("Number of inverse capabilities to compute: %d", (unsigned int) numCapsToCompute);

	// progress in percent
	double progress = 0.0;
	double progressLimiter = 0.0;

	// loop through all inverse capabilities
	for (InverseCapabilityOcTree::leaf_iterator it = map1->begin_leafs(); it != map1->end_leafs(); ++it)
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

		InverseCapability inv_map1 = it->getValue();

		// look in inverse capability in map2
		InverseCapability inv_map2 = map2->getNodeInverseCapability(it.getX(), it.getY(), it.getZ());

		// Compute new inverse capability by adding inv caps of map 1 and map 2
		InverseCapability new_inv_cap = inv_map1 + inv_map2;

		combined_tree.setNodeInverseCapability(it.getX(), it.getY(), it.getZ(), new_inv_cap);
	}

	double max_percent;
	size_t num_inv_cap = 0;
	// normalize nodes
	for (InverseCapabilityOcTree::leaf_iterator it = combined_tree.begin_leafs(); it != combined_tree.end_leafs(); ++it)
	{
		// normalize InverseCapability 2.0, since both arms
		it->normalize((double) 2.0);
		// check for max percent
		const std::pair<double, double> pair = it->getInverseCapability().getMaxThetaPercent();
		if (max_percent < pair.second)
			max_percent = pair.second;
		// Count the number of inverse capabilities
		num_inv_cap += it->getInverseCapability().getThetasPercent().size();
	}
	combined_tree.setMaximumPercent(max_percent);

    printf("done              \n");
    ROS_INFO("Maximum percent of inverse capability: %lf", max_percent);
    ROS_INFO("Number of total inverse capabilities: %lu", num_inv_cap);

    if (!combined_tree.writeFile(input.path_name))
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
