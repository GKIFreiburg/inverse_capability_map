#include <capability_map/CapabilityOcTree.h>
#include <inverse_capability_map/InverseCapabilityOcTree.h>
#include <inverse_capability_map_utils/path_utils.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <tclap/CmdLine.h>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <ostream>

using namespace inverse_capability_map_utils;

struct Input {
    double resolution;
    unsigned int theta_resolution;
    std::string path_capa;
    std::string path_name;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    bool loggingEnabled;
};

CapabilityOcTree* capa_tree = NULL;

Input verifyInput(int argc, const char * const * argv)
{
    std::string msg = "Generates the inverse capability map of the region specified by given bounding box.";
    TCLAP::CmdLine cmd(msg, ' ', "1.0");

    msg = "Distance between two voxels in meter, default is 0.1 m.";
    TCLAP::ValueArg<double> resolution_arg("r", "resolution", msg, false, 0.1, "floating point");

    msg = "Resolution indicating how many different angles should be computed for the base.";
    TCLAP::ValueArg<unsigned int> theta_resolution_arg("t", "theta-resolution", msg, false, 16, "integer");

    msg = "Filename and path to the capability map. \nExample: -c mydir/mysubdir/filename.cpm";
    TCLAP::ValueArg<std::string> path_capa_arg("c", "path-capability-map", msg, true, "./capability_map.cpm", "string");

    msg = "Filename and path where the inverse capability map should be stored. \nExample: -c mydir/mysubdir/filename.icpm";
    TCLAP::ValueArg<std::string> path_name_arg("p", "path-inverse-capability-map", msg, true, "./inverse_capability_map.icpm", "string");

    msg = "The start/end point of the bounding box in x-direction.\n\
           If only one x-value is given, a slice (or a point) at this position depending on y- and z-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(x1, x2, ...) to max(x1, x2, ...).\n\
           Example: -x -0.1 -x 2.3";
    TCLAP::MultiArg<double> x_arg("x", "x-pos", msg, true, "floating point");

    msg = "The start/end point of the bounding box in y-direction.\n\
           If only one y-value is given, a slice (or a point) at this position depending on x- and z-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(y1, y2, ...) to max(y1, y2, ...).\n\
           Example: -y -0.1 -y 2.3";
    TCLAP::MultiArg<double> y_arg("y", "y-pos", msg, true, "floating point");

    msg = "The start/end point of the bounding box in z-direction.\n\
           If only one z-value is given, a slice (or a point) at this position depending on x- and y-values gets computed.\n\
           If more than 2 values are given, the boundaries are from min(z1, z2, ...) to max(z1, z2, ...).\n\
           Example: -z -0.1 -z 2.3";
    TCLAP::MultiArg<double> z_arg("z", "z-pos", msg, true, "floating point");

//    msg = "If set, writes a log file containing time required and number of computed capabilities to map_name.cpm.build_log";
//    TCLAP::SwitchArg log_arg("l", "log", msg, false);

    cmd.add(z_arg);
    cmd.add(y_arg);
    cmd.add(x_arg);
    cmd.add(resolution_arg);
    cmd.add(theta_resolution_arg);
    cmd.add(path_capa_arg);
    cmd.add(path_name_arg);
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
    input.resolution = resolution_arg.getValue();
    input.theta_resolution = theta_resolution_arg.getValue();
    input.path_capa = path_capa_arg.getValue();
    input.path_name = path_name_arg.getValue();
    input.x = x_arg.getValue();
    input.y = y_arg.getValue();
    input.z = z_arg.getValue();
//    input.loggingEnabled = log_arg.getValue();

    // load capability map
    capa_tree = CapabilityOcTree::readFile(input.path_capa);

    if (capa_tree == NULL)
    {
        ROS_ERROR("Could not load capability map file %s", input.path_capa.c_str());
        ros::shutdown();
        exit(1);
    }

    // Verify that input values are conform
    verifyPath(input.path_name, ".icpm");

    if (input.resolution <= 0.0)
    {
        ROS_ERROR("Error: resolution must be positive and greater than 0.0");
        ros::shutdown();
        exit(1);
    }

    if (input.theta_resolution <= 0)
    {
        ROS_ERROR("Error: theta resolution must be positive and greater than 0");
        ros::shutdown();
        exit(1);
    }

	return input;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inverse_capability_generator");

    Input input = verifyInput(argc, argv);

    ros::NodeHandle nhPriv("~");

    // aliases
    std::vector<double>& x_args = input.x;
    std::vector<double>& y_args = input.y;
    std::vector<double>& z_args = input.z;
    const double& resolution       = input.resolution;
    const double& theta_resolution = input.theta_resolution;

    InverseCapabilityOcTree inv_tree(resolution);
    inv_tree.setGroupName(capa_tree->getGroupName());
    inv_tree.setBaseName(capa_tree->getBaseName());
    inv_tree.setTipName(capa_tree->getTipName());
    inv_tree.setThetaResolution(theta_resolution);
	ROS_INFO("Group name is: %s", inv_tree.getGroupName().c_str());
	ROS_INFO("Base frame is: %s", inv_tree.getBaseName().c_str());
	ROS_INFO("Tip frame is: %s", inv_tree.getTipName().c_str());
	ROS_INFO("Resolution is: %g", inv_tree.getResolution());
	ROS_INFO("Theta resolution is: %d", inv_tree.getThetaResolution());

    // sort x, y and z values
    std::sort(x_args.begin(), x_args.end());
    std::sort(y_args.begin(), y_args.end());
    std::sort(z_args.begin(), z_args.end());

    // get and adjust the boundaries for iteration
    double startX = inv_tree.getAlignment(x_args[0]);
    double endX   = inv_tree.getAlignment(x_args[x_args.size() - 1]);
    double startY = inv_tree.getAlignment(y_args[0]);
    double endY   = inv_tree.getAlignment(y_args[y_args.size() - 1]);
    double startZ = inv_tree.getAlignment(z_args[0]);
    double endZ   = inv_tree.getAlignment(z_args[z_args.size() - 1]);

    double numCapsToCompute = ((endX - startX) / resolution + 1.0) * ((endY - startY) / resolution + 1.0)
                              * ((endZ - startZ) / resolution + 1.0);
    double numCapsComputed = 0.0;

    ROS_INFO("Number of inverse capabilities to compute: %d", (unsigned int)numCapsToCompute);

    //  add a small value to end due to floating point precision
    endX += resolution/100.0;
    endY += resolution/100.0;
    endZ += resolution/100.0;

    // progress in percent
    double progress = 0.0;
    double progressLimiter = 0.0;

    // store highest percent
    double max_percent = 0.0;

    for(double x = startX; x <= endX; x += resolution)
    {
        for(double y = startY; y <= endY; y += resolution)
        {
            for(double z = startZ; z <= endZ; z += resolution)
            {

				std::map<double, double> inv_capa;
                for (unsigned int i = 0; i < theta_resolution; ++i)
                {
                	// compute theta
                	double theta = (2 * M_PI / theta_resolution) * i;

                    geometry_msgs::Pose robo;
                    robo.position.x = x;
                    robo.position.y = y;
                    robo.position.z = z;
                    tf::Quaternion q;
                    q.setRPY(0, 0, theta);
                    tf::quaternionTFToMsg(q, robo.orientation);

                    tf::Pose roboPose;
                    tf::poseMsgToTF(robo, roboPose);
                    // compute inverse transformation, from object frame to robot frame
                    // object located at origin
                    tf::Pose result = roboPose.inverse();

                	// look up percent for current robot pose
                    double percent = capa_tree->getNodeCapability(result.getOrigin().x(), result.getOrigin().y(), result.getOrigin().z()).getPercentReachable();
                    if (percent == 0)
                    	continue;

                    // add (theta, percent) pair to map
                    inv_capa.insert(std::make_pair(theta, percent));

                    if (max_percent < percent)
                    	max_percent = percent;
                }
				inv_tree.setNodeInverseCapability(x, y, z, inv_capa);
				// ROS_INFO("MAP SIZE :%lu", inv_capa.size());

                numCapsComputed += 1.0;
                progress = 100.0 * numCapsComputed / numCapsToCompute;
                if (progress > progressLimiter)
                {
                    progressLimiter = progress + 0.1;
                    printf("progress: %3.2f%%\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b", progress);
                    fflush(stdout);
                }
            }
        }
    }
    printf("done              \n");

    // store maximum percent in tree
    inv_tree.setMaximumPercent(max_percent);
    if (!inv_tree.writeFile(input.path_name))
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
