#include "inverse_capability_map/InverseCapabilityOcTree.h"
#include "inverse_capability_map_utils/visualization_utils.h"
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <tclap/CmdLine.h>
#include <string>
#include <vector>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <inverse_capability_map_utils/polygon_utils.h>
#include <shape_msgs/Mesh.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace inverse_capability_map_utils;

void setArmsToSide(robot_state::RobotState& robot_state)
{
//	<!-- Self defined arm positions -->
//    <group_state name="right_arm_to_side" group="right_arm">
//		<joint name="r_shoulder_pan_joint" value="-2.110" />
//		<joint name="r_shoulder_lift_joint" value="1.230" />
//		<joint name="r_upper_arm_roll_joint" value="-2.06" />
//        <joint name="r_elbow_flex_joint" value="-1.69" />  # r_upper_arm_joint
//        <joint name="r_forearm_roll_joint" value="0.3" /> # r_elbow_flex_joint
//        <joint name="r_wrist_flex_joint" value="-1.32" />	# r_forearm_roll_joint
//        <joint name="r_wrist_roll_joint" value="1.57" /> # r_forearm_joint
//    </group_state>
//    <group_state name="left_arm_to_side" group="left_arm">
//		<joint name="l_shoulder_pan_joint" value="2.110" />
//        <joint name="l_shoulder_lift_joint" value="1.230" />
//        <joint name="l_upper_arm_roll_joint" value="2.06" />
//        <joint name="l_elbow_flex_joint" value="-1.69" />
//        <joint name="l_forearm_roll_joint" value="-0.3" />
//        <joint name="l_wrist_flex_joint" value="-1.32" />
//        <joint name="l_wrist_roll_joint" value="1.57" />
//    </group_state>

	std::string arm = "right_arm";
	const moveit::core::JointModelGroup* right_arm = robot_state.getJointModelGroup(arm);
	std::map< std::string, double > values;
	// right_arm_to_side is a group defined in pr2.srdf (tidyup_pr2_moveit_config)
	if (!right_arm->getVariableDefaultPositions("right_arm_to_side", values))
		ROS_WARN("%s: Could not set positions for %s to side!", __func__, arm.c_str());
	std::map< std::string, double >::iterator it;
	for (it = values.begin(); it != values.end(); it++)
	{
		// ROS_INFO_STREAM(it->first << " " << it->second);
		robot_state.setJointPositions(it->first, &it->second);
	}

	arm = "left_arm";
	const moveit::core::JointModelGroup* left_arm = robot_state.getJointModelGroup(arm);
	if (!left_arm->getVariableDefaultPositions("left_arm_to_side", values))
		ROS_WARN("%s: Could not set positions for %s to side!", __func__, arm.c_str());

	for (it = values.begin(); it != values.end(); it++)
	{
		// ROS_INFO_STREAM(it->first << " " << it->second);
		robot_state.setJointPositions(it->first, &it->second);
	}
}

int main(int argc, char** argv )
{
    ros::init(argc, argv, "inverse_capability_polygon_robot_visualization");

    // arguments
    TCLAP::CmdLine cmd("Visualizes the inverse capability map given by argument", ' ', "1.0");

    TCLAP::ValueArg<std::string> path_inv_cap_arg("p", "path", "Path and filename of the inverse capability map to be visualized.\n\
                                             Example: -p mydir/mysubdir/filename.icpm", true, "./inverse_capability_map.icpm", "string");

    std::string msg;
    msg = "Filename and path to the polygon file. \nExample: -c mydir/mysubdir/polygon.poly";
    TCLAP::ValueArg<std::string> path_poly_arg("i", "path-polygon", msg, true, "./polygon.poly", "string");

    msg = "Specifies the x position of the Robot.";
    TCLAP::ValueArg<double> x_robo_arg("x", "x-robot", msg, true, 0.0, "floating point");

    msg = "Specifies the y position of the Robot.";
    TCLAP::ValueArg<double> y_robo_arg("y", "y-robot", msg, true, 0.0, "floating point");

    msg = "Specifies the theta index of the orientation of the Robot.";
    TCLAP::ValueArg<double> theta_robo_arg("t", "theta-robot", msg, true, 0.0, "floating point");

    msg = "Specifies the height offset between torso link and surface. (surface is reference)";
    TCLAP::ValueArg<double> z_offset_arg("z", "z-offset", msg, true, 0.0, "floating point");

    cmd.add(x_robo_arg);
    cmd.add(y_robo_arg);
    cmd.add(z_offset_arg);
    cmd.add(theta_robo_arg);
    cmd.add(path_inv_cap_arg);
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

    std::string pathName = path_inv_cap_arg.getValue();
    InverseCapabilityOcTree* tree = InverseCapabilityOcTree::readFile(pathName);
    std::string path_poly = path_poly_arg.getValue();

    if (tree == NULL)
    {
        ROS_ERROR("Error: Inverse capability map could not be loaded.\n");
        ros::shutdown();
        exit(1);
    }

    // load polygon
    geometry_msgs::Polygon* poly = NULL;
    poly = polygon::loadPolygon(path_poly);
    if (poly == NULL)
    {
        ROS_ERROR("Could not load polygon from file %s", path_poly.c_str());
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

	double x_robot = x_robo_arg.getValue();
	double y_robot = y_robo_arg.getValue();
	double theta_robot = theta_robo_arg.getValue();
	double z_offset = z_offset_arg.getValue();
	ROS_INFO("Robot position: \nx: %lf\ny: %lf\ntheta-index: %lf\ntorso-table offset: %lf", x_robot, y_robot, theta_robot, z_offset);

	// Set up planning scene
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	planning_scene::PlanningScene planning_scene(kinematic_model);

	robot_state::RobotState& robot_state = planning_scene.getCurrentStateNonConst();
	// set robot state arms at side
	setArmsToSide(robot_state);

//    std::vector<std::string> names = robot_state.getVariableNames();
//    for (int i = 0; i < names.size(); i++)
//    {
//    	double position = robot_state.getVariablePosition(names[i]);
//    	ROS_INFO_STREAM(names[i] << " " << position);
//    }

	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;

	// Add polygon to planning scene
	ROS_INFO("Planning frame: %s", planning_scene.getPlanningFrame().c_str());
	moveit_msgs::CollisionObject co;
	co.id = "surface";
	co.header.frame_id = planning_scene.getPlanningFrame();
	co.header.stamp = ros::Time::now();
	double table_height = poly->points[0].z;
	// returned mesh is centered in the origin (0, 0), the pose defines the position and rotation
	shape_msgs::Mesh mesh = polygon::createMeshFromPolygon(*poly, - table_height, 0.03);
	co.meshes.push_back(mesh);
	// pose position (0, 0, 0), orientation (0, 0, 0, 1)
	geometry_msgs::Pose pose = geometry_msgs::Pose();
	Eigen::Affine3d e = robot_state.getGlobalLinkTransform(tree->getBaseName());
	tf::Transform t;
	tf::transformEigenToTF(e, t);
	double torso_height = t.getOrigin().z();
	ROS_INFO("Torso height: %lf", torso_height);

	octomap::OcTreeKey key = tree->coordToKey(x_robot, y_robot, z_offset);
	octomath::Vector3 v = tree->keyToCoord(key);

	pose.position.z = torso_height + v.z() + tree->getResolution() / 2;
	co.mesh_poses.push_back(pose);
	co.operation = co.ADD;
	planning_scene.processCollisionObjectMsg(co);
	moveit_msgs::ObjectColor oc;
	oc.id = co.id;
	oc.color.r = 0.67;
	oc.color.g = 0.33;
	oc.color.b = 0.0;
	oc.color.a = 1.0;
	planning_scene.setObjectColor(oc.id, oc.color);

	ros::NodeHandle nh;
	ros::Publisher pub_ps = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
	ROS_INFO("Publishing planning scene to: planning_scene");

	InverseCapability inv_cap = tree->getNodeInverseCapability(x_robot, y_robot, z_offset);

	double theta = 2 * M_PI / tree->getThetaResolution() * theta_robot;
	if (inv_cap.getThetasPercent().find(theta) == inv_cap.getThetasPercent().end())
	{
		ROS_ERROR("Could not find theta orientation!");
		std::map<double, double>::const_iterator it;
		for (it = inv_cap.getThetasPercent().begin(); it != inv_cap.getThetasPercent().end(); it++)
			ROS_INFO("Possible theta: %lf", it->first);
		return 1;
	}

	// take care of torso to base transform
	const moveit::core::RobotModelConstPtr robot_model = robot_state.getRobotModel();
	const moveit::core::LinkModel* torso_link = robot_model->getLinkModel(tree->getBaseName());
	const Eigen::Affine3d& torso_trans = robot_state.getGlobalLinkTransform(torso_link->getName());
	const Eigen::Affine3d& base_trans = robot_state.getGlobalLinkTransform(torso_link->getParentLinkModel()->getName());
	// convert eigen into tf
	tf::Pose torso_transform, base_transform, torso_base_transform;
	tf::poseEigenToTF(torso_trans, torso_transform);
	tf::poseEigenToTF(base_trans, base_transform);
	// from torso to base transform
	torso_base_transform = torso_transform.inverseTimes(base_transform);
	ROS_ASSERT(torso_base_transform.getOrigin().getX() == 0.05);
	ROS_ASSERT(torso_base_transform.getOrigin().getY() == 0.0);

	// full collision check, check if robot is in collision with polygon using base_link as reference frame
	robot_state.setVariablePosition("world_joint/x", v.x() + torso_base_transform.getOrigin().getX());
	robot_state.setVariablePosition("world_joint/y", v.y() + torso_base_transform.getOrigin().getY());
	robot_state.setVariablePosition("world_joint/theta", theta);

	planning_scene.setCurrentState(robot_state);

	moveit_msgs::PlanningScene ps_msg;
	planning_scene.getPlanningSceneMsg(ps_msg);
	pub_ps.publish(ps_msg);


    ros::spin();

    if (!ros::ok())
    	delete tree;
}

