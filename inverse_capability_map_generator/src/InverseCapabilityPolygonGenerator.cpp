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

    cmd.add(resolution_arg);
    cmd.add(path_object_arg);
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

    // get values from arguments
    Input input;
    input.resolution  = resolution_arg.getValue();
    input.path_object = path_object_arg.getValue();
    input.path_name   = path_name_arg.getValue();
    input.path_poly   = path_poly_arg.getValue();

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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "inverse_capability_polygon_generator");

	Input input = verifyInput(argc, argv);
	ros::NodeHandle nhPriv("~");

	// parameters and aliases
	const double& resolution       = input.resolution;

	InverseCapabilityOcTree table_tree(resolution);
	table_tree.setGroupName(object_tree->getGroupName());
	table_tree.setBaseName(object_tree->getBaseName());
	table_tree.setTipName(object_tree->getTipName());
	table_tree.setThetaResolution(object_tree->getThetaResolution());
	ROS_INFO("Group name is: %s", table_tree.getGroupName().c_str());
	ROS_INFO("Base frame is: %s", table_tree.getBaseName().c_str());
	ROS_INFO("Tip frame is: %s", table_tree.getTipName().c_str());
	ROS_INFO("Resolution is: %g", table_tree.getResolution());
	ROS_INFO("Theta resolution is: %d", table_tree.getThetaResolution());
	bool collision_checking;
	nhPriv.param("collision_checking", collision_checking, true);
	ROS_INFO("Collision checking is turned %s", collision_checking ? "ON" : "OFF");
	std::string poly_name;
	nhPriv.param<std::string>("poly_name", poly_name, "#UNDEFINED");
	ROS_INFO("Polygon name is: %s\n", poly_name.c_str());


	polygon::bbox bbox = polygon::computeBoundingBox(*poly);
	double widthBbox, lengthBbox;
	widthBbox = bbox.xmax - bbox.xmin;
	lengthBbox = bbox.ymax - bbox.ymin;
	ROS_INFO("Bounding box width : %lf", widthBbox);
	ROS_INFO("Bounding box length: %lf", lengthBbox);
	polygon::center center = polygon::computeBoundingBoxCenter(bbox);
	ROS_INFO("Bounding box center: (%lf, %lf)", center.x, center.y);

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

	// progress in percent
	double progress = 0.0;
	double progressLimiter = 0.0;

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
	shape_msgs::Mesh mesh = polygon::createMeshFromPolygon(*poly, 0.0, 0.03);
	co.meshes.push_back(mesh);
	// pose position (0, 0, 0), orientation (0, 0, 0, 1)
	geometry_msgs::Pose pose = geometry_msgs::Pose();
	Eigen::Affine3d e = robot_state.getGlobalLinkTransform(object_tree->getBaseName());
	tf::Transform t;
	tf::transformEigenToTF(e, t);
	double torso_height = t.getOrigin().z();
	ROS_INFO("Torso height: %lf", torso_height);
	pose.position.z = torso_height;
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


	// start pose at left lower corner of bounding box, but in center of grid cell
	geometry_msgs::PoseStamped start_pose;
	start_pose.header.frame_id = planning_scene.getPlanningFrame();
	start_pose.pose.position.x = center.x - widthBbox / 2;
	start_pose.pose.position.y = center.y - lengthBbox / 2;
	start_pose.pose.position.z = t.getOrigin().z();
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	tf::quaternionTFToMsg(q, start_pose.pose.orientation);

	// take care of torso to base transform
	const moveit::core::RobotModelConstPtr robot_model = robot_state.getRobotModel();
	const moveit::core::LinkModel* torso_link = robot_model->getLinkModel(object_tree->getBaseName());
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


//	octomap::OcTreeKey key = object_tree->coordToKey(0.95, -0.15, 0.05);
//	octomath::Vector3 v = object_tree->keyToCoord(key);
//	ROS_WARN("x: %lf, y: %lf, z: %lf", v.x(), v.y(), v.z());
//	octomap::OcTreeKey key2 = object_tree->coordToKey(0.04,0,0);
//	v = object_tree->keyToCoord(key2);
//	ROS_WARN("x: %lf, y: %lf, z: %lf", v.x(), v.y(), v.z());
//	if (key == key2)
//		ROS_WARN("EQUAL");
//	else
//		ROS_WARN("not equal");

	geometry_msgs::PoseStamped object_in_map_frame, robot_torso_in_object_frame;
	for (unsigned int l = 0; l <= length_cells; l++)
    {
		for (unsigned int w = 0; w <= width_cells; w++)
    	{
    		// update object_pose
			object_in_map_frame = start_pose;
			object_in_map_frame.pose.position.x = start_pose.pose.position.x + w * resolution;
			object_in_map_frame.pose.position.y = start_pose.pose.position.y + l * resolution;

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

				// set header and orientation
				robot_torso_in_object_frame = start_pose;
				robot_torso_in_object_frame.pose.position.x = it.getX();
				robot_torso_in_object_frame.pose.position.y = it.getY();
				robot_torso_in_object_frame.pose.position.z = it.getZ();

				// translate robot pose into map frame
				ROS_ASSERT(robot_torso_in_object_frame.header.frame_id == object_in_map_frame.header.frame_id);
				tf::Pose robot_torso_in_obj_frame;
				tf::poseMsgToTF(robot_torso_in_object_frame.pose, robot_torso_in_obj_frame);
				tf::Pose obj_in_map_frame;
				tf::poseMsgToTF(object_in_map_frame.pose, obj_in_map_frame);
				tf::Pose robot_torso_in_map_frame;
				robot_torso_in_map_frame = robot_torso_in_obj_frame * obj_in_map_frame;

				// check if robot position is inside polygon, meaning invalid position
				// if return value of pointInPolygon is odd then position is in polygon
				if (polygon::pointInPolygon(*poly, robot_torso_in_map_frame.getOrigin().x(), robot_torso_in_map_frame.getOrigin().y()) % 2 != 0)
					continue;

				InverseCapability inv_obj = it->getValue();

				if (collision_checking)
				{
					std::map<double, double> thetas = inv_obj.getThetasPercent();
					std::map<double, double>::iterator mit;

					std::set<double> delete_thetas;
					for (mit = thetas.begin(); mit != thetas.end(); mit++)
					{
						// full collision check, check if robot is in collision with polygon using base_link as reference frame
						// TODO: replace +0.05 with transform. From Torso to Base Link
//						robot_state.setVariablePosition("world_joint/x", robot_torso_in_map_frame.getOrigin().x() + torso_base_transform.getOrigin().getX());
//						robot_state.setVariablePosition("world_joint/y", robot_torso_in_map_frame.getOrigin().y() + torso_base_transform.getOrigin().getY());
						robot_state.setVariablePosition("world_joint/x", robot_torso_in_map_frame.getOrigin().x());
						robot_state.setVariablePosition("world_joint/y", robot_torso_in_map_frame.getOrigin().y());
						robot_state.setVariablePosition("world_joint/theta", mit->first);
						planning_scene.setCurrentState(robot_state);

						// adapt height of surface for collision checking
						co.mesh_poses[0].position.z = torso_height + it.getZ();
						co.operation = co.MOVE;
						co.meshes.clear();
						planning_scene.processCollisionObjectMsg(co);

						collision_result.clear();
						planning_scene.checkCollision(collision_request, collision_result, robot_state);
						// if collision is found, remove (theta, percent) from thetas
						if (collision_result.collision)
							// TODO: ERROR HERE!!!
//							thetas.erase(mit);
							delete_thetas.insert(mit->first);
					}
					for (std::set<double>::iterator sit = delete_thetas.begin(); sit != delete_thetas.end(); sit++)
						thetas.erase(*sit);

					// update inv_obj, in case some (theta, percent) have been removed
					inv_obj.setThetasPercent(thetas);
				}

				// substract torso height again, so that values are centered around torso_lift_link
				tf::Pose robot_torso = robot_torso_in_map_frame;
				tf::Vector3 v = robot_torso.getOrigin();
				v.setZ(v.getZ() - t.getOrigin().z());
				robot_torso.setOrigin(v);

				// look in current tree if inverse capability already exists, if not an empty InverseCapability is return
				InverseCapability inv_table = table_tree.getNodeInverseCapability(robot_torso.getOrigin().x(),
						robot_torso.getOrigin().y(),
						robot_torso.getOrigin().z());

				InverseCapability new_inv_cap = inv_obj + inv_table;

				// add new computed InverseCapability to table tree
				table_tree.setNodeInverseCapability(robot_torso.getOrigin().x(),
						robot_torso.getOrigin().y(),
						robot_torso.getOrigin().z(),
						new_inv_cap);
			}
		}
	}

	double max_percent;
	size_t num_inv_cap = 0;
	// normalize nodes
	for (InverseCapabilityOcTree::leaf_iterator it = table_tree.begin_leafs(); it != table_tree.end_leafs(); ++it)
	{
		// normalize InverseCapability by the number of grid cells
		it->normalize((double) grid_cells);
		// check for max percent
		const std::pair<double, double> pair = it->getInverseCapability().getMaxThetaPercent();
		if (max_percent < pair.second)
			max_percent = pair.second;
		// Count the number of inverse capabilities
		num_inv_cap += it->getInverseCapability().getThetasPercent().size();
	}
	table_tree.setMaximumPercent(max_percent);

    printf("done              \n");
    ROS_INFO("Maximum percent of inverse capability: %lf", max_percent);
    ROS_INFO("Number of total inverse capabilities: %lu", num_inv_cap);

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
