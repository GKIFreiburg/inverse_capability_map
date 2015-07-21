#include "inverse_capability_map/InverseCapabilityDrawing.h"
#include <tf_conversions/tf_eigen.h>

InverseCapabilityDrawing::InverseCapabilityDrawing(long int seed)
{
    if(seed == 0)
        srand48(time(NULL));
    else
        srand48(seed);
}

InverseCapabilityDrawing::~InverseCapabilityDrawing()
{
}

InverseCapabilityDrawing::posePercent InverseCapabilityDrawing::drawBestOfXSamples(planning_scene::PlanningScene& planning_scene,
			const InverseCapabilityOcTree* tree, const geometry_msgs::PoseStamped& surface_pose, unsigned int numberOfDraws)
{
	unsigned int i = 0;
	posePercent result;
	result.percent = 0;
	moveit::core::RobotState& robot_state = planning_scene.getCurrentStateNonConst();

	std::pair<double, double> z_range = computeZSamplingRange(robot_state, tree, surface_pose);
	ROS_DEBUG("z_range: %lf, %lf", z_range.first, z_range.second);

	samplingBoundingBox bbox = computeSamplingBoundingBox(tree, z_range);
	ROS_INFO("Sampling Box: \n"
			"x: [%lf, %lf]\n"
			"y: [%lf, %lf]\n"
			"z: [%lf, %lf]", bbox.x_min, bbox.x_max, bbox.y_min, bbox.y_max, bbox.z_min, bbox.z_max);

	int debug_count = 0;
	while (i < numberOfDraws)
	{
		debug_count++;

		posePercent torso_pose_in_surface_frame = sampleTorsoPose(tree, bbox);
//		ROS_INFO_STREAM(torso_pose_in_surface_frame);

		posePercent torso_pose_in_map_frame = transformPosePercentInMap(torso_pose_in_surface_frame, surface_pose);
//		ROS_INFO_STREAM(torso_pose_in_map_frame);

		posePercent base_pose;
		if (!robotInCollision(planning_scene, torso_pose_in_map_frame, tree->getBaseName(), base_pose))
		{
			if (base_pose.percent > result.percent)
				result = base_pose;
			i++;
		}
	}
//	ROS_INFO_STREAM("Sampled Pose: Percent: " << result.percent << "\n" << result.pose);
	ROS_WARN("Debug count: %d", debug_count);

	return result;
}

double InverseCapabilityDrawing::getLinkHeight(const moveit::core::RobotState& robot_state, const std::string& link_name)
{
	Eigen::Affine3d e = robot_state.getGlobalLinkTransform(link_name);
	tf::Transform t;
	tf::transformEigenToTF(e, t);
	return (double)t.getOrigin().z();
}


std::pair<double, double> InverseCapabilityDrawing::computeZSamplingRange(const moveit::core::RobotState& robot_state,
		const InverseCapabilityOcTree* tree,
		const geometry_msgs::PoseStamped& surface_pose)
{
	std::pair<double, double> range;

	// get the current torso height
	double torso_height = getLinkHeight(robot_state, tree->getBaseName());
	ROS_ASSERT(torso_height > 0.795);

	// get the table height
	double table_height = surface_pose.pose.position.z;

	// compute offset between torso height to table height
	double z_offset = torso_height - table_height;
	ROS_INFO("InverseCapabilityDrawing::%s: Torso height: %lf, table height: %lf, resulting z-offset: %lf",
			__func__, torso_height, table_height, z_offset);

	const moveit::core::RobotModelConstPtr robot_model = robot_state.getRobotModel();
	const moveit::core::LinkModel* torso_link = robot_model->getLinkModel(tree->getBaseName());

	const moveit::core::JointModel* torso_joint =  torso_link->getParentJointModel();
	const std::vector<std::string>& variable_names = torso_joint->getVariableNames();
	ROS_ASSERT(variable_names.size() == 1);
	ROS_ASSERT(variable_names[0] == "torso_lift_joint");
	moveit::core::VariableBounds joint_bounds = torso_joint->getVariableBounds(variable_names[0]);

	// get torso joint value
	const double* torso_joint_value = robot_state.getJointPositions(torso_joint->getName());

	double torso_to_max = joint_bounds.max_position_ - *torso_joint_value;
	double torso_to_min = joint_bounds.min_position_ - *torso_joint_value; // negative values
//	ROS_WARN("joint min bound: %lf, torso joint value: %lf", joint_bounds.min_position_, *torso_joint_value);
	ROS_ASSERT(torso_to_min < 0.001);

	double z_max_sample = z_offset + torso_to_max;
	double z_min_sample = z_offset + torso_to_min;
	ROS_ASSERT(z_max_sample > z_min_sample);

	range = std::make_pair(z_min_sample, z_max_sample);

	return range;
}

InverseCapabilityDrawing::samplingBoundingBox InverseCapabilityDrawing::computeSamplingBoundingBox(const InverseCapabilityOcTree* tree,
		const std::pair<double, double>& z_range)
{
	samplingBoundingBox bbox;
	double x_min, x_max, y_min, y_max;
	x_min = HUGE_VAL;
	x_max = -HUGE_VAL;
	y_min = HUGE_VAL;
	y_max = -HUGE_VAL;

	for (InverseCapabilityOcTree::leaf_iterator it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
	{
		if (it.getX() < x_min)
			x_min = it.getX();
		else if (it.getX() > x_max)
			x_max = it.getX();

		if (it.getY() < y_min)
			y_min = it.getY();
		else if (it.getY() > y_max)
			y_max = it.getY();
	}
	bbox.x_min = x_min;
	bbox.x_max = x_max;
	bbox.y_min = y_min;
	bbox.y_max = y_max;
	bbox.z_min = z_range.first;
	bbox.z_max = z_range.second;

	return bbox;
}

InverseCapabilityDrawing::posePercent InverseCapabilityDrawing::sampleTorsoPose(const InverseCapabilityOcTree* tree,
		const samplingBoundingBox& bbox)
{
	double x_range, y_range, z_range;
	x_range = bbox.x_max - bbox.x_min;
	y_range = bbox.y_max - bbox.y_min;
	z_range = bbox.z_max - bbox.z_min;

	double x, y, z;
	bool valid = false;

	InverseCapability inv_cap;
	while (!valid)
	{
		x = x_range * drand48() + bbox.x_min;
		y = y_range * drand48() + bbox.y_min;
		z = z_range * drand48() + bbox.z_min;

		// check if robot position is inside polygon, meaning invalid position
		// if return value of pointInPolygon is odd then position is in polygon
		// NOT NEEDED since we look if inverse capabilities possesses thetas, if not it is not a valid position

		inv_cap = tree->getNodeInverseCapability(x, y, z);
		// if size is != 0 means that there exists at least one <tetha, percent> pair
		if (inv_cap.getThetasPercent().size() != 0)
			valid = true;
	}

	unsigned int index;
	// generate random number between 0 and size
	index = rand() % inv_cap.getThetasPercent().size();
	// copy thetas into vector to access them by operator[]
	std::vector<double> thetas;
	std::map<double, double>::const_iterator it;
	for ( it = inv_cap.getThetasPercent().begin(); it != inv_cap.getThetasPercent().end(); it++ )
	    thetas.push_back(it->first);

	ROS_ASSERT(index < thetas.size());
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "torso_pose_in_surface_frame";
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = z;
	tf::Quaternion q;
	q.setRPY(0, 0, thetas[index]);
	tf::quaternionTFToMsg(q, pose.pose.orientation);
	double percent = inv_cap.getThetaPercent(thetas[index]);

	posePercent p;
	p.pose = pose;
	p.percent = percent;

	return p;
}

InverseCapabilityDrawing::posePercent InverseCapabilityDrawing::transformPosePercentInMap(
		const posePercent& torso_pose_in_surface, const geometry_msgs::PoseStamped& surface_pose_in_map)
{
	tf::Pose robot_torso_in_surface_frame;
	tf::poseMsgToTF(torso_pose_in_surface.pose.pose, robot_torso_in_surface_frame);

	tf::Pose surface_pose_in_map_frame;
	tf::poseMsgToTF(surface_pose_in_map.pose, surface_pose_in_map_frame);

	tf::Pose robot_torso_in_map_frame;
//	robot_torso_in_map_frame = robot_torso_in_surface_frame * surface_pose_in_map_frame;
	// from: robot_torso_in_surface_frame to surface_pose_in_map_frame || read from right to left
	robot_torso_in_map_frame = surface_pose_in_map_frame * robot_torso_in_surface_frame;

	posePercent ret;
	ret.pose.header.frame_id = surface_pose_in_map.header.frame_id;
	tf::poseTFToMsg(robot_torso_in_map_frame, ret.pose.pose);
	ret.percent = torso_pose_in_surface.percent;

	return ret;
}

bool InverseCapabilityDrawing::robotInCollision(planning_scene::PlanningScene& planning_scene,
		const posePercent& sampled_pose, const std::string& base_name, posePercent& base_pose)
{
	moveit::core::RobotState new_robot_state = planning_scene.getCurrentStateNonConst();

	// compute new torso height
	double old_torso_height = getLinkHeight(new_robot_state, base_name);
	double new_torso_height = sampled_pose.pose.pose.position.z;
	double torso_old_new_offset = new_torso_height - old_torso_height;
//	ROS_WARN("old_torso_height: %lf, new_torso_height: %lf, torso_old_new_offset: %lf", old_torso_height, new_torso_height, torso_old_new_offset);

	// set new torso height
	const moveit::core::RobotModelConstPtr robot_model = new_robot_state.getRobotModel();
	const moveit::core::LinkModel* torso_link = robot_model->getLinkModel(base_name);
	const moveit::core::JointModel* torso_joint =  torso_link->getParentJointModel();
	double torso_joint_value = new_robot_state.getVariablePosition(torso_joint->getName());
	new_robot_state.setVariablePosition(torso_joint->getName(), torso_joint_value + torso_old_new_offset);

	// look up torso footprint transform (result expected: x = 0.05, y = 0.0, z = dont care, qx, qy, qz = 0, qw = 1)
	ROS_ASSERT(torso_link->getName() == base_name);
	const Eigen::Affine3d& torso_trans = new_robot_state.getGlobalLinkTransform(torso_link->getName());
	const Eigen::Affine3d& base_trans = new_robot_state.getGlobalLinkTransform(torso_link->getParentLinkModel()->getName());
	// convert eigen into tf
	tf::Pose torso_transform, base_transform, torso_base_transform;
	tf::poseEigenToTF(torso_trans, torso_transform);
	tf::poseEigenToTF(base_trans, base_transform);
	// from torso to base transform
	torso_base_transform = torso_transform.inverseTimes(base_transform);
//	ROS_INFO("torso base transform: x: %lf, y: %lf", torso_base_transform.getOrigin().getX(), torso_base_transform.getOrigin().getY());
	// FIXME: even if assertions are equal they return unequal
//	ROS_ASSERT(torso_base_transform.getOrigin().getX() == 0.05);
//	ROS_ASSERT(torso_base_transform.getOrigin().getY() == 0.0);

	geometry_msgs::Pose2D base;
	base.x = sampled_pose.pose.pose.position.x + torso_base_transform.getOrigin().getX();
	base.y = sampled_pose.pose.pose.position.y + torso_base_transform.getOrigin().getY();
//	ROS_WARN("sampled pose: [%lf, %lf], base pose: [%lf, %lf]", sampled_pose.pose.pose.position.x, sampled_pose.pose.pose.position.y, base.x, base.y);
	tf::Quaternion q;
	tf::quaternionMsgToTF(sampled_pose.pose.pose.orientation, q);
	base.theta = tf::getYaw(q);

	// full collision check, check if robot is in collision with polygon using base_link as reference frame
	new_robot_state.setVariablePosition("world_joint/x", base.x);
	new_robot_state.setVariablePosition("world_joint/y", base.y);
	new_robot_state.setVariablePosition("world_joint/theta", base.theta);

	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	collision_result.clear();
	planning_scene.checkCollision(collision_request, collision_result, new_robot_state);

	// convert 2D pose into posePercent
	base_pose = sampled_pose;
	base_pose.pose.pose.position.x = base.x;
	base_pose.pose.pose.position.y = base.y;
	tf::Quaternion orientation = tf::createQuaternionFromYaw(base.theta);
	tf::quaternionTFToMsg(orientation, base_pose.pose.pose.orientation);
	base_pose.percent = sampled_pose.percent;

	bool inCollision = false;
	if (collision_result.collision)
	{
		inCollision = true;
//		ROS_ERROR_STREAM(base_pose);
	}
//	ROS_INFO_STREAM(base_pose);

	return inCollision;
}

std::ostream& operator<<(std::ostream& out, const InverseCapabilityDrawing::posePercent& pose)
{
	return out << "Percent: " << pose.percent << "\n" << pose.pose;
}
