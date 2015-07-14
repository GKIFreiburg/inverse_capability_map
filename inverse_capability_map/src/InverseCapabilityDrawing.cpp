#include "inverse_capability_map/InverseCapabilityDrawing.h"
#include <tf_conversions/tf_eigen.h>

InverseCapabilityDrawing::InverseCapabilityDrawing(long int seed)
{
    if(seed == 0)
        srand48(time(NULL));
    else
        srand48(seed);
}

std::pair<double, double> InverseCapabilityDrawing::drawBestOfXSamples(const planning_scene::PlanningScene& planning_scene,
			const InverseCapabilityOcTree* tree, const std::string& surface_id, const geometry_msgs::Pose& surface_pose, unsigned int numberOfDraws)
{
	std::pair<double, double> ret;

	return ret;
}


std::pair<double, double> InverseCapabilityDrawing::computeZSamplingRange(const moveit::core::RobotState& robot_state,
		const InverseCapabilityOcTree* tree,
		const moveit_msgs::CollisionObject& table)
{
	std::pair<double, double> range;

	// get the current torso height
	Eigen::Affine3d e = robot_state.getGlobalLinkTransform(tree->getBaseName());
	tf::Transform t;
	tf::transformEigenToTF(e, t);
	double torso_height = t.getOrigin().z();

	// get the table height
	double table_height;
	if (table.mesh_poses.size() != 0)
		table_height = table.mesh_poses[0].position.z;
	else if (table.primitive_poses.size() != 0)
		table_height = table.primitive_poses[0].position.z;
	else
	{
		ROS_ERROR("InverseCapabilityDrawing::%s: collision object %s does not have a position defined!", __func__, table.id.c_str());
		return range;
	}

	// compute offset between torso height to table height
	double z_offset = torso_height - table_height;
	ROS_INFO("InverseCapabilityDrawing::%s: Torso height: %lf, table height: %lf, resulting z-offset: %lf",
			__func__, torso_height, table_height, z_offset);

	const moveit::core::RobotModelConstPtr robot_model = robot_state.getRobotModel();
	const moveit::core::LinkModel* torso_link = robot_model->getLinkModel(tree->getBaseName());

	const moveit::core::JointModel* torso_joint =  torso_link->getParentJointModel();
	const std::vector<std::string>& variable_names = torso_joint->getVariableNames();
	ROS_ASSERT(variable_names.size() == 1);
	moveit::core::VariableBounds joint_bounds = torso_joint->getVariableBounds(variable_names[0]);

	// get torso joint value
	const double* torso_joint_value = robot_state.getJointPositions(torso_joint->getName());

	double torso_to_max = joint_bounds.max_position_ - *torso_joint_value;
	double torso_to_min = joint_bounds.min_position_ - *torso_joint_value; // negative values
	ROS_ASSERT(torso_to_min < 0);

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

InverseCapabilityDrawing::posePercent InverseCapabilityDrawing::sampleTorsoPositions(const InverseCapabilityOcTree* tree,
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

		inv_cap = tree->getNodeInverseCapability(x, y, z);
		// if size is != 0 means that there exists at least one <tetha, percent> pair
		if (inv_cap.getThetasPercent().size() != 0)
			valid = true;
	}

	unsigned int index;
	// generate random number between 0 and size
	index = rand() % inv_cap.getThetasPercent().size();
	// copy tethas into vector to access them by operator[]
	std::vector<double> thetas;
	std::map<double, double>::const_iterator it;
	for ( it = inv_cap.getThetasPercent().begin(); it != inv_cap.getThetasPercent().end(); it++ )
	    thetas.push_back(it->first);

	ROS_ASSERT(index < thetas.size());
	geometry_msgs::Pose pose;
	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = z;
	tf::Quaternion q;
	q.setRPY(0, 0, thetas[index]);
	tf::quaternionTFToMsg(q, pose.orientation);
	double percent = inv_cap.getThetaPercent(thetas[index]);

	posePercent p;
	p.pose = pose;
	p.percent = percent;

	return p;
}





