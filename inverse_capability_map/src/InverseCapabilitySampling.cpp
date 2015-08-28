#include "inverse_capability_map/InverseCapabilitySampling.h"
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/GetMap.h>
#include <angles/angles.h>

InverseCapabilitySampling* InverseCapabilitySampling::instance = NULL;

InverseCapabilitySampling::InverseCapabilitySampling(long int seed) :
		check_robot_in_map_(false)
{
    if(seed == 0)
        srand48(time(NULL));
    else
        srand48(seed);

    // Fetch 2d map
	ros::NodeHandle nh;
	ros::ServiceClient get_map_client = nh.serviceClient<nav_msgs::GetMap>("/static_map", false);
	if (!get_map_client.exists())
	{
		ROS_ERROR("InverseCapabilitySampling::%s: Could not subscribe to service: %s",
				__func__, get_map_client.getService().c_str());
		check_robot_in_map_ = false;
	}

	nav_msgs::GetMap srv;
	if (!get_map_client.call(srv))
	{
		ROS_ERROR("InverseCapabilitySampling::%s: %s request failed.",
				__func__, get_map_client.getService().c_str());
		check_robot_in_map_ = false;
	}

	check_robot_in_map_ = true;
	map_ = srv.response.map;
}

InverseCapabilitySampling::~InverseCapabilitySampling()
{
}

InverseCapabilitySampling::PosePercent InverseCapabilitySampling::drawBestOfXSamples(planning_scene::PlanningScenePtr& planning_scene,
			const InverseCapabilityOcTree* tree,
			const geometry_msgs::PoseStamped& surface_pose,
			unsigned int numberOfDraws,
			const std::map<std::string, geometry_msgs::PoseStamped>& samples,
			const Eigen::Matrix4d& covariance,
			const double min_percent_of_max,
			bool verbose,
			long int seed)
{
	if (instance == NULL)
		instance = new InverseCapabilitySampling(seed);
	return instance->drawBestOfXSamples_(planning_scene, tree, surface_pose, numberOfDraws, samples, covariance, min_percent_of_max, verbose);
}

InverseCapabilitySampling::PosePercent InverseCapabilitySampling::drawBestOfXSamples_(planning_scene::PlanningScenePtr& planning_scene,
			const InverseCapabilityOcTree* tree,
			const geometry_msgs::PoseStamped& surface_pose,
			unsigned int numberOfDraws,
			const std::map<std::string, geometry_msgs::PoseStamped>& samples,
			const Eigen::Matrix4d& covariance,
			const double min_percent_of_max,
			bool verbose)
{
	unsigned int i = 0;
	PosePercent result;
	result.percent = 0;
	moveit::core::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
	// compute miniumum percent
	const double minimum_percent = tree->getMaximumPercent() * (min_percent_of_max / 100);

	std::pair<double, double> z_range = instance->computeZSamplingRange(robot_state, tree, surface_pose);
	ROS_DEBUG("z_range: %lf, %lf", z_range.first, z_range.second);

	SamplingBoundingBox bbox = instance->computeSamplingBoundingBox(tree, z_range);
	if (verbose)
		ROS_INFO("Sampling Box: \n"
			"x: [%lf, %lf]\n"
			"y: [%lf, %lf]\n"
			"z: [%lf, %lf]", bbox.x_min, bbox.x_max, bbox.y_min, bbox.y_max, bbox.z_min, bbox.z_max);

	// check if we have an octomap, otherwise print msg because collision checks make no sense!
	moveit_msgs::PlanningSceneComponents comp;
	comp.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
	moveit_msgs::PlanningScene octomap_msg;
	planning_scene->getPlanningSceneMsg(octomap_msg, comp);
	// the resolution of the recorder map is 0.02 and the live recorded ones are set to 0.25
	// is a bit hacky, but its only purpose is to inform us if the octomap was not loaded
	if (octomap_msg.world.octomap.octomap.resolution != 0.02 )
		ROS_WARN("InverseCapabilitySampling::%s: No recorded octomap present for collision checks!", __func__);

	int count_draws = 0;
	while (i < numberOfDraws)
	{
		count_draws++;

		PosePercent torso_pose_in_surface_frame = instance->sampleTorsoPose(tree, bbox);

		PosePercent torso_pose_in_map_frame = instance->transformPosePercentInMap(torso_pose_in_surface_frame, surface_pose);

		PosePercent base_pose;
		// if true, robot is in collision and skip that pose + convert sampled pose to base pose!
		// the reason why 3d collision check is done before 2d map check
		if (instance->robotInCollision(planning_scene, torso_pose_in_map_frame, tree->getBaseName(), base_pose))
			continue;

		// if no 2d cost map can be loaded, skip this check
		if (check_robot_in_map_)
		{
			// if true, robot pose is outside = an invalid pose
			if (instance->robotOutsideMap(base_pose))
				continue;
		}

		double mahalanobis_distance = 1.0;
		if (samples.size() != 0)
		{
			// compute mahalanobis distance to each drawn sample, to determine if new drawn pose is close to a sample,
			// if so punish new sample by decreasing percent
			mahalanobis_distance = instance->computeMahalanobisDistance(base_pose, samples, covariance);
			// ROS_INFO("mahalanobis_distance: %lf", mahalanobis_distance);

			// this condition is needed so that it is possible to return an empty sampled pose
			// because there are enough samples drawn - the region is grounded out!
			if (base_pose.percent * mahalanobis_distance < minimum_percent)
			{
				// ROS_INFO("base_pose_percent: %lf *  maha: %lf < minimum_percent: %lf", base_pose.percent, mahalanobis_distance, minimum_percent);
				i++;
				if (i == numberOfDraws && verbose)
					ROS_WARN("InverseCapabilitySampling::%s: grounded out!", __func__);
				continue;
			}
		}

		// pose is fine, check for better pose and increase number of valid poses
		if (base_pose.percent * mahalanobis_distance > result.percent)
			result = base_pose;
		i++;
	}



	if (verbose)
		ROS_INFO("Total number of draws: %d", count_draws);

	return result;
}

double InverseCapabilitySampling::getLinkHeight(const moveit::core::RobotState& robot_state, const std::string& link_name)
{
	Eigen::Affine3d e = robot_state.getGlobalLinkTransform(link_name);
	tf::Transform t;
	tf::transformEigenToTF(e, t);
	return (double)t.getOrigin().z();
}


std::pair<double, double> InverseCapabilitySampling::computeZSamplingRange(const moveit::core::RobotState& robot_state,
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
	ROS_DEBUG("InverseCapabilitySampling::%s: Torso height: %lf, table height: %lf, resulting z-offset: %lf",
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
	if (torso_to_min > 0.001)
	{
		ROS_ERROR("joint min bound: %lf, torso joint value: %lf", joint_bounds.min_position_, *torso_joint_value);
		ROS_ERROR("torso_to_min: joint min bound - torso joint value: %lf", torso_to_min);
	}
	ROS_ASSERT(torso_to_min < 0.001);

	double z_max_sample = z_offset + torso_to_max;
	double z_min_sample = z_offset + torso_to_min;
	ROS_ASSERT(z_max_sample > z_min_sample);

	range = std::make_pair(z_min_sample, z_max_sample);

	return range;
}

InverseCapabilitySampling::SamplingBoundingBox InverseCapabilitySampling::computeSamplingBoundingBox(const InverseCapabilityOcTree* tree,
		const std::pair<double, double>& z_range)
{
	SamplingBoundingBox bbox;
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

InverseCapabilitySampling::PosePercent InverseCapabilitySampling::sampleTorsoPose(const InverseCapabilityOcTree* tree,
		const SamplingBoundingBox& bbox)
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

	PosePercent p;
	p.pose = pose;
	p.percent = percent;

	return p;
}

InverseCapabilitySampling::PosePercent InverseCapabilitySampling::transformPosePercentInMap(
		const PosePercent& torso_pose_in_surface, const geometry_msgs::PoseStamped& surface_pose_in_map)
{
	tf::Pose robot_torso_in_surface_frame;
	tf::poseMsgToTF(torso_pose_in_surface.pose.pose, robot_torso_in_surface_frame);

	tf::Pose surface_pose_in_map_frame;
	tf::poseMsgToTF(surface_pose_in_map.pose, surface_pose_in_map_frame);

	tf::Pose robot_torso_in_map_frame;
//	robot_torso_in_map_frame = robot_torso_in_surface_frame * surface_pose_in_map_frame;
	// from: robot_torso_in_surface_frame to surface_pose_in_map_frame || read from right to left
	robot_torso_in_map_frame = surface_pose_in_map_frame * robot_torso_in_surface_frame;

	PosePercent ret;
	ret.pose.header.frame_id = surface_pose_in_map.header.frame_id;
	tf::poseTFToMsg(robot_torso_in_map_frame, ret.pose.pose);
	ret.percent = torso_pose_in_surface.percent;

	return ret;
}

bool InverseCapabilitySampling::robotInCollision(planning_scene::PlanningScenePtr& planning_scene,
		const PosePercent& sampled_pose, const std::string& base_name, PosePercent& base_pose)
{
	moveit::core::RobotState new_robot_state = planning_scene->getCurrentStateNonConst();

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

	// look up torso base_link transform
	ROS_ASSERT(torso_link->getName() == base_name);
	const Eigen::Affine3d& torso_trans = new_robot_state.getGlobalLinkTransform(torso_link->getName());
	const Eigen::Affine3d& base_trans = new_robot_state.getGlobalLinkTransform(torso_link->getParentLinkModel()->getName());
	// convert eigen into tf
	tf::Pose torso_transform, base_transform, torso_base_transform;
	tf::poseEigenToTF(torso_trans, torso_transform);
	tf::poseEigenToTF(base_trans, base_transform);
	// from torso to base transform
	// (result expected: x = 0.05, y = 0.0, z = dont care, qx, qy, qz = 0, qw = 1)
	torso_base_transform = torso_transform.inverseTimes(base_transform);

	tf::Pose map_torso_pose, map_base_transform;
	// convert sampled pose (from map to torso)
	tf::poseMsgToTF(sampled_pose.pose.pose, map_torso_pose);

	// for matrix multiplication (from right to left)
	// from map_torso to torso_base => from map to base
	map_base_transform = torso_base_transform * map_torso_pose;

	geometry_msgs::Pose map_base_pose;
	tf::poseTFToMsg(map_base_transform, map_base_pose);

	geometry_msgs::Pose2D base;
	// old code: transform added by hand
	//	base.x = sampled_pose.pose.pose.position.x + torso_base_transform.getOrigin().getX();
	//	base.y = sampled_pose.pose.pose.position.y + torso_base_transform.getOrigin().getY();
	base.x = map_base_pose.position.x;
	base.y = map_base_pose.position.y;

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
	planning_scene->checkCollision(collision_request, collision_result, new_robot_state);

	// convert 2D pose into PosePercent
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

bool InverseCapabilitySampling::robotOutsideMap(const PosePercent& base_pose)
{
	// Init cost map
	costmap_2d::Costmap2D cost_map(map_.info.width, map_.info.height, map_.info.resolution,
			map_.info.origin.position.x, map_.info.origin.position.y);

	// map coordinates
	unsigned int mx, my;

	// Convert from world coordinates to map coordinates
    if (!cost_map.worldToMap(base_pose.pose.pose.position.x, base_pose.pose.pose.position.y, mx, my))
    	return true;

    // compute index
    unsigned int index = cost_map.getIndex(mx, my);

    uint8_t value = map_.data[index];

    // check if sample is on an free spot
    if (value == costmap_2d::FREE_SPACE)
    	return false;

	return true;
}

double InverseCapabilitySampling::computeMahalanobisDistance(const PosePercent& base_pose,
		const std::map<std::string, geometry_msgs::PoseStamped>& samples,
		const Eigen::Matrix4d& covariance)
{
	double result = 1.0;
	Eigen::Matrix4d inverse;
	bool invertible;
	double determinant = covariance.determinant();
	// compute inverse of covariance matrix and check if invertible
	covariance.computeInverseAndDetWithCheck(inverse, determinant, invertible);
//	ROS_INFO_STREAM("covariance:\n" << covariance);
//	ROS_INFO_STREAM("inverse:\n" << inverse);

	if (!invertible)
	{
		ROS_ERROR("InverseCapabilitySampling::%s: Covariance matrix is NOT invertible", __func__);
		return 1.0;
	}

	// fill vector: x, y, z, theta (= yaw)
	// set mu
	Eigen::Vector4d mu;
	const geometry_msgs::Pose& base = base_pose.pose.pose;
	// angle in range of 0 to 2pi
	double theta = angles::normalize_angle_positive(tf::getYaw(base_pose.pose.pose.orientation));

	mu << base.position.x, base.position.y, base.position.z, theta;

	std::map<std::string, geometry_msgs::PoseStamped>::const_iterator it;
	for (it = samples.begin(); it != samples.end(); it++)
	{
		const geometry_msgs::Pose& sample = it->second.pose;
		double yaw = angles::normalize_angle_positive(tf::getYaw(sample.orientation));
		Eigen::Vector4d x;
		x << sample.position.x, sample.position.y, sample.position.z, yaw;

		// mahalanobis distance = (x - mu)' * (covariance-Matrix)^(-1) * (x - mu)
		Eigen::Vector4d difference;
		difference = x - mu;
//		ROS_INFO_STREAM(difference);
		double maha_dist = difference.transpose() * inverse * difference;

//		ROS_INFO("dist: %lf", maha_dist);
		if (maha_dist < 1.0)
			result *= maha_dist;
	}

	return result;
}

std::ostream& operator<<(std::ostream& out, const InverseCapabilitySampling::PosePercent& pose)
{
	return out << "Percent: " << pose.percent << "\n" << pose.pose;
}
