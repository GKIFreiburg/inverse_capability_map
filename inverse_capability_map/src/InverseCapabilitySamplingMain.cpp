#include "inverse_capability_map/InverseCapabilitySampling.h"
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseArray.h>

#include <symbolic_planning_utils/load_tables.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inverse_capability_sampling");

	ros::NodeHandle nh;

	/************************************************************************************************************************************************
	 * Set up planning scene
	 ***********************************************************************************************************************************************/
	ros::ServiceClient srvPlanningScene;
	srvPlanningScene = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
	planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
	planning_scene::PlanningScenePtr planning_scene = psm->getPlanningScene();

	ROS_INFO("Waiting for %s service.", "get_planning_scene");
	srvPlanningScene.waitForExistence();

	moveit_msgs::GetPlanningScene::Request request;
	moveit_msgs::GetPlanningScene::Response response;
	request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
		moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
		moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
		moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
		moveit_msgs::PlanningSceneComponents::OBJECT_COLORS |
		moveit_msgs::PlanningSceneComponents::OCTOMAP |
		moveit_msgs::PlanningSceneComponents::ROBOT_STATE |
		moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS |
		moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;

	if (!srvPlanningScene.call(request, response)) {
		ROS_ERROR("Planning scene request failed.");
		return 1;
	}

	planning_scene->setPlanningSceneMsg(response.scene);

	/************************************************************************************************************************************************
	 * Set Params
	 ***********************************************************************************************************************************************/
	// load table locations from file
	ros::NodeHandle nhPriv("~");

	int numberOfDraws, numberOfIterations;
	nhPriv.param("numberOfDraws", numberOfDraws, 20);
	nhPriv.param("numberOfIterations", numberOfIterations, 500);
	double dev_x, dev_y, dev_z, dev_theta;
	nhPriv.param("deviation_x", dev_x, 0.5);
	nhPriv.param("deviation_y", dev_y, 0.5);
	nhPriv.param("deviation_z", dev_z, 0.5);
	nhPriv.param("deviation_theta", dev_theta, M_PI / 4);
	double min_percent_of_max;
	nhPriv.param("min_percent_of_max", min_percent_of_max, 30.0);
	double min_torso_position;
	nhPriv.param("min_torso_position", min_torso_position, 0.012);
	bool apply_negative_update;
	nhPriv.param("apply_negative_update", apply_negative_update, true);

	ROS_INFO("numberOfDraws: %d", numberOfDraws);
	ROS_INFO("numberOfIterations: %d", numberOfIterations);
	ROS_INFO("Deviation in x: %lf", dev_x);
	ROS_INFO("Deviation in y: %lf", dev_y);
	ROS_INFO("Deviation in z: %lf", dev_z);
	ROS_INFO("Deviation in theta: %lf", dev_theta);
	ROS_INFO("min_percent_of_max: %lf", min_percent_of_max);
	ROS_INFO("min_torso_position: %lf", min_torso_position);
	ROS_INFO("apply_negative_update: %s", apply_negative_update ? "true" : "false");

	std::string table_name;
	if(!nhPriv.getParam("poly_name", table_name)) {
		ROS_ERROR("Could not find param poly_name in namespace %s", nhPriv.getNamespace().c_str());
		return 1;
	}
	ROS_INFO("Sampling around table: %s", table_name.c_str());

	std::string path_inv_cap;
	if(!nhPriv.getParam("path_inv_cap", path_inv_cap)) {
		ROS_ERROR("Could not find param %s in namespace %s, indicating path to inverse capability map.", table_name.c_str(), nhPriv.getNamespace().c_str());
		return 1;
	}
	std::cout << "\n\n\n";

	/************************************************************************************************************************************************
	 * Load Data
	 ***********************************************************************************************************************************************/
	// load inverseCapTree
	InverseCapabilityOcTree* tree = InverseCapabilityOcTree::readFile(path_inv_cap);

	// load table pose
	std::vector<symbolic_planning_utils::LoadTables::TableLocation> tables;
	if (!symbolic_planning_utils::LoadTables::getTables(tables))
	{
		ROS_ERROR("Could not load tables", __func__);
		return 1;
	}

	geometry_msgs::PoseStamped table_pose;
	bool found = false;
	for (size_t i = 0; i < tables.size(); i++)
	{
		if (tables[i].name == table_name)
		{
			table_pose = tables[i].pose;
			found = true;
		}
	}
	if (found == false)
	{
		ROS_ERROR("Could not find table with name: %s", table_name.c_str());
		return 1;
	}

	/************************************************************************************************************************************************
	 * Sample one time and publish result
	 ***********************************************************************************************************************************************/

	ROS_INFO("Sample one time");
	InverseCapabilitySampling::PosePercent sampled_pose = InverseCapabilitySampling::drawBestOfXSamples(planning_scene, tree, table_pose, numberOfDraws);
	ROS_INFO_STREAM("Sampled Pose: Percent: " << sampled_pose.percent << "\n" << sampled_pose.pose);

	moveit::core::RobotState robot_state = planning_scene->getCurrentState();
	// full collision check, check if robot is in collision with polygon using base_link as reference frame
	robot_state.setVariablePosition("world_joint/x", sampled_pose.pose.pose.position.x);
	robot_state.setVariablePosition("world_joint/y", sampled_pose.pose.pose.position.y);
	tf::Quaternion q;
	tf::quaternionMsgToTF(sampled_pose.pose.pose.orientation, q);
	robot_state.setVariablePosition("world_joint/theta", tf::getYaw(q));
	// set new torso height
	const moveit::core::RobotModelConstPtr robot_model = robot_state.getRobotModel();
	const moveit::core::LinkModel* torso_link = robot_model->getLinkModel(tree->getBaseName());
	const moveit::core::JointModel* torso_joint =  torso_link->getParentJointModel();
	Eigen::Affine3d e = robot_state.getGlobalLinkTransform(tree->getBaseName());
	tf::Transform t;
	tf::transformEigenToTF(e, t);
	double torso_height = t.getOrigin().getZ();
	double torso_joint_value = robot_state.getVariablePosition(torso_joint->getName());
	robot_state.setVariablePosition(torso_joint->getName(), torso_joint_value + sampled_pose.pose.pose.position.z - torso_height);

	planning_scene->setCurrentState(robot_state);

	std::string ps_topic = "virtual_planning_scene";
	ros::Publisher pub_ps = nh.advertise<moveit_msgs::PlanningScene>(ps_topic, 1, true);
	moveit_msgs::PlanningScene ps_msg;
	planning_scene->getPlanningSceneMsg(ps_msg);
	ROS_INFO("Publish sampled robot pose on topic: %s", ps_topic.c_str());
	pub_ps.publish(ps_msg);
	std::cout << "\n\n\n";

	/************************************************************************************************************************************************
	 * Testing mahalanobis distance
	 ***********************************************************************************************************************************************/

	ROS_INFO("Computing Mahalanobis distance with static entries");
	const InverseCapabilitySampling::PosePercent base_pose = sampled_pose;

	Eigen::Matrix4d covariance;
	// declare covariances
	double cov_x, cov_y, cov_z, cov_theta;
	cov_x = dev_x * dev_x;
	cov_y = dev_y * dev_y;
	cov_z = dev_z * dev_z;
	cov_theta = dev_theta * dev_theta;
	// declare covariance matrix
	covariance << cov_x,  0.0  ,  0.0  , 0.0,
			       0.0 , cov_y ,  0.0  , 0.0,
			       0.0 ,  0.0  , cov_z , 0.0,
			       0.0 ,  0.0  ,  0.0  , cov_theta;

	std::map<std::string, geometry_msgs::PoseStamped> samples;
	sampled_pose.pose.pose.position.x = base_pose.pose.pose.position.x + 0.2;
	sampled_pose.pose.pose.position.y = base_pose.pose.pose.position.y + 0.2;
	double yaw = tf::getYaw(base_pose.pose.pose.orientation);
	yaw += M_PI/16;
	tf::Quaternion quaternion = tf::createQuaternionFromYaw(yaw);
	tf::quaternionTFToMsg(quaternion, sampled_pose.pose.pose.orientation);
	samples["sample0"] = sampled_pose.pose;

	sampled_pose.pose.pose.position.x = base_pose.pose.pose.position.x + 0.2;
	sampled_pose.pose.pose.position.y = base_pose.pose.pose.position.y + 0.4;
	yaw = tf::getYaw(base_pose.pose.pose.orientation);
	yaw += M_PI/128;
	quaternion = tf::createQuaternionFromYaw(yaw);
	tf::quaternionTFToMsg(quaternion, sampled_pose.pose.pose.orientation);
	samples["sample1"] = sampled_pose.pose;

	double dist = InverseCapabilitySampling::computeMahalanobisDistance(base_pose, samples, covariance);
	ROS_INFO("Mahalanobis distance: %lf", dist);
	std::cout << "\n\n\n";

	/************************************************************************************************************************************************
	 * Testing inverse update using mahalanobis distance
	 ***********************************************************************************************************************************************/

	// create a lof of samples and publish them via a PoseArray
	std::string poses_topic = "sampled_poses";
	ros::Publisher pub_poses = nh.advertise<geometry_msgs::PoseArray>(poses_topic, 1, true);
	geometry_msgs::PoseArray samples_array;
	samples_array.header = sampled_pose.pose.header;

//	Eigen::Matrix4d covariance;
//	// declare covariances
//	double cov_x, cov_y, cov_z, cov_theta;
//	cov_x = dev_x * dev_x;
//	cov_y = dev_y * dev_y;
//	cov_z = dev_z * dev_z;
//	cov_theta = dev_theta * dev_theta;
//	// declare covariance matrix
//	covariance << cov_x,  0.0  ,  0.0  , 0.0,
//			       0.0 , cov_y ,  0.0  , 0.0,
//			       0.0 ,  0.0  , cov_z , 0.0,
//			       0.0 ,  0.0  ,  0.0  , cov_theta;
//	std::map<std::string, geometry_msgs::PoseStamped> samples;
	samples.clear();

	ROS_INFO("Iterations: %d", numberOfIterations);
	int count_grounded_out = 0;
	for (int i = 0; i < numberOfIterations; i++)
	{
		if (i % (numberOfIterations/10) == 0)
			ROS_WARN("Number of performed iterations: %d", i);
		InverseCapabilitySampling::PosePercent sample = InverseCapabilitySampling::drawBestOfXSamples(
				planning_scene, tree, table_pose, numberOfDraws, samples, covariance, min_percent_of_max, min_torso_position);
		if (sample.percent == 0)
		{
			count_grounded_out++;
			// ROS_INFO("Skip sample, too low percentage - GROUNDED OUT", __func__);
			continue;
		}

		// if apply_negative_update is set to false, no samples are added, so no negative update is computed
		if (apply_negative_update)
		{
			std::stringstream s;
			s << i;
			samples["sample" + s.str()] = sample.pose;
		}
		samples_array.poses.push_back(sample.pose.pose);
	}

	ROS_INFO("Publish samples (PoseArray) to topic: %s", poses_topic.c_str());
	pub_poses.publish(samples_array);
	ROS_INFO("Number of published samples: %lu", samples_array.poses.size());
	ROS_INFO("Number of rejected samples due to grounding out: %d", count_grounded_out);

	ros::spin();
}
