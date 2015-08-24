#include "inverse_capability_map/InverseCapabilitySampling.h"
#include <tf_conversions/tf_eigen.h>

#include <symbolic_planning_utils/load_tables.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inverse_capability_sampling");

	ros::NodeHandle nh;
	ros::ServiceClient srvPlanningScene;
	srvPlanningScene = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

	// Set up planning scene
	planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
	planning_scene::PlanningScenePtr planning_scene = psm->getPlanningScene();

//	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//	planning_scene::PlanningScene planning_scene(kinematic_model);

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

	// load table locations from file
	ros::NodeHandle nhPriv("~");
	std::string table_name;
	if(!nhPriv.getParam("poly_name", table_name)) {
		ROS_ERROR("Could not find param poly_name in namespace %s", nhPriv.getNamespace().c_str());
		return 1;
	}

	std::string path_inv_cap;
	if(!nhPriv.getParam("path_inv_cap", path_inv_cap)) {
		ROS_ERROR("Could not find param %s in namespace %s, indicating path to inverse capability map.", table_name.c_str(), nhPriv.getNamespace().c_str());
		return 1;
	}

	int numberOfDraws;
	nhPriv.param("num_draws", numberOfDraws, 100);

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

	ros::Publisher pub_ps = nh.advertise<moveit_msgs::PlanningScene>("virtual_planning_scene", 1, true);
	moveit_msgs::PlanningScene ps_msg;
	planning_scene->getPlanningSceneMsg(ps_msg);
	pub_ps.publish(ps_msg);

	/****************************************************
	 * Testing inverse update using mahalanobis distance
	 ***************************************************/
	InverseCapabilitySampling::PosePercent base_pose = sampled_pose;

	Eigen::Matrix4d covariance;
	// declare standard deviations
	double dev_x, dev_y, dev_z, dev_theta;
	dev_x = 0.5;
	dev_y = 0.5;
	dev_z = 0.5;
	dev_theta = M_PI/4;

	double cov_x, cov_y, cov_z, cov_theta;
	cov_x = dev_x * dev_x;
	cov_y = dev_y * dev_y;
	cov_z = dev_z * dev_z;
	cov_theta = dev_theta * dev_theta;

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
	samples["test"] = sampled_pose.pose;

	sampled_pose.pose.pose.position.x = base_pose.pose.pose.position.x + 0.2;
	sampled_pose.pose.pose.position.y = base_pose.pose.pose.position.y + 0.4;
	yaw = tf::getYaw(base_pose.pose.pose.orientation);
	yaw += M_PI/128;
	quaternion = tf::createQuaternionFromYaw(yaw);
	tf::quaternionTFToMsg(quaternion, sampled_pose.pose.pose.orientation);
	samples["test1"] = sampled_pose.pose;

	double dist = InverseCapabilitySampling::computeMahalanobisDistance(base_pose, samples, covariance);
	ROS_WARN("mahalanobis distance: %lf", dist);

	ros::spin();

}
