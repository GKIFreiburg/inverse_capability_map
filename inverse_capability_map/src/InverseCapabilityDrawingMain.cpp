#include "inverse_capability_map/InverseCapabilityDrawing.h"
#include <tf_conversions/tf_eigen.h>

#include <symbolic_planning_utils/load_tables.h>
#include <moveit_msgs/GetPlanningScene.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inverse_capability_drawing");

    InverseCapabilityDrawing drawing;

	ros::NodeHandle nh;
	ros::ServiceClient srvPlanningScene;
	srvPlanningScene = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

	// Set up planning scene
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	planning_scene::PlanningScene planning_scene(kinematic_model);

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

	planning_scene.setPlanningSceneMsg(response.scene);

	// load inverseCapTree
	// load table locations from file
	std::string path_name;
	std::string table_name = "table1";
	ros::NodeHandle nhPriv("~");
	if(!nhPriv.getParam(table_name, path_name)) {
		ROS_ERROR("Could not find param %s in namespace %s, indicating path to inverse capability map.", table_name.c_str(), nhPriv.getNamespace().c_str());
		return 1;
	}
	InverseCapabilityOcTree* tree = InverseCapabilityOcTree::readFile(path_name);

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

	unsigned int numberOfDraws = 100;

	InverseCapabilityDrawing::posePercent sampled_pose = drawing.drawBestOfXSamples(planning_scene, tree, table_pose, numberOfDraws);
	ROS_INFO_STREAM("Sampled Pose: Percent: " << sampled_pose.percent << "\n" << sampled_pose.pose);

	ros::shutdown();
}
