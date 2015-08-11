 #ifndef INVERSECAPABILITYSAMPLING_H
#define INVERSECAPABILITYSAMPLING_H

#include "inverse_capability_map/InverseCapabilityOcTree.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

class InverseCapabilitySampling
{
	public:

	struct PosePercent
	{
		geometry_msgs::PoseStamped pose;
		double percent;
	};

	InverseCapabilitySampling(long int seed = 0);
	~InverseCapabilitySampling();

	static PosePercent drawBestOfXSamples(planning_scene::PlanningScenePtr& planning_scene,
			const InverseCapabilityOcTree* tree,
			const geometry_msgs::PoseStamped& surface_pose,
			unsigned int numberOfDraws,
			long int seed = 0);

	friend std::ostream& operator<<(std::ostream& out, const PosePercent& pose);

	protected:

	struct SamplingBoundingBox
	{
		double x_min, x_max, y_min, y_max, z_min, z_max;
	};

	PosePercent drawBestOfXSamples_(planning_scene::PlanningScenePtr& planning_scene,
			const InverseCapabilityOcTree* tree,
			const geometry_msgs::PoseStamped& surface_pose,
			unsigned int numberOfDraws);

	double getLinkHeight(const moveit::core::RobotState& robot_state, const std::string& link_name);

	// Since the torso lift joint has a certain range only certain z slices of the inverse capability map are of interest
	// The return value defines the sampling range which constitutes of a pair containing the z_min and z_max (in that order!)
	std::pair<double, double> computeZSamplingRange(const moveit::core::RobotState& robot_state,
			const InverseCapabilityOcTree* tree,
			const geometry_msgs::PoseStamped& surface_pose);

	// Compute a bounding box around the inverse capability map with the goal the find more valid samples
	SamplingBoundingBox computeSamplingBoundingBox(const InverseCapabilityOcTree* tree,
			const std::pair<double, double>& z_range);

	// Sample poses and also return the reachability percentage
	PosePercent sampleTorsoPose(const InverseCapabilityOcTree* tree,
			const SamplingBoundingBox& bbox);

	// Input: torso_pose in surface frame and pose of surface in map frame
	// Output: torso_pose in map frame
	PosePercent transformPosePercentInMap(const PosePercent& torso_pose_in_surface,
			const geometry_msgs::PoseStamped& surface_pose_in_map);

	// First set robot state before performing collision checks
	// Input: planning_scene, torso pose in map frame and torso link name
	// Output: base pose in 2D
	// Return true if robot is in collision, else return false
	bool robotInCollision(planning_scene::PlanningScenePtr& planning_scene,
			const PosePercent& sampled_pose, const std::string& base_name,
			PosePercent& base_pose);

	static InverseCapabilitySampling* instance;


};

#endif // INVERSECAPABILITYSAMPLING_H
