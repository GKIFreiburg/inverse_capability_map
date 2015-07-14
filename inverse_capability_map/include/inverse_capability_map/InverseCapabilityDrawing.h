#ifndef INVERSECAPABILITYDRAWING_H
#define INVERSECAPABILITYDRAWING_H

#include "inverse_capability_map/InverseCapabilityOcTree.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

class InverseCapabilityDrawing
{
	public:
	InverseCapabilityDrawing(long int seed = 0);
	~InverseCapabilityDrawing();

	std::pair<double, double> drawBestOfXSamples(const planning_scene::PlanningScene& planning_scene,
			const InverseCapabilityOcTree* tree, const std::string& surface_id, const geometry_msgs::Pose& surface_pose, unsigned int numberOfDraws);

	protected:

	struct samplingBoundingBox
	{
		double x_min, x_max, y_min, y_max, z_min, z_max;
	};

	struct posePercent
	{
		geometry_msgs::Pose pose;
		double percent;
	};

	// Since the torso lift joint has a certain range only certain z slices of the inverse capability map are of interest
	// The return value defines the sampling range which constitutes of a pair containing the z_min and z_max (in that order!)
	std::pair<double, double> computeZSamplingRange(const moveit::core::RobotState& robot_state,
			const InverseCapabilityOcTree* tree,
			const moveit_msgs::CollisionObject& table);

	// Compute a bounding box around the inverse capability map with the goal the find more valid samples
	samplingBoundingBox computeSamplingBoundingBox(const InverseCapabilityOcTree* tree,
			const std::pair<double, double>& z_range);

	// Sample 2D poses and also return the reachability percentage
	posePercent sampleTorsoPositions(const InverseCapabilityOcTree* tree,
			const samplingBoundingBox& bbox);








};

#endif // INVERSECAPABILITYDRAWING_H
