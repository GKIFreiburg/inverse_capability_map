 #ifndef INVERSECAPABILITYSAMPLING_H
#define INVERSECAPABILITYSAMPLING_H

#include "inverse_capability_map/InverseCapabilityOcTree.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

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

	// Draw samples out of a 4D (x,y,z,theta) space
	/* Input:
	 * 		- planning_scene: needed for collision checks
	 * 		- tree: inverse surface reachability map
	 * 		- surface_pose: the pose of surface (table) in the map
	 * 		- numberOfDraws: Draw this many valid samples and chose the best one (with hightest percentage)
	 * 		- samples: previously drawn samples, needed to compute mahalanobis distance to build negative update
	 * 				   to spread the samples around the surface
	 * 		- covariance: covariance matrix indicating how two close samples should be punished
	 * 		- min_percent_of_max: Enter a percentage, needed to calculate a reference which is build from the
	 * 		           maximum value in tree and as reference to eliminate new samples
	 * 		- verbose: showing some additional info
	 * 		- seed: to initialize random number generator
	 *
	 *  Return Value:
	 *  	- PosePercent: a geometry_msgs::PoseStamped around the surface with an additional double, representing
	 *  			   the reachability on the surface
	 *  		For the PoseStamped:
	 *  				- x and y correspond to the base of the robot (base_link)
	 *  				- z contains the height from the ground to the torso
	 *  				- the orientation represents the orientation of the torso
	 */
	static PosePercent drawBestOfXSamples(planning_scene::PlanningScenePtr& planning_scene,
			const InverseCapabilityOcTree* tree,
			const geometry_msgs::PoseStamped& surface_pose,
			unsigned int numberOfDraws,
			const std::map<std::string, geometry_msgs::PoseStamped>& samples = std::map<std::string, geometry_msgs::PoseStamped>(),
			const Eigen::Matrix4d& covariance = Eigen::Matrix4d(),
			const double min_percent_of_max = 0.0,
			bool verbose = false,
			long int seed = 0);

	// Compute the (4D - x,y,z, theta) mahalanobis distance between a newly sampled pose and previous samples poses
	static double computeMahalanobisDistance(const PosePercent& base_pose,
			const std::map<std::string, geometry_msgs::PoseStamped>& samples,
			const Eigen::Matrix4d& covariance);

	friend std::ostream& operator<<(std::ostream& out, const PosePercent& pose);

	protected:

	struct SamplingBoundingBox
	{
		double x_min, x_max, y_min, y_max, z_min, z_max;
	};

	PosePercent drawBestOfXSamples_(planning_scene::PlanningScenePtr& planning_scene,
			const InverseCapabilityOcTree* tree,
			const geometry_msgs::PoseStamped& surface_pose,
			unsigned int numberOfDraws,
			const std::map<std::string, geometry_msgs::PoseStamped>& samples = std::map<std::string, geometry_msgs::PoseStamped>(),
			const Eigen::Matrix4d& covariance = Eigen::Matrix4d(),
			const double min_percent_of_max = 0.0,
			bool verbose = false);

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

	// Input: planning_scene for robot transforms, base frame name (torso_lift_link) and sampled_pose
	// Output: Pose Percent with x, y in base frame an z represent torso height from the ground floor
	//			The orientation represents the orientation of the torso frame
	PosePercent transformTorsoFrameIntoBaseFrame(const planning_scene::PlanningScenePtr& planning_scene,
			const std::string& base_name,
			const PosePercent& sampled_pose);

	// First set robot state before performing collision checks
	// Input: planning_scene, torso pose in map frame and torso link name
	// Output: base pose with percentage and convert sampled pose to base frame!
	// returned base_pose is in base_link frame
	// Return true if robot is in collision, else return false
	bool robotInCollision(planning_scene::PlanningScenePtr& planning_scene,
			const PosePercent& sampled_pose);

	// Check if robot pose is outside map, then return true
	// Input: 2d map and the pose
	bool robotOutsideMap(const PosePercent& base_pose);

	static InverseCapabilitySampling* instance;

	// needed by 2d map check (robotOutsideMap)
	nav_msgs::OccupancyGrid map_;
	bool map_checks_;
	bool collision_checks_;


};

#endif // INVERSECAPABILITYSAMPLING_H
