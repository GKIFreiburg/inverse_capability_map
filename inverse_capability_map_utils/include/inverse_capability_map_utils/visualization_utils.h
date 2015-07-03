#ifndef VISUALIZATION_UTILS
#define VISUALIZATION_UTILS

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

namespace inverse_capability_map_utils
{

void drawArrow2D(visualization_msgs::Marker& marker, const geometry_msgs::Point& start, const geometry_msgs::Point& end,
		const geometry_msgs::Vector3& scale);

void drawArrow3D(visualization_msgs::Marker& marker, const geometry_msgs::Point& start, const geometry_msgs::Point& end,
		const geometry_msgs::Vector3& scale = geometry_msgs::Vector3());

void setMarkerColor(visualization_msgs::Marker& marker, double value);

}; // namespace

#endif // VISUALIZATION_UTILS
