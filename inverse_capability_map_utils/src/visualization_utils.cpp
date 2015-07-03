#include "inverse_capability_map_utils/visualization_utils.h"

namespace inverse_capability_map_utils
{

void drawArrow2D(visualization_msgs::Marker& marker, const geometry_msgs::Point& start, const geometry_msgs::Point& end,
		const geometry_msgs::Vector3& scale)
{
	marker.type = visualization_msgs::Marker::LINE_LIST;

	marker.color.a = 1.0;
	marker.color.g = 1.0;

	marker.points.push_back(start);
	marker.points.push_back(end);
	// scale.x is the shaft diameter,
	// and scale.y is the head diameter.
	// If scale.z is not zero, it specifies the head length.

	/*					  /|
	 * 					/  |
	 * 				  /	   |  head_radius
	 * 			end	<______|___________________ start
	 *					head_lenght
	 */

	double head_radius = scale.y / 2;
	double head_length = scale.z;
	if (head_length <= 0)
		head_length = hypot(start.x - end.x, start.y - end.y) / 2;

	double head_angle = atan(head_radius / head_length);

	double angle = std::atan2(end.y - start.y, end.x - start.x);

	geometry_msgs::Point arrow;
	double total_angle = angle + head_angle;
	arrow.x = end.x - head_length * cos(total_angle);
	arrow.y = end.y - head_length * sin(total_angle);
	arrow.z = end.z;

	marker.points.push_back(end);
	marker.points.push_back(arrow);

	total_angle = angle - head_angle;
	arrow.x = end.x - head_length * cos(total_angle);
	arrow.y = end.y - head_length * sin(total_angle);

	marker.points.push_back(end);
	marker.points.push_back(arrow);
}

void drawArrow3D(visualization_msgs::Marker& marker, const geometry_msgs::Point& start, const geometry_msgs::Point& end,
		const geometry_msgs::Vector3& scale)
{
	marker.type = visualization_msgs::Marker::ARROW;

	marker.color.a = 1.0;
	marker.color.g = 1.0;
	marker.points.push_back(start);
	marker.points.push_back(end);
}

void setMarkerColor(visualization_msgs::Marker& marker, double value)
{
    if (value < 0.0)
    {
        value = 0.0;
    }
    else if (value > 1.0)
    {
        value = 1.0;
    }

    if (value <= 0.25)
    {
        marker.color.r = 0.0;
        marker.color.g = value * 4.0;
        marker.color.b = 1.0;
    }
    else if (value <= 0.5)
    {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0 - (value - 0.25) * 4.0 ;
    }
    else if (value <= 0.75)
    {
        marker.color.r = (value - 0.5) * 4.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    else
    {
        marker.color.r = 1.0;
        marker.color.g = 1.0 - (value - 0.75) * 4.0;
        marker.color.b = 0.0;
    }
    marker.color.a = 1.0;
}

}; // namespace



