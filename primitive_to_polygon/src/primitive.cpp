#include "primitive_to_polygon/primitive.h"
#include <ros/assert.h>

Rectangle::Rectangle(double width, double height)
	: width_(width), height_(height)
{
}

geometry_msgs::Polygon Rectangle::createPolygon()
{
	geometry_msgs::Polygon polygon;
	polygon.points.resize(4);
	ROS_ASSERT(polygon.points.size() == 4);
	geometry_msgs::Point32 p;
	p.x = -width_ / 2;
	p.y = -height_ / 2;
	polygon.points[0] = p;

	p.x = width_ / 2;
	p.y = -height_ / 2;
	polygon.points[1] = p;

	p.x = width_ / 2;
	p.y = height_ / 2;
	polygon.points[2] = p;

	p.x = -width_ / 2;
	p.y = height_ / 2;
	polygon.points[3] = p;

//	ROS_ASSERT(polygon.points[0] != polygon.points[1]);
	return polygon;
}

Circle::Circle(double radius, unsigned int points)
	: radius_(radius), points_(points)
{
}

geometry_msgs::Polygon Circle::createPolygon()
{
	geometry_msgs::Polygon polygon;
	polygon.points.resize(points_);
	ROS_ASSERT(polygon.points.size() == points_);
	geometry_msgs::Point32 p;

	const double angle = 2 * M_PI / points_;
	for (unsigned int i = 0; i < points_; i++)
	{
		p.x = radius_ * cos(angle * i);
		p.y = radius_ * sin(angle * i);
		polygon.points[i] = p;
	}

	return polygon;
}
