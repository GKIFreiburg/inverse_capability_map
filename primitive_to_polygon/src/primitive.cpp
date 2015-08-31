#include "primitive_to_polygon/primitive.h"
#include <ros/assert.h>

Rectangle::Rectangle(double width, double length, double height, double padding)
	: width_(width), length_(length), height_(height), padding_(padding)
{
	if (padding_ > 0.0)
	{
		width_  += 2 * padding_;
		height_ += 2 * padding_;
	}
}

geometry_msgs::Polygon Rectangle::createPolygon()
{
	geometry_msgs::Polygon polygon;
	polygon.points.resize(4);
	ROS_ASSERT(polygon.points.size() == 4);
	geometry_msgs::Point32 p;
	p.z = height_;

	p.x = -width_ / 2;
	p.y = -length_ / 2;
	polygon.points[0] = p;

	p.x = width_ / 2;
	p.y = -length_ / 2;
	polygon.points[1] = p;

	p.x = width_ / 2;
	p.y = length_ / 2;
	polygon.points[2] = p;

	p.x = -width_ / 2;
	p.y = length_ / 2;
	polygon.points[3] = p;

//	ROS_ASSERT(polygon.points[0] != polygon.points[1]);
	return polygon;
}

Circle::Circle(double radius, double height, unsigned int points, double padding)
	: radius_(radius), height_(height), points_(points), padding_(padding)
{
	if (padding_ > 0.0)
		radius_ += padding;
}

geometry_msgs::Polygon Circle::createPolygon()
{
	geometry_msgs::Polygon polygon;
	polygon.points.resize(points_);
	ROS_ASSERT(polygon.points.size() == points_);
	geometry_msgs::Point32 p;
	p.z = height_;

	const double angle = 2 * M_PI / points_;
	for (unsigned int i = 0; i < points_; i++)
	{
		p.x = radius_ * cos(angle * i);
		p.y = radius_ * sin(angle * i);
		polygon.points[i] = p;
	}

	return polygon;
}
