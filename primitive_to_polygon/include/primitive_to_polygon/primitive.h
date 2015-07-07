#include <geometry_msgs/Polygon.h>

class Primitive
{
public:
//	virtual Primitive() = 0;
//	virtual ~Primitive() = 0;

	// create a polygon which center is at origin
	virtual geometry_msgs::Polygon createPolygon() = 0;
};

class Rectangle : public Primitive
{
public:
	Rectangle(double width, double height);
	~Rectangle();
	geometry_msgs::Polygon createPolygon();

private:
	double width_, height_;
};

class Circle : public Primitive
{
public:
	Circle(double radius, unsigned int points = 16);
	~Circle();
	geometry_msgs::Polygon createPolygon();

private:
	double radius_;
	unsigned int points_;
};
