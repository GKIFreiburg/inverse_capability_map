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
	Rectangle(double width, double length, double height, double padding = 0);
	~Rectangle();
	geometry_msgs::Polygon createPolygon();

private:
	double width_, length_, height_, padding_;
};

class Circle : public Primitive
{
public:
	Circle(double radius, double height, unsigned int points = 16, double padding = 0);
	~Circle();
	geometry_msgs::Polygon createPolygon();

private:
	double radius_, height_, padding_;
	unsigned int points_;
};
