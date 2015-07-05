#ifndef POLYGON_UTILS
#define POLYGON_UTILS

#include <string>
#include <geometry_msgs/Polygon.h>

namespace polygon
{

struct bbox
{
	double xmin, xmax, ymin, ymax;
};

struct center
{
	double x, y;
};

// Create a file at given path and dump the given polygon in yaml format
bool dumpPolygon(const std::string& path, const geometry_msgs::Polygon& polygon);

// Read polygon from file
geometry_msgs::Polygon* loadPolygon(const std::string& path);

bbox computeBoundingBox(const geometry_msgs::Polygon& polygon);

center computeBoundingBoxCenter(const bbox& bbox);

bbox computeCenteredBoundingBox(const geometry_msgs::Polygon& polygon);

// Check if point given by x,y is inside given polygon
// using Jordan curve theorem
// if result is odd, point is inside polygon, else if even, point outside polygon
int pointInPolygon(const geometry_msgs::Polygon& polygon, const double& x, const double& y);


}; // namespace

#endif // POLYGON_UTILS
