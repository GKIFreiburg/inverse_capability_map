#ifndef POLYGON_UTILS
#define POLYGON_UTILS

#include <string>
#include <geometry_msgs/Polygon.h>
#include <shape_msgs/Mesh.h>
#include <moveit_msgs/CollisionObject.h>

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

// Compute the bounding box
bbox computeBoundingBox(const geometry_msgs::Polygon& polygon);

// Compute center of bounding box
center computeBoundingBoxCenter(const bbox& bbox);

// Compute bounding box where the center is at origin (0, 0)
bbox computeBoundingBoxInOrigin(const geometry_msgs::Polygon& polygon);

// Check if point given by x,y is inside given polygon
// using Jordan curve theorem
// if result is odd, point is inside polygon, else if even, point outside polygon
int pointInPolygon(const geometry_msgs::Polygon& polygon, const double& x, const double& y);

// Create Mesh from polygon
shape_msgs::Mesh createMeshFromPolygon(const geometry_msgs::Polygon& polygon,
		const double& z_offset, const double& thickness);

}; // namespace

#endif // POLYGON_UTILS
