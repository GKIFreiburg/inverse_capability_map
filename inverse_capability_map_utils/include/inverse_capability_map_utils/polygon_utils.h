#ifndef POLYGON_UTILS
#define POLYGON_UTILS

#include <string>
#include <geometry_msgs/Polygon.h>

namespace inverse_capability_map_utils
{

// Create a file at given path and dump the given polygon in yaml format
bool dumpPolygon(const std::string& path, const geometry_msgs::Polygon& polygon);

// Read polygon from file
geometry_msgs::Polygon loadPolygon(const std::string& path);

}; // namespace

#endif // POLYGON_UTILS
