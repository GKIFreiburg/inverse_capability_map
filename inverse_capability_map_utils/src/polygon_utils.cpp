#include <inverse_capability_map_utils/polygon_utils.h>
#include <ros/console.h>
#include <fstream>
#include <iostream>

namespace polygon
{

bool dumpPolygon(const std::string& path, const geometry_msgs::Polygon& polygon)
{
    std::ofstream file(path.c_str(), std::ios_base::out);

    if (!file.is_open())
    {
        ROS_ERROR_STREAM("Filestream to " << path << " not open, nothing written.");
        return false;
    }
    else
    {
    	for (size_t i = 0; i < polygon.points.size(); i++)
    	{
    		file << "point_" << i << ": [" << polygon.points[i].x << ", " << polygon.points[i].y << ", " << polygon.points[i].z << "]\n";
    	}
    	file << std::endl;
        file.close();
    }
    return true;
}

geometry_msgs::Polygon loadPolygon(const std::string& path)
{
	geometry_msgs::Polygon result;
	std::ifstream file(path.c_str(), std::ios::in);
	if (!file.is_open())
	{
		ROS_ERROR_STREAM("Filestream to " << path << " not open, nothing read.");
		return result;
	}
	else
	{
		geometry_msgs::Point32 p;
		std::string qualifier;
		while (!file.eof())
		{
			file.ignore(10, '[');
			file >> p.x;
			file.ignore(1);
			file >> p.y;
			file.ignore(1);
			file >> p.z;
			file.ignore(1);
			result.points.push_back(p);
		}
		file.close();
	}
	ROS_INFO_STREAM(result);
	return result;
}

bbox computeBoundingBox(geometry_msgs::Polygon& polygon)
{
	bbox bbox;
	bbox.xmin = HUGE_VAL;
	bbox.ymin = HUGE_VAL;
	bbox.xmax = -HUGE_VAL;
	bbox.xmax = -HUGE_VAL;

	for (size_t i = 0; i < polygon.points.size(); i++)
	{
		geometry_msgs::Point32& p = polygon.points[i];
		if (p.x < bbox.xmin)
			bbox.xmin = p.x;
		else if (p.x > bbox.xmax)
			bbox.xmax = p.x;
		if (p.y < bbox.ymin)
			bbox.ymin = p.y;
		else if (p.y > bbox.ymax)
			bbox.ymax = p.y;
	}

	return bbox;
}

center computeCenterBoundingBox(const bbox& bbox)
{
	center center;
	center.x = bbox.xmax - bbox.xmin;
	center.y = bbox.ymax - bbox.ymin;

	return center;
}

int pointInPolygon(const geometry_msgs::Polygon& polygon, double x, double y)
{
  int i, j, c = 0;
  for (i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++) {
    if ( ((polygon.points[i].y >= y) != (polygon.points[j].y >= y)) &&
     (x <= (polygon.points[j].x - polygon.points[i].x) * (y - polygon.points[i].y) / (polygon.points[j].y - polygon.points[i].y) + polygon.points[i].x) )
       c = !c;
  }
  return c;
}

}; // namespace
