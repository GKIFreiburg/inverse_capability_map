#include <inverse_capability_map_utils/polygon_utils.h>
#include <ros/console.h>
#include <fstream>
#include <iostream>

namespace inverse_capability_map_utils
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

}; // namespace
