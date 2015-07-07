#include <inverse_capability_map_utils/polygon_utils.h>
#include <ros/console.h>
#include <fstream>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <ros/assert.h>

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

geometry_msgs::Polygon* loadPolygon(const std::string& path)
{
	geometry_msgs::Polygon* result = new geometry_msgs::Polygon;
	std::ifstream file(path.c_str(), std::ios::in);
	if (!file.is_open())
	{
		ROS_ERROR_STREAM("Filestream to " << path << " not open, nothing read.");
		return NULL;
	}
	else
	{
		geometry_msgs::Point32 p;
		while (!file.eof())
		{
			std::string qualifier;
			file >> qualifier;
			// check if lines start with "point_" if not break while loop
			if (qualifier.find("point_") == std::string::npos)
				break;
			file.ignore(10, '[');
			file >> p.x;
			file.ignore(1);
			file >> p.y;
			file.ignore(1);
			file >> p.z;
			file.ignore(1);
			result->points.push_back(p);
		}
		file.close();
	}
//	ROS_INFO_STREAM(*result);
	return result;
}

bbox computeBoundingBox(const geometry_msgs::Polygon& polygon)
{
	bbox bbox;
	bbox.xmin = HUGE_VAL;
	bbox.ymin = HUGE_VAL;
	bbox.xmax = -HUGE_VAL;
	bbox.xmax = -HUGE_VAL;
	for (size_t i = 0; i < polygon.points.size(); i++)
	{
		const geometry_msgs::Point32& p = polygon.points[i];
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

center computeBoundingBoxCenter(const bbox& bbox)
{
	center center;
	center.x = (fabs(bbox.xmax) + fabs(bbox.xmin)) / 2 + bbox.xmin;
	center.y = (fabs(bbox.ymax) + fabs(bbox.ymin)) / 2 + bbox.ymin;

	return center;
}

bbox computeBoundingBoxInOrigin(const geometry_msgs::Polygon& polygon)
{
	bbox bbox = computeBoundingBox(polygon);
	center center = computeBoundingBoxCenter(bbox);

	bbox.xmin = bbox.xmin - center.x;
	bbox.xmax = bbox.xmax - center.x;

	bbox.ymin = bbox.ymin - center.y;
	bbox.ymax = bbox.ymax - center.y;

	return bbox;
}

int pointInPolygon(const geometry_msgs::Polygon& polygon, const double& x, const double& y)
{
	int i, j, c = 0;
	for (i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++)
	{
		if ( ((polygon.points[i].y > y) != (polygon.points[j].y > y)) &&
		 (x < (polygon.points[j].x - polygon.points[i].x) * (y - polygon.points[i].y) / (polygon.points[j].y - polygon.points[i].y) + polygon.points[i].x) )
		{
		   c = !c;
		}
	}
	return c;
}

shape_msgs::Mesh createMeshFromPolygon(const geometry_msgs::Polygon& polygon,
		const double& z_offset, const double& thickness)
{
    ROS_ASSERT(polygon.points.size() > 2);

    shape_msgs::Mesh mesh;
    for (size_t i = 0; i < polygon.points.size(); i++)
    {
    	geometry_msgs::Point p;
    	p.x = polygon.points[i].x;
    	p.y = polygon.points[i].y;
    	p.z = polygon.points[i].z + z_offset;
    	mesh.vertices.push_back(p);
    }

    for(int i = 2; i < mesh.vertices.size(); ++i) {
        // poor man's triangulation: Fan pattern.
        shape_msgs::MeshTriangle tri;
        tri.vertex_indices[0] = 0;
        tri.vertex_indices[1] = i;
        tri.vertex_indices[2] = i - 1;
        mesh.triangles.push_back(tri);
    }

    // Add the under side of the table
    for(size_t i = 0; i < polygon.points.size(); i++) {
        geometry_msgs::Point p;
    	p.x = polygon.points[i].x;
    	p.y = polygon.points[i].y;
    	p.z = polygon.points[i].z + z_offset - thickness;
        mesh.vertices.push_back(p);
    }

    ROS_ASSERT(mesh.vertices.size() == 2 * polygon.points.size());
    // underside tris are the same as top, but with under side verts + inverted vertex order
    for(size_t i = polygon.points.size() + 2; i < mesh.vertices.size(); ++i) {
        // poor man's triangulation: Star pattern.
        shape_msgs::MeshTriangle tri;
        tri.vertex_indices[0] = polygon.points.size();
        tri.vertex_indices[1] = i - 1;
        tri.vertex_indices[2] = i;
        mesh.triangles.push_back(tri);
    }

    // create sides
    for(int i = 0; i < polygon.points.size(); ++i) {
        int first_top = i;
        int next_top = (first_top + 1) % polygon.points.size();
        int first_bottom = first_top + polygon.points.size();
        int next_bottom = next_top + polygon.points.size();
        // make a quad of these
        shape_msgs::MeshTriangle tri_top;
        tri_top.vertex_indices[0] = first_top;
        tri_top.vertex_indices[1] = next_top;
        tri_top.vertex_indices[2] = first_bottom;
        mesh.triangles.push_back(tri_top);
        shape_msgs::MeshTriangle tri_bottom;
        tri_bottom.vertex_indices[0] = first_bottom;
        tri_bottom.vertex_indices[1] = next_top;
        tri_bottom.vertex_indices[2] = next_bottom;
        mesh.triangles.push_back(tri_bottom);
    }

    return mesh;
}

}; // namespace
