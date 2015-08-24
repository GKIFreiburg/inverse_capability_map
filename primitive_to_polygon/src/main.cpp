#include <primitive_to_polygon/primitive.h>
#include <inverse_capability_map_utils/path_utils.h>
#include <inverse_capability_map_utils/polygon_utils.h>
#include <tclap/CmdLine.h>
#include <ros/console.h>
#include <ros/init.h>
#include <geometry_msgs/Polygon.h>

using namespace polygon;

void verifyInput(const std::vector<double>& rectangle, const std::vector<double>& circle)
{
    if (rectangle.size() == 0 && circle.size() == 0)
    {
        ROS_ERROR("Error: No primitive was entered!");
        ros::shutdown();
        exit(1);
    }

    if (rectangle.size() != 2 && rectangle.size() != 0)
    {
        ROS_ERROR("Error: Primitive rectangle was not defined properly. Please enter width and length.");
        ros::shutdown();
        exit(1);
    }

    else if (circle.size() > 2)
    {
        ROS_ERROR("Error: Primitive circle was not defined properly. Please enter radius (optional: number of points).");
        ros::shutdown();
        exit(1);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "primitive_to_polygon");

    // arguments
    TCLAP::CmdLine cmd("Convert the predefined primitive to polygon", ' ', "1.0");

    TCLAP::ValueArg<std::string> pathNameArg("p", "path", "Path and filename of the polygon to be stored.\n\
                                             Example: -p mydir/mysubdir/filename.poly", true, "./polygon.poly", "string");

    std::string msg;
    msg = "Specify the shape of a rectangle, first width in m and then height in m.\n"
    		"Example: -r -2.13 -r 1.02";
    TCLAP::MultiArg<double> rectangleArg("r", "rectanlge", msg, false, "floating point");

    msg = "Specify the shape of a circle. First parameter defines the radius in m, the second is optional and defines the number"
    		"of points used to approximate the circle.\n"
    		"Example: -c 1.5 OR -c 1.5 -c 30";
    TCLAP::MultiArg<double> circleArg("c", "circle", msg, false, "floating point");

    msg = "Specify the padding around the primitive\n"
    		"Example: -d 0.1";
    TCLAP::ValueArg<double> paddingArg("d", "padding", msg, false, 0.1, "floating point");

    cmd.add(rectangleArg);
    cmd.add(circleArg);
    cmd.add(paddingArg);
    cmd.add(pathNameArg);

    // parse arguments with TCLAP
    try
    {
        cmd.parse(argc, argv);
    }
    catch (TCLAP::ArgException &e)  // catch any exceptions
    {
        ROS_ERROR("Error: %s for argument %s", e.error().c_str(), e.argId().c_str());
        ros::shutdown();
        exit(1);
    }

    // verify path name
    std::string pathName = pathNameArg.getValue();
    inverse_capability_map_utils::verifyPath(pathName, ".poly");

    // verify input
    std::vector<double> rectangle = rectangleArg.getValue();
    std::vector<double> circle    = circleArg.getValue();
    verifyInput(rectangle, circle);

    // add padding if necessary
    // padding must be positive since we want to increase the scope of the primitive
	double padding = paddingArg.getValue();
    if (padding < 0.0)
    	padding = 0.0;
    else
    	ROS_INFO("Padding was set to %lf m.", padding);

    // Factory Pattern to create the primitive
    Primitive* primitive;
    if (rectangle.size() != 0)
    {
    	primitive = new Rectangle(rectangle[0], rectangle[1], padding);
    }
    else if (circle.size() != 0)
    {
    	if (circle.size() == 1)
    		primitive = new Circle(circle[0], padding);
    	else // circle size == 2
    		primitive = new Circle(circle[0], circle[1],  padding);
    }

    geometry_msgs::Polygon polygon = primitive->createPolygon();
    if (!dumpPolygon(pathName, polygon))
    	ROS_ERROR("Error: Could not save polygon into file: %s", pathName.c_str());

    ROS_INFO("Done");
    ros::shutdown();
}
