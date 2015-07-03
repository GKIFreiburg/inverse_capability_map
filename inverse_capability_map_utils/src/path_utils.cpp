#include "inverse_capability_map_utils/path_utils.h"
#include <cerrno>
#include <sys/stat.h>
#include <ros/console.h>
#include <ros/init.h>
//#include <ros/ros.h>

namespace inverse_capability_map_utils
{

// creates the given directory chain
bool makeDir(const std::string &path, const int filemode)
{
    if(mkdir(path.c_str(), filemode) && errno != EEXIST)
    {
        size_t pos = path.find_last_of("/");
        // check if path is root, if so return false, directory chain could not be created
        if (pos == 0 || pos == std::string::npos)
        {
            return false;
        }
        std::string parentPath = path.substr(0, pos);
        // recursively create directory chain of parent paths
        if (!makeDir(parentPath, filemode))
        {
            return false;
        }
        // parent directory should now exist
        if (mkdir(path.c_str(), filemode))
        {
            return false;
        }
    }
    return true;
}

// verify path if error occurs, exit program
void verifyPath(std::string& path, const std::string& ending)
{
    if (path.find_first_of('/') != 0)
    {
        ROS_ERROR("Error: Path to %s is not absolute! The inverse_capability_map_generator node may not \
                   run in your current working directory, only absolute paths are accepted!", path.c_str());
        ros::shutdown();
        exit(1);
    }

    if (path.find_last_of('/') == path.length() - 1)
    {
        ROS_ERROR("Error: Path to %s does not name a valid file!", path.c_str());
        ros::shutdown();
        exit(1);
    }

    unsigned int pos = path.find_last_of('/');
    std::string path_to_file = path.substr(0, pos);

    if (!makeDir(path_to_file, 0775))
    {
        ROS_ERROR("Error: Could not create directory to %s", path_to_file.c_str());
        ros::shutdown();
        exit(1);
    }

    std::string fileName = path.substr(pos + 1);
    pos = fileName.find_last_of('.');

    // append ending to file name if not already there
    if (pos == std::string::npos || fileName.substr(pos) != ending)
    {
        path += ending;
    }
}

}; // namespace
