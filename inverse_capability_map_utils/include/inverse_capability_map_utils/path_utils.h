#ifndef PATH_UTILS
#define PATH_UTILS

#include <string>

namespace inverse_capability_map_utils
{

// creates the given directory chain
bool makeDir(const std::string &path, const int filemode);

// verify path if error occurs, exit program
void verifyPath(std::string& path,  const std::string& ending);

}; // namespace

#endif // PATH_UTILS
