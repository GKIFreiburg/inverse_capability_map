#ifndef UTILS
#define UTILS

#include <string>

namespace inverse_capability_map_generator
{

// creates the given directory chain
bool makeDir(const std::string &path, const int filemode);

// verify path if error occurs, exit program
void verifyPath(std::string& path);

}; // namespace

#endif // UTILS
