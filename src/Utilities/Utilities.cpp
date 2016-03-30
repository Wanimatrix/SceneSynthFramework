#include "Utilities.h"
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <cmath>

bool utilities::doubleEpsEqual(double a, double b, double epsilon)
{
    return std::abs(b-a) < epsilon;
}

bool utilities::doubleEpsDiff(double a, double b, double epsilon)
{
    return !doubleEpsEqual(a,b,epsilon);
}

void utilities::checkPath(std::string path)
{
    bool expPathOk = false;
    boost::filesystem::path dir(path);
    if(!boost::filesystem::exists(dir)) {
        if(boost::filesystem::create_directories(dir))
            expPathOk = true;
    } else
        expPathOk = true;
}
