#include "Utilities.h"
#include <cmath>

bool utilities::doubleEpsEqual(double a, double b, double epsilon)
{
    return std::abs(b-a) < epsilon;
}

bool utilities::doubleEpsDiff(double a, double b, double epsilon)
{
    return !doubleEpsEqual(a,b,epsilon);
}
