#define DEFAULT_EPSILON 0.000001

namespace utilities 
{
    bool doubleEpsEqual(double a, double b, double epsilon = DEFAULT_EPSILON);
    bool doubleEpsDiff(double a, double b, double epsilon = DEFAULT_EPSILON);
}
