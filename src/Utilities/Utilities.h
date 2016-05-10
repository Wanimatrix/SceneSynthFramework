#include <string>
#include <vector>
#define DEFAULT_EPSILON 0.000001

namespace utilities 
{
    bool doubleEpsEqual(double a, double b, double epsilon = DEFAULT_EPSILON);
    bool doubleEpsDiff(double a, double b, double epsilon = DEFAULT_EPSILON);
    void checkPath(std::string path);

    double uniformDouble(double min, double max);
    double normalDouble(double mean, double variance);
    std::vector<double> uniformDoubleVector(int amount, double min, double max);
    int uniformInt(int min, int max);
    std::vector<int> uniformIntVector(int amount, int min, int max);
}
