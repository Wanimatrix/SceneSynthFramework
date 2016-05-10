#include "Utilities.h"
#include "../Debug/DebugTools.h"
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <cmath>
#include <random>

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
    DebugLogger::ss << "PATH: " << path;
    DebugLogger::log();
    boost::filesystem::path dir(path);
    boost::filesystem::remove_all(dir);
    if(!boost::filesystem::exists(dir)) {
        if(boost::filesystem::create_directories(dir))
            expPathOk = true;
    } else
    {
        expPathOk = true;
    }
}

double utilities::uniformDouble(double min, double max) 
{
    std::random_device m_rseed;
    std::mt19937 rgen(m_rseed());
    std::uniform_real_distribution<double> ddist(min,max);
    return ddist(rgen);
}

double utilities::normalDouble(double mean, double variance) 
{
    std::random_device m_rseed;
    std::mt19937 rgen(m_rseed());
    std::normal_distribution<double> ddist(mean,variance);
    return ddist(rgen);
}

std::vector<double> utilities::uniformDoubleVector(int amount, double min, double max) 
{
    std::random_device m_rseed;
    std::vector<double> result;
    result.reserve(amount);
    std::mt19937 rgen(m_rseed());
    std::uniform_real_distribution<double> ddist(min,max);
    while(result.size() < amount) {
        result.push_back(ddist(rgen));
    }
    return result;
}

int utilities::uniformInt(int min, int max) 
{
    std::random_device m_rseed;
    std::mt19937 rgen(m_rseed());
    std::uniform_int_distribution<int> ddist(min,max);
    return ddist(rgen);
}

std::vector<int> utilities::uniformIntVector(int amount, int min, int max) 
{
    std::random_device m_rseed;
    std::vector<int> result;
    result.reserve(amount);
    std::mt19937 rgen(m_rseed());
    std::uniform_int_distribution<int> ddist(min,max);
    while(result.size() < amount) {
        result.push_back(ddist(rgen));
    }
    return result;
}
