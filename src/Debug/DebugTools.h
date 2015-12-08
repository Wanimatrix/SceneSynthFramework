#pragma once

#ifdef PERFTIMER
#include "Timer.h"
#endif

#include <sstream>
#include <string>
#include "gnuplot-iostream.h"

// DebugLogger
// -----------
class DebugLogger
{
public:
    static std::ostringstream ss;
    static void log();
};

// DebugTimer
// ----------
class DebugTimer
{
public:
    DebugTimer() {}
    virtual ~DebugTimer() {}

    virtual void start();
    virtual void stop();
    virtual double getElapsedTime();
    virtual void printElapsedTime(const std::string &eventName);
private:
#ifdef PERFTIMER
    Timer timer;
#endif
};

// Plotter
// -------
class Plotter
{
public:
    static void plotHist(const std::vector<double> &histData, const std::string &plotTitle = std::string(""));
    static void newWindow(const std::string &windowTitle);
    static void newMultiWindow(int width, int height, const std::string &windowTitle = std::string(""), int numGraphs = -1);
private:
    static Gnuplot gp;
    static int windowIdx;
    static int multiWindowIdx;
    static int multiWindowNumGraphs;
    static std::pair<int,int> multiWindowDims;
    static bool unsetMultiplot;

    static void multiWindow(int width, int height, int numGraphs, const std::string &windowTitle);
    static void nextMultiWindow();
    static std::string getMultiWindow();
    static void afterPlot();
    static void setWindowTitle(const std::string &windowTitle);
};