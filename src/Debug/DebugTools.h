#pragma once

#ifdef DEBUG
#include "Timer.h"
#endif

#include <sstream>
#include <string>

// DebugLogger
// -----------
class DebugLogger
{
public:
    static void log(const std::ostringstream &ss);
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
#ifdef DEBUG
    Timer timer;
#endif
};

// DebugPlotter
// ----------
class DebugPlotter
{
public:
    DebugPlotter() {}
    virtual ~DebugPlotter() {}

    virtual void plotHist(std::vector<double> histogram);
private:
#ifdef DEBUG
    Timer timer;
#endif
};