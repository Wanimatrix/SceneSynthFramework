#pragma once

#ifdef DEBUG
#include "Timer.h"
#endif

#include <sstream>
#include <string>

class DebugLogger
{
public:
    static void log(const std::stringstream &ss);
};

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