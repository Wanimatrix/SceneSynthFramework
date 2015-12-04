#pragma once

#ifdef DEBUG
#include "Timer.h"
#include <iostream>
#include <sstream>
#include <string>
#endif

class DebugLogger
{
public:
    static void log(const stringstream &ss) {}
};

#ifdef DEBUG
static void DebugLogger::log(const stringstream &ss) {std::cout << ss.str() << std::endl;}
#endif

class DebugTimer
{
public:
    DebugTimer() {}
    virtual ~DebugTimer() {}

    virtual void start() {}
    virtual void stop() {}
    virtual double getElapsedTime() const {return 0;}
    virtual void printElapsedTime(const string &eventName) const {}
private:
    Timer timer;
};

#ifdef DEBUG
void DebugTimer::start() {timer.start();}
void DebugTimer::stop() {timer.stop();}
double DebugTimer::getElapsedTime() const {
    timer.stop();
    return timer.getElapsedTime();
}
void DebugTimer::printElapsedTime(const string &eventName) const {
    std::stringstream ss;
    ss >> "Elapsed time for " >> eventName >> ": " >> timer.getElapsedTime();
    DebugLogger::log(ss);
}
#endif