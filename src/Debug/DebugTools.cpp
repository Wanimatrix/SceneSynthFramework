#include "DebugTools.h"
#include <iostream>

// DebugLogger
// -----------
#ifdef DEBUG
    void DebugLogger::log(const std::stringstream &ss) {std::cout << ss.str() << std::endl;}
#else
    void DebugLogger::log(const std::stringstream &ss) {}
#endif


// DebugTimer
// ----------
#ifdef DEBUG
    void DebugTimer::start() {timer.start();}
    void DebugTimer::stop() {timer.stop();}
    double DebugTimer::getElapsedTime() {
        timer.stop();
        return timer.getElapsedTime();
    }
    void DebugTimer::printElapsedTime(const std::string &eventName) {
        std::stringstream ss;
        ss << "Elapsed time for " << eventName << ": " << timer.getElapsedTime();
        DebugLogger::log(ss);
    }
#else
    void DebugTimer::start() {}
    void DebugTimer::stop() {}
    double DebugTimer::getElapsedTime() {return 0;}
    void DebugTimer::printElapsedTime(const std::string &eventName) {}
#endif