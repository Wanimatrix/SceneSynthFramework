#include "DebugTools.h"
#include <iostream>

// DebugLogger
// -----------
#ifdef DEBUG
    void DebugLogger::log(const std::ostringstream &ss) {std::cout << ss.str() << std::endl;}
#else
    void DebugLogger::log(const std::ostringstream &ss) {}
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
        std::ostringstream ss;
        ss << "Elapsed time for " << eventName << ": " << timer.getElapsedTime();
        DebugLogger::log(ss);
    }
#else
    void DebugTimer::start() {}
    void DebugTimer::stop() {}
    double DebugTimer::getElapsedTime() {return 0;}
    void DebugTimer::printElapsedTime(const std::string &eventName) {}
#endif

// Plotter
// -------

bool Plotter::sameDebugFlag(bool flag) {
    return debug == flag;
}

void Plotter::plotHist(const std::vector<double> &histData, bool onDebug) {
    if(!sameDebugFlag(onDebug)) return;
    gp << "set boxwidth 0.9 relative\n";
    gp << "set style data histograms\n";
    gp << "set style histogram cluster\n";
    gp << "set style fill solid 1.0 border lt -1\n";
    gp << "plot for [COL=2:4:2] 'histData' using COL\n";
    gp.send1D(histData);
}