#include "DebugTools.h"
#include <iostream>
#include <assert.h>

// DebugLogger
// -----------
std::ostringstream DebugLogger::ss;

#ifdef DEBUG
    void DebugLogger::log() {
        std::cout << DebugLogger::ss.str() << std::endl;
        DebugLogger::ss.str("");
    }
#else
    void DebugLogger::log() {}
#endif


// DebugTimer
// ----------
#ifdef PERFTIMER
    void DebugTimer::start() {timer.start();}
    void DebugTimer::stop() {timer.stop();}
    double DebugTimer::getElapsedTime() {
        timer.stop();
        return timer.getElapsedTime();
    }
    void DebugTimer::printElapsedTime(const std::string &eventName) {
        DebugLogger::ss << "Elapsed time for " << eventName << ": " << timer.getElapsedTime();
        DebugLogger::log();
    }
#else
    void DebugTimer::start() {}
    void DebugTimer::stop() {}
    double DebugTimer::getElapsedTime() {return 0;}
    void DebugTimer::printElapsedTime(const std::string &eventName) {}
#endif

// Plotter
// -------

Gnuplot Plotter::gp;
int Plotter::windowIdx = -1;
int Plotter::multiWindowIdx = -1;
int Plotter::multiWindowNumGraphs = -1;
std::pair<int,int> Plotter::multiWindowDims;
bool Plotter::unsetMultiplot = false;

void Plotter::plotHist(const std::vector<double> &histData, const std::string &plotTitle) {
    assert(windowIdx != -1);
    // gp << "set boxwidth 0.9 relative\n";
    // gp << "set style data histograms\n";
    // gp << "set style histogram cluster\n";
    gp << "set style fill solid 1.0 border\n";
    gp << "plot '-' with histogram ";
    if(plotTitle != std::string("")) gp << "title \"" << plotTitle << "\"";
    gp << getMultiWindow() << "\n";
    gp.send1d(boost::make_tuple(histData));

    afterPlot();
}

void Plotter::multiWindow(int width, int height, int numGraphs, const std::string &windowTitle) {
    gp << "set multiplot layout " << height << "," << width << " rowsfirst ";
    if(windowTitle != std::string("")) gp << "title \"{/:Bold=15 " << windowTitle << "}\"";
    gp << "\n";
    gp << "set tics font \", 10\"\n";
    multiWindowDims.first = width;
    multiWindowDims.second = height;
    multiWindowIdx = 1;
    multiWindowNumGraphs = numGraphs;
}

std::string Plotter::getMultiWindow() {
    std::string result;
    if(multiWindowIdx != -1) {
        std::ostringstream ss;
        ss << std::string("lt ") << multiWindowIdx;
        result = ss.str();
        Plotter::nextMultiWindow();
    }
    else result = std::string("");
    return result;
}

void Plotter::nextMultiWindow() {
    assert(multiWindowIdx != -1);
    multiWindowIdx++;
    if(multiWindowIdx > multiWindowDims.first * multiWindowDims.second 
            || (multiWindowNumGraphs > -1 && multiWindowIdx > multiWindowNumGraphs)) {
        multiWindowIdx = -1;
        multiWindowNumGraphs = -1;
        unsetMultiplot = true;
    }
}

void Plotter::afterPlot() {
    if(unsetMultiplot) {
        std::cout << unsetMultiplot << std::endl;
        gp << "unset multiplot\n";
        gp << "reset" << std::endl;
        //gp << "show out" << std::endl;
        unsetMultiplot = false;
    }
}

void Plotter::setWindowTitle(const std::string &windowTitle) {
     gp << "set term png title \"{/:Bold=15 " << windowTitle << "}\"";
}

void Plotter::newWindow(const std::string &windowTitle, const std::string &savePath) {
    gp << "set term png\n";
    ++windowIdx;
    if(savePath == std::string(""))
        gp << "set output '| display png:-'\n";
    else
        gp << "set output '" << savePath << "'\n";
    gp << "set key font \",10\"\n";
    //gp << "set term png " <<  << "\n";
    //gp << "set term png enhanced font \"arial,15\"\n";
    if(windowTitle != std::string("")) setWindowTitle(windowTitle);
}

void Plotter::newMultiWindow(int width, int height, const std::string &windowTitle, const std::string &savePath, int numGraphs) {
    newWindow(std::string(""),savePath);
    multiWindow(width, height, numGraphs, windowTitle);
}