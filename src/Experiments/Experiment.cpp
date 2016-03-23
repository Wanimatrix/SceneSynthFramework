#include "Experiment.h"
#include "../Debug/DebugTools.h"
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

void Experiment::checkPath(std::string path){
    bool expPathOk = false;
    boost::filesystem::path dir(path);
    if(!boost::filesystem::exists(dir)) {
        if(boost::filesystem::create_directories(dir))
            expPathOk = true;
    } else
        expPathOk = true;
}

void Experiment::run(std::shared_ptr<Input> in, std::string centralObj) 
{
    DebugLogger::ss << "Running experiment...";
    DebugLogger::log();
    std::ostringstream path;
    path << EXP_PATH << "/" << getExpPath() << "/";
    checkPath(path.str());
    std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>> sets = handleInput(in, centralObj);
    std::vector<std::shared_ptr<IBS>> ibses = compute(sets);
    output(ibses, path.str());
}
