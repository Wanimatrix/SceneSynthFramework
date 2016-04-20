#include "Experiment.h"
#include "../Utilities/Utilities.h"
#include "../Debug/DebugTools.h"


void Experiment::run(std::shared_ptr<Input> in, std::string centralObj) 
{
    DebugLogger::ss << "Running experiment...";
    DebugLogger::log();
    utilities::checkPath(Configuration::getInstance().get("ExperimentRunPath"));
    utilities::checkPath(Configuration::getInstance().get("ExperimentTmpPath"));
    std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>> sets = handleInput(in, centralObj);
    std::vector<std::shared_ptr<IBS>> ibses = compute(sets);
    output(ibses, Configuration::getInstance().get("ExperimentRunPath"));
}
