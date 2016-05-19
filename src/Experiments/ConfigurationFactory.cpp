#include "ConfigurationFactory.h"
#include <boost/algorithm/string.hpp>
/* #include <boost/regex.hpp> */
#include <boost/algorithm/string/regex.hpp>
#include <stack>
#include <fstream>

std::vector<Configuration> ConfigurationFactory::readConfigurations(std::string configurationsFile)
{
    std::vector<Configuration> result;

    std::ifstream infile;
    infile.open(configurationsFile);
    Configuration global;
    Configuration current;
    Configuration super;
    bool experiment = false;
    bool subExperiment = false;
    std::stack<bool> nameGiven;
    std::string line;

    while(std::getline(infile, line))
    {
        std::vector<std::string> splitted;
        boost::split_regex(splitted, line, boost::regex("//"));
        line = splitted[0];
        boost::trim(line);
        /* if(line.find("//")!=std::string::npos) continue; */

        if(line=="{") 
        {
            assert(!experiment || !subExperiment);
            if(experiment)
            {
                super = current;
                current = Configuration();
                subExperiment = true;
            }
            else
            {
                global = current;
                experiment = true;
            }
            nameGiven.push(false);
            continue;
        }

        if(line=="}") 
        {
            assert(experiment || subExperiment);
            assert(nameGiven.size() > 0 && nameGiven.top());
            nameGiven.pop();
            if(subExperiment)
            {
                super.addSubConfiguration(current);
                current = super;
                subExperiment = false;
            }
            else
            {
                result.push_back(current);
                current = global;
                experiment = false;
            }
            continue;
        }
        
        splitted.clear();
        boost::split(splitted, line, boost::is_any_of("="));
        if(splitted.size() != 2) continue;
        /* std::transform(splitted[1].begin(),splitted[1].end(),splitted[1].begin(), ::tolower); */
        boost::trim(splitted[0]);
        boost::trim(splitted[1]);
        if(nameGiven.size() > 0 && splitted[0] == "Name") 
        {
            nameGiven.pop();
            nameGiven.push(true);
        }
        if(splitted[0] == "Name" && subExperiment) splitted[0] = "SubName";
        current.set(splitted[0],splitted[1]);
    }

    return result;
}
