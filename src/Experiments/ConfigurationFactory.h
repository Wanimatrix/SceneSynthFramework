#pragma once

#include "Configuration.h"
#include <string>

/* typedef struct SubExpConfig */
/* { */
/*     std::string name; */
/*     std::vector<std::pair<std::string,std::string>> config; */
/* } SubExpConfig; */

/* typedef struct ExpConfig */
/* { */
/*     std::string name; */
/*     std::vector<std::pair<std::string,std::string>> config; */
/*     std::vector<SubExpConfig> subExps; */
/* } ExpConfig; */

class ConfigurationFactory
{
public:
    /* ExpConfController(Configuration &conf) : m_config(config) {} */
    /* virtual ~ExpConfController() {} */

    /* virtual void readExperiment(std::ifsteam &infile); */
    /* virtual void startNextExperiment(); */

    static std::vector<Configuration> readConfigurations(std::string configFile);
private:
    /* virtual Configuration m_config; */

    /* virtual std::map<std::string,ExpConfig> m_experiments; */
    /* virtual std::string m_currentExperiment; */
    /* virtual std::string m_currentSubExperiment; */
};
