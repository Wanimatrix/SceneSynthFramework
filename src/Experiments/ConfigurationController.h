#pragma once

#include "Configuration.h"

class ConfigurationController
{
public:
    static ConfigurationController& getInstance()
    {
        static ConfigurationController instance;
        return instance;
    }

    ConfigurationController(ConfigurationController const&) = delete;
    void operator=(ConfigurationController const&) = delete;

    Configuration& getCurrentConfiguration();
    bool advanceConfiguration();

    void setConfigurations(std::vector<Configuration> configurations);
    void addToAll(std::string itemName, ValueType itemValue);
    void setInAll(std::string itemName, ValueType itemValue);
private:
    ConfigurationController() {}
    std::vector<Configuration> m_configurations;
    int m_currentConfig;
};
