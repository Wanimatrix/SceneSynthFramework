#include "ConfigurationController.h"

#include <assert.h>
#include "../Debug/DebugTools.h"

Configuration& ConfigurationController::getCurrentConfiguration()
{
    assert(m_currentConfig >= 0 && m_currentConfig < m_configurations.size());
    return m_configurations[m_currentConfig];
}

bool ConfigurationController::advanceConfiguration()
{
    assert(m_currentConfig >= 0 && m_currentConfig < m_configurations.size());
    // Go to subExperiment
    if(m_configurations[m_currentConfig].advanceSubConfiguration()) return true;
    // Latest experiment and subExperiment?
    if(m_currentConfig == m_configurations.size()-1) return false;
    // Go to next experiment
    m_currentConfig++;
    return true;
}

void ConfigurationController::setConfigurations(std::vector<Configuration> newConfigurations)
{
    m_currentConfig = 0;
    m_configurations = newConfigurations;
    DebugLogger::ss << "Amount of configurations: " << m_configurations.size() << std::endl;
    DebugLogger::ss << "Current config: " << m_currentConfig;
    DebugLogger::log();
}

void ConfigurationController::addToAll(std::string itemName, ValueType itemValue)
{
    for(Configuration &conf : m_configurations)
    {
        conf.add(itemName,itemValue);
    }
}

void ConfigurationController::setInAll(std::string itemName, ValueType itemValue)
{
    for(Configuration &conf : m_configurations)
    {
        conf.set(itemName,itemValue);
    }
}
