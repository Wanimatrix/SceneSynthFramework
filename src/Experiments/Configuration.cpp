#include "Configuration.h"

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include "../Debug/DebugTools.h"

void Configuration::add(std::string itemName, ValueType itemValue)
{
    assert(!exists(itemName));
    m_data[itemName] = itemValue;
}

void Configuration::addToCurrentSubConfiguration(std::string itemName, ValueType itemValue)
{
    assert(hasSubConfiguration() && !existsInData(itemName, getCurrentSubConfiguration()));
    getCurrentSubConfiguration()[itemName] = itemValue;
}

void Configuration::append(std::string itemName, ValueType itemValue)
{
    assert(exists(itemName));
    m_data[itemName] = m_data[itemName]+itemValue;
}

void Configuration::appendToCurrentSubConfiguration(std::string itemName, ValueType itemValue)
{
    assert(hasSubConfiguration() && existsInData(itemName, getCurrentSubConfiguration()));
    getCurrentSubConfiguration()[itemName] = getCurrentSubConfiguration()[itemName]+itemValue;
}

ValueType Configuration::get(std::string itemName) const
{
    assert((hasSubConfigurations() && existsInData(itemName,getCurrentSubConfiguration())) || exists(itemName));
    if(hasSubConfigurations()) return getCurrentSubConfiguration().at(itemName);
    else return m_data.at(itemName);
}

bool Configuration::hasSubConfigurations()
{
    return m_subConfigurations.size() > 0;
}

std::map<KeyType,ValueType> &Configuration::getCurrentSubConfiguration() const
{
    assert(hasSubConfigurations());
    return m_subConfigurations[m_currentSubConfiguration];
}

std::string Configuration::getName() const
{
    assert(exists("Name"));
    std::string result = m_data.at("Name");
    if(hasSubConfigurations())
    {
        assert(existsInData("Name",getCurrentSubConfiguration()));
        return result + "/" + getCurrentSubConfiguration().at("Name");
    }
    DebugLogger::ss << "The name result: " << result;
    DebugLogger::log();
    return result;
}

void Configuration::set(std::string itemName, ValueType itemValue)
{
    m_data[itemName] = itemValue;
}

void Configuration::setToCurrentSubConfiguration(std::string itemName, ValueType itemValue)
{
    assert(hasSubConfiguration());
    getCurrentSubConfiguration()[itemName] = itemValue;
}

bool Configuration::exists(std::string itemName) const
{
    return (m_data.find(itemName) != m_data.end());
}

bool Configuration::existsInData(std::string itemName, std::map<KeyType,ValueType> data) const
{
    return (data.find(itemName) != data.end());
}

void Configuration::erase(std::string itemName)
{
    assert(exists(itemName));
    m_data.erase(itemName);
}

/* Configuration& Configuration::getInstance() */
/* { */
/*     static Configuration configuration; */
/*     return configuration; */
/* } */

void Configuration::writeToFile(std::string filePath) const
{
    std::ofstream outfile;
    outfile.open(filePath+"/configuration.txt");

    outfile << "Experiment configuration" << std::endl;
    outfile << std::endl;

    for(std::map<KeyType,ValueType>::const_iterator iter = m_data.begin(); iter != m_data.end(); ++iter)
    {
        KeyType k = iter->first;
        ValueType v = iter->second;
        outfile << k << "=" << v << std::endl;
    }

    outfile.close();
}

bool Configuration::advanceSubConfiguration()
{
    m_currentSubConfiguration++;
    if(m_currentSubConfiguration >= m_subConfigurations.size()) return false;

    /* m_data = mergeConfigurationData(m_baseConfigurationData, m_subConfigurations[m_currentSubConfiguration]); */
    return true;
}

std::map<KeyType,ValueType> Configuration::mergeConfigurationData(std::map<KeyType,ValueType> first, std::map<KeyType,ValueType> second)
{

    std::map<KeyType,ValueType> result = first;
    result.insert(second.begin(), second.end());

    return result;
}

/* Configuration Configuration::mergeConfigurationWith(Configuration other) */
/* { */
/*     Configuration config; */
/*     for(std::map<KeyType,ValueType>::const_iterator iter = m_data.begin(); iter != m_data.end(); ++iter) */
/*         config.set(iter->first,iter->second); */
/*     for(std::map<KeyType,ValueType>::const_iterator iter = other.m_data.begin(); iter != other.m_data.end(); ++iter) */
/*         config.set(iter->first,iter->second); */

/*     return config; */
/* } */

/* void Configuration::applyConfiguration(Configuration other) */
/* { */
/*     for(std::map<KeyType,ValueType>::const_iterator iter = other.m_data.begin(); iter != other.m_data.end(); ++iter) */
/*         set(iter->first,iter->second); */
/* } */

void Configuration::addSubConfiguration(Configuration newSubConfiguration)
{
    m_subConfigurations.push_back(newSubConfiguration.m_data);
    if(m_subConfigurations.size() == 1)
    {
        m_currentSubConfiguration = -1;
        m_baseConfigurationData = m_data;
        advanceSubConfiguration();
    }
}

/* void Configuration::readFromFile(std::string filePath) */ 
/* { */
/*     std::ifstream infile; */
/*     infile.open(filePath); */
/*     m_data.clear(); */
/*     std::string line; */

/*     while(std::getline(infile, line)) */
/*     { */
/*         std::vector<std::string> splitted; */
/*         boost::split(splitted, line, boost::regex("//")); */
/*         line = tmp[0]; */
/*         /1* if(line.find("//")!=std::string::npos) continue; *1/ */
        
/*         splitted.clear(); */
/*         boost::split(splitted, line, boost::is_any_of("=")); */
/*         if(splitted.size() != 2) continue; */
/*         /1* std::transform(splitted[1].begin(),splitted[1].end(),splitted[1].begin(), ::tolower); *1/ */
/*         boost::trim(splitted[0]); */
/*         boost::trim(splitted[1]); */
/*         add(splitted[0],splitted[1]); */
/*     } */
/* } */
