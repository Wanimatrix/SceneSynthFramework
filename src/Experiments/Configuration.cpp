#include "Configuration.h"

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>

void Configuration::add(std::string itemName, ValueType itemValue)
{
    assert(!exists(itemName));
    m_data[itemName] = itemValue;
}

void Configuration::append(std::string itemName, ValueType itemValue)
{
    assert(exists(itemName));
    m_data[itemName] = m_data[itemName]+itemValue;
}

ValueType Configuration::get(std::string itemName)
{
    assert(exists(itemName));
    return m_data.at(itemName);
}

void Configuration::set(std::string itemName, ValueType itemValue)
{
    if(exists(itemName)) m_data[itemName] = itemValue;
    else add(itemName,itemValue);
}

bool Configuration::exists(std::string itemName)
{
    return (m_data.find(itemName) != m_data.end());
}

void Configuration::erase(std::string itemName)
{
    assert(exists(itemName));
    m_data.erase(itemName);
}

Configuration& Configuration::getInstance()
{
    static Configuration configuration;
    return configuration;
}

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



void Configuration::readFromFile(std::string filePath) 
{
    std::ifstream infile;
    infile.open(filePath);
    m_data.clear();
    std::string line;

    while(std::getline(infile, line))
    {
        if(line.find("//")!=std::string::npos) continue;
        
        std::vector<std::string> splitted;
        boost::split(splitted, line, boost::is_any_of("="));
        if(splitted.size() != 2) continue;
        /* std::transform(splitted[1].begin(),splitted[1].end(),splitted[1].begin(), ::tolower); */
        boost::trim(splitted[0]);
        boost::trim(splitted[1]);
        add(splitted[0],splitted[1]);
    }
}
