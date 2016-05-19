#pragma once

#include <string>
#include <map>
#include <memory>
#include <vector>

typedef std::string ValueType;
typedef std::string KeyType;

class Configuration
{
public:
    virtual void add(std::string itemName, ValueType itemValue);
    virtual void set(std::string itemName, ValueType itemValue);
    virtual ValueType get(std::string itemName) const;
    virtual std::string getName() const;
    virtual bool exists(std::string itemName) const;
    virtual void append(std::string itemName, ValueType itemValue);
    virtual void erase(std::string itemName);

    /* static Configuration& getInstance(); */
    virtual void writeToFile(std::string filePath) const;
    /* virtual void readFromFile(std::string filePath); */
    virtual bool advanceSubConfiguration();
    virtual void addSubConfiguration(Configuration newSubConfiguration);
    /* virtual void applyConfiguration(Configuration other); */
    /* virtual Configuration mergeConfigurationWith(Configuration other); */

private:
    /* Configuration() {} */
    static std::map<KeyType,ValueType> mergeConfigurationData(std::map<KeyType,ValueType> first, std::map<KeyType,ValueType> second);
    virtual bool existsInData(std::string itemName, std::map<KeyType,ValueType> data) const;

    std::map<KeyType,ValueType> m_data;
    std::vector<std::map<KeyType,ValueType>> m_subConfigurations;
    /* Configuration m_baseConfiguration; */
    std::map<KeyType,ValueType> m_baseConfigurationData;
    int m_currentSubConfiguration = 0;
};
