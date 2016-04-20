#pragma once

#include <string>
#include <map>
#include <memory>
#include "boost/variant.hpp"

typedef std::string ValueType;
typedef std::string KeyType;

class Configuration
{
public:
    virtual void add(std::string itemName, ValueType itemValue);
    virtual void set(std::string itemName, ValueType itemValue);
    virtual ValueType get(std::string itemName);
    virtual bool exists(std::string itemName);
    virtual void append(std::string itemName, ValueType itemValue);
    virtual void erase(std::string itemName);

    static Configuration& getInstance();
    virtual void writeToFile(std::string filePath) const;
    virtual void readFromFile(std::string filePath);

    Configuration(Configuration const&)  = delete;
    void operator=(Configuration const&) = delete;
private:
    Configuration() {}

    std::map<KeyType,ValueType> m_data;
};
