#pragma once

#include "ExpIBSSim.h"

class ExpIDIBS : public ExpIBSSim {
public:
    ExpIDIBS(IbsSampleScheme::SampleScheme sampleScheme, bool onePass, std::string expId) 
        : ExpIBSSim(sampleScheme,onePass,expId) {};
    virtual ~ExpIDIBS() {};

    virtual std::vector<std::shared_ptr<IBS>> compute(std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>> sets);
    virtual std::string getExpPath();
    virtual void output(std::vector<std::shared_ptr<IBS>> ibses, std::string path);
};

