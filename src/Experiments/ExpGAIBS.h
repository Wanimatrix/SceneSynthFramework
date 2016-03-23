#pragma once

#include "ExpIBSSim.h"

class ExpGAIBS : public ExpIBSSim {
public:
    ExpGAIBS(IbsSampleScheme::SampleScheme sampleScheme, bool onePass, std::string expId) 
        : ExpIBSSim(sampleScheme,onePass,expId) {};
    virtual ~ExpGAIBS() {};

    virtual std::vector<std::shared_ptr<IBS>> compute(std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>> sets);
    virtual std::string getExpPath();
    virtual void output(std::vector<std::shared_ptr<IBS>> ibses, std::string path);
};

