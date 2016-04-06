#pragma once

#include <memory>
#include <vector>
#include <string>
#include "../Object.h"
#include "../Input.h"
#include "../AnalysisPhase/IBS.h"

#ifdef __CYGWIN__
#define DATA_PATH "../../../../Data/"
#else
#define DATA_PATH "../../Data/"
#endif

#define EXP_PATH std::string(DATA_PATH)+std::string("Experiments")

class Experiment {
public:
    Experiment() {};
    virtual ~Experiment() {};

    virtual void run(std::shared_ptr<Input> in, std::string centralObj) final;
    virtual std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>> handleInput(std::shared_ptr<Input> in, std::string centralObj) = 0;
    virtual std::vector<std::shared_ptr<IBS>> compute(std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>> sets) = 0;
    virtual void output(std::vector<std::shared_ptr<IBS>> ibses, std::string path) = 0;
    virtual std::string getExpPath() = 0;
protected:
    std::string outputPath;
};
