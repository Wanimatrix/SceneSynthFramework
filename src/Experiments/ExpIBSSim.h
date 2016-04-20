#pragma once

#include <vector>
#include <string>
#include <memory>
#include "../AnalysisPhase/IBS.h"
#include "../Object.h"
#include "../Scene.h"
#include "../Input.h"
#include "Experiment.h"

namespace IbsSampleScheme {
    enum SampleScheme {
        UNIFORM, IMPORTANCE_DISTANCE, IMPORTANCE_DISTANCE_HULLAPPROX
    };
    
    std::string getSampleSchemeName(SampleScheme smpl);
    SampleScheme getSampleScheme(std::string smplName);
}

class ExpIBSSim : public Experiment {
public:
    ExpIBSSim(IbsSampleScheme::SampleScheme sampleScheme, bool onePass, std::string expId) : m_sampleScheme(sampleScheme), m_onePass(onePass), m_ID(expId) {};
    virtual ~ExpIBSSim() {};

    virtual std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>> handleInput(std::shared_ptr<Input> in, std::string centralObj);
    virtual std::vector<std::shared_ptr<IBS>> compute(std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>> sets);
    virtual void output(std::vector<std::shared_ptr<IBS>> ibses, std::string path);
    /* virtual std::string getExpPath(); */
protected:
    Scene m_scene;
    IbsSampleScheme::SampleScheme m_sampleScheme;
    std::vector<std::shared_ptr<Object>> m_objects;
    std::string m_ID;
    bool m_onePass;
    virtual void computeFeatures(const std::vector<std::shared_ptr<IBS>> &ibses, bool plot);
    virtual std::vector<std::shared_ptr<IBS>> computeIBSBetweenTwoSets(std::vector<std::shared_ptr<Object>> objs1, std::vector<std::shared_ptr<Object>> objs2);
    virtual std::vector<std::shared_ptr<IBS>> computeIBSBetweenTwoSetsWithOneVoronoi(std::vector<std::shared_ptr<Object>> objs1, std::vector<std::shared_ptr<Object>> objs2);
private:
    void writeSimilarities( std::vector<std::shared_ptr<IBS>> &ibses, const std::string &fileName);
};

