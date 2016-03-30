#ifdef EXP

#include <vector>
#include <string>
#include <memory>
#include <assert.h>
#include <cstdlib>
#include "../Input.h"
#include "../Scene.h"
#include "../Object.h"
#include "../AnalysisPhase/IBS.h"
#include "../Display/Display.h"
#include "../AnalysisPhase/IbsGenerator.h"
#include "../Debug/DebugTools.h"
#include "Experiment.h"
#include "ExpIBSSim.h"
#include "ExpGAIBS.h"
#include "ExpIDIBS.h"
#include <boost/algorithm/string.hpp>

#ifdef __CYGWIN__
#define DATA_PATH "../../../../Data/"
#else
#define DATA_PATH "../../Data/"
#endif

#define SCENE_FOLDER_PATH std::string(DATA_PATH)+std::string("SceneDB/")
#define EXP_PATH std::string(DATA_PATH)+std::string("Experiments/")

#define EXPFUNC(nr) (exp ## nr)
//#define str(x) #x

typedef void(*FunctionPointer)();
typedef int ExpFuncIdx;
typedef std::vector<std::string> SceneList;
//#define RUNEXPS(eNbs...) for(int eNb:std::vector<int>({eNbs})) EXPFUNC(eNb);

//namespace experiments
//{

    //std::function<void(std::shared_ptr<Input>, std::string)> run[] = {
        //[] (std::shared_ptr<Input> in, std::string centralObj) {
            //Experiment *IBSSimilarityExperiments = new ExpIBSSim(IbsSampleScheme::SampleScheme::UNIFORM);
            //IBSSimilarityExperiments->run(in,centralObj);
            //delete IBSSimilarityExperiments;
        //},
        //[] (std::shared_ptr<Input> in, std::string centralObj) {
            //Experiment *IBSSimilarityExperiments = new ExpIBSSim(IbsSampleScheme::SampleScheme::IMPORTANCE_DISTANCE);
            //IBSSimilarityExperiments->run(in,centralObj);
            //delete IBSSimilarityExperiments;
        //},
        //[] (std::shared_ptr<Input> in, std::string centralObj) {
            //Experiment *IBSSimilarityExperiments = new ExpIBSSim(IbsSampleScheme::SampleScheme::IMPORTANCE_DISTANCE_HULLAPPROX);
            //IBSSimilarityExperiments->run(in,centralObj);
            //delete IBSSimilarityExperiments;
        //}
    //};

    //auto experiments = std::vector<boost::tuple<std::vector<std::string>,std::string>>{
        //[> EXPERIMENT 0: A plane with blocks on it <]
        //boost::tuple<std::vector<std::string>,std::string>(std::vector<std::string>{"blocksOnPlane"},"Plane"),
        //[> EXPERIMENT 1: A plane with different objects on it <]
        //boost::tuple<std::vector<std::string>,std::string>(std::vector<std::string>{"objsOnPlane"},"Plane"),
        //[> EXPERIMENT 2: Blocks in a line with a plane below the middle blocks <]
        //boost::tuple<std::vector<std::string>,std::string>(std::vector<std::string>{"distToPlaneGround"},"Plane"),
        //[> EXPERIMENT 3: Blocks above each other, with a plane below <]
        //boost::tuple<std::vector<std::string>,std::string>(std::vector<std::string>{"distToPlaneHeight"},"Plane"),
        //[> EXPERIMENT 4: Small blocks in the corner of a big block <]
        //boost::tuple<std::vector<std::string>,std::string>(std::vector<std::string>{"blockEnclosedInEachOtherCorners"},"Plane"),
        //[> EXPERIMENT 5: Small blocks in the middle within big block <]
        //boost::tuple<std::vector<std::string>,std::string>(std::vector<std::string>{"blockEnclosedInEachOtherGroundMiddle"},"Plane"),
        //[> EXPERIMENT 6: Small blocks on ground plane within big block <]
        //boost::tuple<std::vector<std::string>,std::string>(std::vector<std::string>{"blockEnclosedInEachOtherGroundSide"},"Plane"),
        //[> EXPERIMENT 7: Chairs on plane <]
        //boost::tuple<std::vector<std::string>,std::string>(std::vector<std::string>{"chairsOnPlane"},"Plane"),
        //[> EXPERIMENT 8: Chairs near each other <]
        //boost::tuple<std::vector<std::string>,std::string>(std::vector<std::string>{"chairsNear"},"Chair.004"),
        //[> EXPERIMENT 9: Chairs near each other, double in size <]
        //boost::tuple<std::vector<std::string>,std::string>(std::vector<std::string>{"chairsNearDoubleSize"},"Chair.004"),
    //};
//}

typedef struct ParsedInfo {
    IbsSampleScheme::SampleScheme sampleScheme = IbsSampleScheme::SampleScheme::UNIFORM;
    std::string centralObj = "Plane";
    std::vector<std::string> inputScenes;
    std::string expId = "";
    bool onePass = false;
} ParsedInfo;

ParsedInfo parse(int argc, const char* argv[]) {
    ParsedInfo pi;
    bool idSet = false;
    for (int i = 1; i < argc; ++i) {
        std::string flag = std::string(argv[i]);
        if(flag == "-s" || flag == "--sampleMethod") {
            std::string uppercSampleName = argv[++i];
            boost::algorithm::to_upper(uppercSampleName);
            pi.sampleScheme = IbsSampleScheme::getSampleScheme(uppercSampleName);
            DebugLogger::ss << "Chosen sampleScheme: " << IbsSampleScheme::getSampleSchemeName(pi.sampleScheme);
            DebugLogger::log();
        } else if(flag == "--centralObj") {
            pi.centralObj = argv[++i];
        } else if (flag == "--onePass") {
          pi.onePass = true;
        } else {
            if (!idSet){
                pi.expId = argv[i];
                idSet = true;
            } else {
                while(i < argc) 
                    pi.inputScenes.push_back(argv[i++]);
            }
        }
    }
    assert(pi.inputScenes.size() > 0);
    assert(pi.expId != "");
    return pi;
}

std::shared_ptr<Input> loadScenes(std::vector<std::string> scenes) {
    std::vector<std::string> scenePaths;
    std::shared_ptr<Input> input;
    for(std::string s: scenes) 
        scenePaths.push_back(SCENE_FOLDER_PATH + s + std::string(".blend"));
    if(!scenePaths.empty()) 
        input = std::shared_ptr<Input>(new Input(scenePaths));
    return input;
}

int main(int argc, const char* argv[]) {
    std::shared_ptr<Input> input;
    std::vector<std::string> scenePaths;

    std::cout << "Starting experiment..." << std::endl;

    ParsedInfo pi = parse(argc, argv);
    DebugLogger::ss << "Loading scenes ...";
    DebugLogger::log();
    input = loadScenes(pi.inputScenes);
    DebugLogger::ss << "Scenes loaded";
    DebugLogger::log();

    //Experiment *IBSSimilarityExperiments = new ExpIBSSim(pi.sampleScheme,pi.onePass,pi.expId);
    //IBSSimilarityExperiments->run(input,pi.centralObj);
    //delete IBSSimilarityExperiments;
    Experiment *gaExperiment = new ExpGAIBS(pi.sampleScheme,pi.onePass,pi.expId);
    gaExperiment->run(input,pi.centralObj);
    delete gaExperiment;
    //for (int i = 0; i < 10; i++)
    //{
        //Experiment *idExperiment = new ExpIDIBS(pi.sampleScheme,pi.onePass,pi.expId);
        //idExperiment->run(input,pi.centralObj);
        //delete idExperiment;
    //}
}
#endif
