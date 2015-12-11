#ifdef EXP

#include <vector>
#include <string>
#include <memory>
#include <cstdlib>
#include "../Input.h"
#include "../Scene.h"
#include "../Object.h"
#include "../AnalysisPhase/IBS.h"
#include "../Display/Display.h"
#include "../AnalysisPhase/IbsGenerator.h"

#ifdef __CYGWIN__
#define DATA_PATH "../../../../Data/"
#else
#define DATA_PATH "../../Data/"
#endif

#define SCENE_FOLDER_PATH std::string(DATA_PATH)+std::string("SceneDB/")
#define EXP_PATH std::string(DATA_PATH)+std::string("Experiments/")

#define EXPFUNC(nr) (exp ## nr)
#define str(x) #x

typedef void(*FunctionPointer)();

//#define RUNEXPS(eNbs...) for(int eNb:std::vector<int>({eNbs})) EXPFUNC(eNb);

namespace experiments
{
    std::vector<std::string> scenes = std::vector<std::string>{"blocksOnPlane"};
    

    std::function<void(std::shared_ptr<Input>)> run[] = {


        /*
        EXPERIMENT 0: A plane with blocks on it
        ============
        */
        [] (std::shared_ptr<Input> in) {
            std::string expPath = EXP_PATH+std::string("0/");

            // We only have 1 scene
            Scene s = in->getScenes()[0];

            // Divide objects in 2 sets: 1 with the plane and another with the blocks
            std::vector<std::shared_ptr<Object>> set1;
            std::vector<std::shared_ptr<Object>> set2;
            for(std::shared_ptr<Object> o : s.getObjects()) {
                if(o->getName().find("Plane") != std::string::npos) set2.push_back(o);
                else set1.push_back(o);
            }

            // Sample the objects in the scene
            s.samplePoints();

            // Compute Ibses between two sets
            IbsGenerator ibsGen;
            std::vector<std::shared_ptr<IBS>> ibses = ibsGen.computeIBSBetweenTwoSets(set1, set2);

            // Calculate features (only geometric)
            std::vector<std::shared_ptr<Object>> objects = s.getObjects();
            DebugTimer timer;
            timer.start();
            #pragma omp parallel for
            for(int i = 0; i < ibses.size(); i++) {
                auto ibs = ibses[i];
                ibs->computeGeomFeatures();
                ibs->computeTopoFeatures();
                ibs->plotFeatures(expPath+"IBSFeat_"+ibs->ibsObj->getName()+".png");
                #pragma omp critical
                objects.push_back(ibs->ibsObj);
            }
            timer.stop();
            timer.printElapsedTime(std::string("Feature calculation"));

            // Display the result
            Display::display(objects,expPath+std::string("IbsesBlocksOnPlane.blend"),false);
        }
    };
}

int main(int argc, const char* argv[]) {
    std::shared_ptr<Input> input;
    std::vector<std::string> scenePaths;
    for(std::string s: experiments::scenes) scenePaths.push_back(SCENE_FOLDER_PATH + s + std::string(".blend"));
    if(!scenePaths.empty()) input = std::shared_ptr<Input>(new Input(scenePaths));

    for (int i = 0; i < argc-1; ++i)
    {
        int eNb = strtol(argv[i+1],0,10);
        experiments::run[eNb](input);
    }
}
#endif