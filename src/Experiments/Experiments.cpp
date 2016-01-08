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
//#define str(x) #x

typedef void(*FunctionPointer)();
typedef int ExpFuncIdx;
typedef std::vector<std::string> SceneList;
//#define RUNEXPS(eNbs...) for(int eNb:std::vector<int>({eNbs})) EXPFUNC(eNb);

namespace experiments
{

    std::string expPath = EXP_PATH+std::string("0/");
    
    void writeSimilarities(const std::vector<std::shared_ptr<IBS>> &ibses, const std::string &fileName){
        std::ofstream fs;
        std::vector<boost::tuple<int,int,double>> heatMapData;
        std::vector<std::vector<double>> heatMapMat(ibses.size(), std::vector<double>(ibses.size()));
        std::pair<double,double> minMax({5,1});
        fs.open(expPath+fileName+".txt");
        for(int i = 0; i < ibses.size(); i++) {
            auto ibsI = ibses[i];
            for(int j = i; j < ibses.size(); j++) {
                auto ibsJ = ibses[j];
                double sim = ibsI->getSimilarity(*ibsJ);
                fs << ibsI->ibsObj->getName() << " vs " << ibsJ->ibsObj->getName() << " are " << sim << " similar." << std::endl;
                heatMapMat[i][j] = sim;
                heatMapMat[j][i] = sim;
                if(minMax.first > sim) minMax.first = sim;
            }
        }
        fs.close();

        for(int i = 0; i < heatMapMat.size(); i++) {
            for(int j = 0; j < heatMapMat[i].size(); j++) {
                heatMapData.push_back(boost::make_tuple(i,j,heatMapMat[i][j]));
            }
        }
    
        Plotter::newWindow(std::string("Similarities Heatmap"),expPath+fileName+".png");
        DebugLogger::ss << "Min: " << minMax.first << ", max: " << minMax.second;
        DebugLogger::log();
        Plotter::plotHeatMap(heatMapData,minMax,"Similarities Heatmap");

    }
    
    void computeFeatures(const std::vector<std::shared_ptr<IBS>> &ibses, bool plot = false){
        int ibsAmount = ibses.size();
        #pragma omp parallel for 
        for(int i = 0; i < ibsAmount; i++) {
            IBS *ibs = &*ibses[i];
            ibs->computeGeomFeatures();
            ibs->computeTopoFeatures();
            if(plot)
                ibs->plotFeatures(expPath+"IBSFeat_"+ibs->ibsObj->getName()+".png");
        }
    }

    bool compareIBS(const std::shared_ptr<IBS> i,const std::shared_ptr<IBS> j) {
        return i->obj2->compareObjOnCentroid(j->obj2);
        //Point3d centrI = i->obj2->getCentroid();
        //Point3d centrJ = j->obj2->getCentroid();
        //if(centrI.z() == centrJ.z()) {
            //if(centrI.y() == centrJ.y())
                //return centrI.x() < centrJ.z();
            //else
                //return centrI.y() < centrJ.y();
        //} else 
            //return centrI.z() < centrJ.z();
    }

    std::function<void(std::shared_ptr<Input>)> run[] = {
        [] (std::shared_ptr<Input> in) {
            // We only have 1 scene
            Scene s = in->getScenes()[0];

            // Divide objects in 2 sets: 1 with the plane and another with the blocks
            std::vector<std::shared_ptr<Object>> set1;
            std::vector<std::shared_ptr<Object>> set2;
            for(std::shared_ptr<Object> o : s.getObjects()) {
                if(o->getName(false).find("Plane") != std::string::npos) set2.push_back(o);
                else set1.push_back(o);
            }

            // Sample the objects in the scene
            s.samplePoints();

            // Compute Ibses between two sets
            IbsGenerator ibsGen;
            std::vector<std::shared_ptr<IBS>> ibses = ibsGen.computeIBSBetweenTwoSets(set1, set2);

            // Calculate features (only geometric)
            std::vector<std::shared_ptr<Object>> objects = s.getObjects();
            for(std::shared_ptr<IBS> ibs:ibses) {
                objects.push_back(ibs->ibsObj);
            }
            DebugTimer timer;
            timer.start();
            computeFeatures(ibses,true);
            timer.stop();
            timer.printElapsedTime(std::string("Feature calculation"));

            //std::sort(ibses.begin(),ibses.end(),compareIBS);
            writeSimilarities(ibses,"ibsSimilarities");

            // Display the result
            Display::display(objects,expPath+std::string("ibses.blend"),false);
        }
    };

    auto experiments = std::vector<std::pair<ExpFuncIdx,std::vector<std::string>>>{
        /* EXPERIMENT 0: A plane with blocks on it */
        std::pair<ExpFuncIdx,std::vector<std::string>>(0,std::vector<std::string>{"blocksOnPlane"}),
        /* EXPERIMENT 1: A plane with different objects on it */
        std::pair<ExpFuncIdx,std::vector<std::string>>(0,std::vector<std::string>{"objsOnPlane"}),
        /* EXPERIMENT 2: Distance from cubes on plane to plane */
        std::pair<ExpFuncIdx,std::vector<std::string>>(0,std::vector<std::string>{"distToPlaneGround"}),
        /* EXPERIMENT 3: Distance from cubes above plane to plane */
        std::pair<ExpFuncIdx,std::vector<std::string>>(0,std::vector<std::string>{"distToPlaneHeight"}),
        /* EXPERIMENT 4: Distance from cubes to plane */
        std::pair<ExpFuncIdx,std::vector<std::string>>(0,std::vector<std::string>{"blockEnclosedInEachOtherCorners"}),
        /* EXPERIMENT 5: Distance from cubes to plane */
        std::pair<ExpFuncIdx,std::vector<std::string>>(0,std::vector<std::string>{"blockEnclosedInEachOtherGroundMiddle"}),
        /* EXPERIMENT 6: Distance from cubes to plane */
        std::pair<ExpFuncIdx,std::vector<std::string>>(0,std::vector<std::string>{"blockEnclosedInEachOtherGroundSide"}),
        /* EXPERIMENT 7: Distance from cubes to plane */
        std::pair<ExpFuncIdx,std::vector<std::string>>(0,std::vector<std::string>{"blockEnclosedInEachOtherHeightMiddle"}),
        /* EXPERIMENT 8: Distance from cubes to plane */
        std::pair<ExpFuncIdx,std::vector<std::string>>(0,std::vector<std::string>{"blockEnclosedInEachOtherHeightSide"})
    };
}

int main(int argc, const char* argv[]) {
    //std::shared_ptr<Input> input;
    //std::vector<std::string> scenePaths;
    //for(std::string s: experiments::scenes) scenePaths.push_back(SCENE_FOLDER_PATH + s + std::string(".blend"));
    //if(!scenePaths.empty()) input = std::shared_ptr<Input>(new Input(scenePaths));

    for (int i = 0; i < argc-1; ++i)
    {
        int eNb = strtol(argv[i+1],0,10);
        std::shared_ptr<Input> input;
        std::vector<std::string> scenePaths;
        for(std::string s: experiments::experiments[eNb].second) 
            scenePaths.push_back(SCENE_FOLDER_PATH + s + std::string(".blend"));
        if(!scenePaths.empty()) 
            input = std::shared_ptr<Input>(new Input(scenePaths));

        std::ostringstream path;
        path << EXP_PATH << eNb << "/";
        
        bool expPathOk = false;
        boost::filesystem::path dir(path.str());
        if(!boost::filesystem::exists(dir)) {
            if(boost::filesystem::create_directory(dir))
                expPathOk = true;
        } else
            expPathOk = true;
        experiments::expPath = path.str();
        experiments::run[experiments::experiments[eNb].first](input);
    }
}
#endif