#ifndef EXP

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include "Input.h"
#include "AnalysisPhase/IbsGenerator.h"
#include "Display/Display.h"
#include "Display/writeOBJ.h"
#include "Debug/DebugTools.h"

using namespace std;



int main() {


    /*Plotter p(true);
    p.plotHist(std::vector<double>({1,6,3,7,4}),true);*/

    //string inputfile = "../../Data/table_cup.blend";
    string inputfile = "../../../../Data/SceneDB/blocksOnPlane.blend";

    //Scene s = SceneLoader::load(inputfile);

    Input in(std::vector<string>({inputfile}));
    Scene s = in.getScenes()[0];

    std::cout << "Amount of meshes: " << s.getObjects().size() << std::endl;
    std::vector<std::shared_ptr<Object>> set1;
    std::vector<std::shared_ptr<Object>> set2;
    for(std::shared_ptr<Object> o : s.getObjects()) {
        if(o->getName().find("Plane") != std::string::npos) set2.push_back(o);
        else set1.push_back(o);
        /*std::cout << "Mesh: " << o->getName() << std::endl;
        std::cout << "Mesh face amount: " << o->getMesh().mesh3d->number_of_faces() << std::endl;
        std::cout << "Mesh vertex amount: " << o->getMesh().mesh3d->number_of_vertices() << std::endl;*/
    }

    s.samplePoints();
    IbsGenerator ibsGen;

    //std::vector<std::shared_ptr<IBS>> ibses = ibsGen.computeIBSForEachTwoObjs(s.getObjects());
    std::vector<std::shared_ptr<IBS>> ibses = ibsGen.computeIBSBetweenTwoSets(set1, set2);

    vector<std::shared_ptr<Object>> objects = s.getObjects();
    #pragma omp parallel for
    for(int i = 0; i < ibses.size(); i++) {
        auto ibs = ibses[i];
        ibs->computeGeomFeatures();
        ibs->computeTopoFeatures();
        //ibs->plotFeatures();
        #pragma omp critical
        objects.push_back(ibs->ibsObj);
    }

    /*
    for (int i = 0; i < ibses.size(); ++i)
    {
        for (int j = i; j < ibses.size(); ++j) {
            DebugLogger::ss << "Similarity between " << ibses[i]->ibsObj->getName() << " and " << ibses[j]->ibsObj->getName() << " = " << ibses[i]->getSimilarity(*ibses[j]) << std::endl;
            DebugLogger::log();
        }
    }
*/
    Display::display(objects);

    return 0;
}

#endif