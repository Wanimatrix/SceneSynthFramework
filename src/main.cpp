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
    Plotter p(true);
    p.plotHist(std::vector<double>({1,6,3,7,4}),true);

    // string inputfile = "../../Data/table_cup.blend";
    // // string inputfile = "../../../../Data/SceneDB/table_cup.blend";

    // //Scene s = SceneLoader::load(inputfile);

    // Input i(std::vector<string>({inputfile}));
    // Scene s = i.getScenes()[0];

    // std::cout << "Amount of meshes: " << s.getObjects().size() << std::endl;
    // for(std::shared_ptr<Object> o : s.getObjects()) {
    //     std::cout << "Mesh: " << o->getName() << std::endl;
    //     std::cout << "Mesh face amount: " << o->getMesh().mesh3d->number_of_faces() << std::endl;
    //     std::cout << "Mesh vertex amount: " << o->getMesh().mesh3d->number_of_vertices() << std::endl;
    // }

    // s.samplePoints();
    // IbsGenerator ibsGen;

    // std::vector<std::shared_ptr<IBS>> ibses = ibsGen.computeIBSForEachTwoObjs(s.getObjects());

    // vector<std::shared_ptr<Object>> objects = s.getObjects();
    // for(auto ibs:ibses) objects.push_back(ibs->ibsObj);
    // // ostringstream sstr;
    // // for(int i = 0; i < meshes.size(); i++) {
    // //     sstr << i;
    // //     objects.push_back(Object("ibs"+sstr.str(), meshes[i]));
    // //     sstr.str("");
    // // }

    // Display::display(objects);

    return 0;
}