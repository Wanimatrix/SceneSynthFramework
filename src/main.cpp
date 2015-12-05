#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include "Input.h"
#include "AnalysisPhase/IbsGenerator.h"
#include "Display/Display.h"
#include "Display/writeOBJ.h"

using namespace std;

int main() {
    string inputfile = "../../Data/table_cup.blend";
    // string inputfile = "../../../../Data/SceneDB/table_cup.blend";

    //Scene s = SceneLoader::load(inputfile);

    Input i(std::vector<string>({inputfile}));
    Scene s = i.getScenes()[0];

    std::cout << "Amount of meshes: " << s.getObjects().size() << std::endl;
    for(Object o : s.getObjects()) {
        std::cout << "Mesh: " << o.getName() << std::endl;
        std::cout << "Mesh face amount: " << o.getMesh().mesh3d->number_of_faces() << std::endl;
        std::cout << "Mesh vertex amount: " << o.getMesh().mesh3d->number_of_vertices() << std::endl;
    }

    s.samplePoints();
    IbsGenerator ibsGen;

    std::vector<Mesh> meshes = ibsGen.computeIBSForEachTwoObjs(s.getObjects());

    vector<Object> objects = s.getObjects();
    ostringstream sstr;
    for(int i = 0; i < meshes.size(); i++) {
        sstr << i;
        objects.push_back(Object("ibs"+sstr.str(), meshes[i]));
        sstr.str("");
    }

    Display::display(objects);

    return 0;
}