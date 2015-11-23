#include <iostream>
#include <string>
#include <vector>
#include "Input.h"
#include "AnalysisPhase/IbsGenerator.h"

using namespace std;

int main() {
    string inputfile = "../../Data/Table_Chairs.blend";

    //Scene s = SceneLoader::load(inputfile);

    Input i(std::vector<string>({inputfile}));
    Scene s = i.getScenes()[0];

    std::cout << "Amount of meshes: " << s.getObjects().size() << std::endl;
    std::vector<Object*> objs;
    for(Object o : s.getObjects()) {
        objs.push_back(&o);
        std::cout << "Mesh: " << o.getName() << std::endl;
        std::cout << "Mesh face amount: " << o.getMesh().number_of_faces() << std::endl;
        std::cout << "Mesh vertex amount: " << o.getMesh().number_of_vertices() << std::endl;
    }

    IbsGenerator ibsGen;
    std::vector<Mesh3d*> meshes = ibsGen.computeIBS(objs);

    /*Assimp::Importer importer;
    Assimp::DefaultLogger::create("", Assimp::Logger::VERBOSE);

    const aiScene *aiScene = importer.ReadFile(inputfile, 0);
    std::cout << "Meshes: " << aiScene->mNumMeshes << std::endl;
    std::cout << "MeshName: " << aiScene->mMeshes[0]->mName.C_Str() << std::endl;
    std::cout << "MeshName: " << aiScene->mMeshes[1]->mName.C_Str() << std::endl;*/

    return 0;
}