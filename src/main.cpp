#include <iostream>
#include <string>
#include <vector>
#include "Input.h"
#include "AnalysisPhase/IbsGenerator.h"
#include "writeObj.h"

using namespace std;

int main() {
    //string inputfile = "../../Data/Table_Chairs.blend";
    string inputfile = "../../../../Data/SceneDB/table_cup.blend";

    //Scene s = SceneLoader::load(inputfile);

    Input i(std::vector<string>({inputfile}));
    Scene s = i.getScenes()[0];

    std::cout << "Amount of meshes: " << s.getObjects().size() << std::endl;
    for(Object o : s.getObjects()) {
        std::cout << "Mesh: " << o.getName() << std::endl;
        std::cout << "Mesh face amount: " << o.getMesh().number_of_faces() << std::endl;
        std::cout << "Mesh vertex amount: " << o.getMesh().number_of_vertices() << std::endl;
    }

    s.samplePoints();
    IbsGenerator ibsGen;

    std::vector<Object> first = std::vector<Object>({s.getObjects()[0],s.getObjects()[1]});
    std::vector<Object> second = std::vector<Object>({s.getObjects()[1],s.getObjects()[2]});
    std::vector<Object> third = std::vector<Object>({s.getObjects()[0],s.getObjects()[2]});

    writeOBJ::write(*ibsGen.computeIBS(first)[0], "ibs1.obj" );
    ibsGen.reset();
    writeOBJ::write(*ibsGen.computeIBS(second)[0], "ibs2.obj" );
    ibsGen.reset();
    writeOBJ::write(*ibsGen.computeIBS(third)[0], "ibs3.obj" );

    /*Assimp::Importer importer;
    Assimp::DefaultLogger::create("", Assimp::Logger::VERBOSE);

    const aiScene *aiScene = importer.ReadFile(inputfile, 0);
    std::cout << "Meshes: " << aiScene->mNumMeshes << std::endl;
    std::cout << "MeshName: " << aiScene->mMeshes[0]->mName.C_Str() << std::endl;
    std::cout << "MeshName: " << aiScene->mMeshes[1]->mName.C_Str() << std::endl;*/

    return 0;
}