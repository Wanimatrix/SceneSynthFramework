#include "SceneLoader.h"

#include "Object.h"

#include "Debug/DebugTools.h"
#include <string>
#include <sstream>
#include <iostream>
#include <exception>
//#include <boost/algorithm/string.hpp>

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/postprocess.h>     // Post processing flags
#include <assimp/DefaultLogger.hpp>     // logger


aiMatrix4x4 blenderTransformation = aiMatrix4x4(1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1);
Scene SceneLoader::load(const std::string &path) {
    aiMatrix4x4 transformation = aiMatrix4x4();

    if (path.find(std::string(".dae")) != std::string::npos) { // When using collada, the coordinate system is diferent from Blender.
        transformation = aiMatrix4x4(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1);
    }

    Assimp::Importer importer;
    Assimp::DefaultLogger::create("", Assimp::Logger::VERBOSE);

    importer.SetPropertyInteger("AI_CONFIG_PP_RVC_FLAGS", aiComponent_MESHES | aiComponent_MATERIALS | aiComponent_CAMERAS);
    const aiScene *aiScene = importer.ReadFile(path, 0 | aiProcess_RemoveComponent);

    // If the import failed, report it
    if (!aiScene)
    {
        std::stringstream errMsg;
        errMsg << importer.GetErrorString();
        //throw std::exception(errMsg.str().c_str());
    }

    Scene scene;

    recursive_scene_setup(scene, aiScene, aiScene->mRootNode, transformation);
    for(std::shared_ptr<Object> o : scene.getObjects()) {
        DebugLogger::ss << o->getName() << std::endl;
        DebugLogger::ss << o->getCentroid() << std::endl;
    }
    DebugLogger::log();
    return scene;
}

void SceneLoader::recursive_scene_setup(Scene &scene, const aiScene *sc, const aiNode* nd, const aiMatrix4x4 currentTrans)
{

    /*std::cout << "[[" << currentTrans.a1 << "," << currentTrans.a2 << "," << currentTrans.a3 << "," << currentTrans.a4 << "]" << std::endl;
    std::cout << "[" << currentTrans.b1 << "," << currentTrans.b2 << "," << currentTrans.b3 << "," << currentTrans.b4 << "]" << std::endl;
    std::cout << "[" << currentTrans.c1 << "," << currentTrans.c2 << "," << currentTrans.c3 << "," << currentTrans.c4 << "]" << std::endl;
    std::cout << "[" << currentTrans.d1 << "," << currentTrans.d2 << "," << currentTrans.d3 << "," << currentTrans.d4 << "]]" << std::endl;*/

    // Get node transformation matrix
    aiMatrix4x4 m = nd->mTransformation;
    const aiMatrix4x4 newTrans = m*currentTrans;

    // draw all meshes assigned to this node
    for (unsigned int n = 0; n < nd->mNumMeshes; ++n) {

        aiMesh *mesh = sc->mMeshes[nd->mMeshes[n]];
        std::vector<std::string> splittedName;
        //boost::split(splittedName, std::string(mesh->mName.C_Str()), boost::is_any_of("_."));
        std::shared_ptr<Object> o(new Object(/*splittedName[0], */mesh, blenderTransformation*newTrans));
        scene.addObject(o);
        //std::cout << "Mesh: " << mesh->mName.C_Str() << std::endl;
        //std::cout << "First  coordinate: (" << mesh->mVertices[0].x << "," << mesh->mVertices[0].y << "," << mesh->mVertices[0].z << ")" << std::endl;
        

    }

    // draw all children
    for (unsigned int n = 0; n < nd->mNumChildren; ++n) {
        recursive_scene_setup(scene, sc, nd->mChildren[n], newTrans);
    }
}