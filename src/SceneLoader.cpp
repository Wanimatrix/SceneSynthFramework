#include "SceneLoader.h"

#include "Object.h"

#include "Debug/DebugTools.h"
#include "Utilities/Utilities.h"
#include <string>
#include <sstream>
#include <iostream>
#include <exception>
//#include <boost/algorithm/string.hpp>

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/postprocess.h>     // Post processing flags
#include <assimp/DefaultLogger.hpp>     // logger

Eigen::Matrix4d blenderScalingMat;
Eigen::Matrix4d blenderRotMat;

using namespace utilities;

aiMatrix4x4 blenderTransformation = aiMatrix4x4(1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1);
Scene SceneLoader::load(const std::string &path) {
    aiMatrix4x4 transformation = aiMatrix4x4();
    std::stack<Transformation> transformationStack;


    //if (path.find(std::string(".dae")) != std::string::npos) { // When using collada, the coordinate system is diferent from Blender.
        //transformation = aiMatrix4x4(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1);
    //}
    if (path.find(std::string(".blend")) != std::string::npos) { // When using collada, the coordinate system is diferent from Blender.
        blenderScalingMat << 1, 0,  0, 0,
                             0, 1,  0, 0,
                             0, 0, -1, 0,
                             0, 0,  0, 1;
        blenderRotMat << 1, 0, 0, 0,
                         0, 0, 1, 0,
                         0, 1, 0, 0,
                         0, 0, 0, 1;
    }

    Assimp::Importer importer;
    Assimp::DefaultLogger::create("", Assimp::Logger::VERBOSE);

    importer.SetPropertyInteger("AI_CONFIG_PP_RVC_FLAGS", aiComponent_MESHES | aiComponent_MATERIALS | aiComponent_CAMERAS);
    DebugLogger::ss << "Reading scene...";
    DebugLogger::log();
    const aiScene *aiScene = importer.ReadFile(path, 0 | aiProcess_RemoveComponent);
    DebugLogger::ss << "Scene read";
    DebugLogger::log();

    // If the import failed, report it
    if (!aiScene)
    {
        std::stringstream errMsg;
        DebugLogger::ss << importer.GetErrorString();
        DebugLogger::log();
        //throw std::exception(errMsg.str().c_str());
    }

    Scene scene;

    recursive_scene_setup(scene, aiScene, aiScene->mRootNode, transformationStack, transformation);
    for(std::shared_ptr<Object> o : scene.getObjects()) {
        DebugLogger::ss << o->getName() << std::endl;
        //DebugLogger::ss << o->getCentroid() << std::endl;
    }
    DebugLogger::log();
    return scene;
}

void SceneLoader::recursive_scene_setup(Scene &scene, const aiScene *sc, const aiNode* nd, std::stack<Transformation> currentTransfStack, aiMatrix4x4 currTrans)
{

    DebugLogger::ss << "Reading node ...";
    DebugLogger::log();

    /*std::cout << "[[" << currentTrans.a1 << "," << currentTrans.a2 << "," << currentTrans.a3 << "," << currentTrans.a4 << "]" << std::endl;
    std::cout << "[" << currentTrans.b1 << "," << currentTrans.b2 << "," << currentTrans.b3 << "," << currentTrans.b4 << "]" << std::endl;
    std::cout << "[" << currentTrans.c1 << "," << currentTrans.c2 << "," << currentTrans.c3 << "," << currentTrans.c4 << "]" << std::endl;
    std::cout << "[" << currentTrans.d1 << "," << currentTrans.d2 << "," << currentTrans.d3 << "," << currentTrans.d4 << "]]" << std::endl;*/

    // Get node transformation matrix
    //Eigen::Matrix4d transformationMatrix;
    //aiMatrix4x4 m = nd->mTransformation;
    //transformationMatrix << m.a1, m.a2, m.a3, m.a4,
                      //m.b1, m.b2, m.b3, m.b4,
                      //m.c1, m.c2, m.c3, m.c4,
                      //m.d1, m.d2, m.d3, m.d4;
    //std::bitset<3> transformationType;
    //if(m.a4 != 0)
        //transformationType = Transformation.TRANSLATION;
    //else if(m.a1 != 1)
        //transformationType = Transformation.SCALING;
    //else
        //transformationType = Transformation.ROTATION;
    //Transformation currentTransformation(transformationMatrix,transformationType);
    //transformationStack.push(currentTransformation);
    Eigen::Matrix4d transformationMatrix;
    aiMatrix4x4 m = nd->mTransformation;
    aiVector3D scaling;
    aiQuaternion rotation;
    aiVector3D translation;
    m.Decompose(scaling, rotation, translation);
    aiMatrix3x3 rotationMat = rotation.GetMatrix();
    if(doubleEpsDiff(scaling.x,1) || doubleEpsDiff(scaling.y,1) || doubleEpsDiff(scaling.z,1))
    {
        Eigen::Matrix4d scaleMatrix = Eigen::Matrix4d::Identity();
        scaleMatrix(0,0) = scaling.x;
        scaleMatrix(1,1) = scaling.y;
        scaleMatrix(2,2) = scaling.z;
        Transformation tmp(scaleMatrix,TransformationType::SCALING);
        currentTransfStack.push(tmp);
    }
    if(doubleEpsDiff(rotationMat.a1,1) || doubleEpsDiff(rotationMat.a2,0) || doubleEpsDiff(rotationMat.a3,0)
        || doubleEpsDiff(rotationMat.b1,0) || doubleEpsDiff(rotationMat.b2,1) || doubleEpsDiff(rotationMat.b3,0)
        || doubleEpsDiff(rotationMat.c1,0) || doubleEpsDiff(rotationMat.c2,0) || doubleEpsDiff(rotationMat.c3,1))
    {
        Eigen::Matrix4d rotationEigMat;
        rotationEigMat << m.a1, m.a2, m.a3, 0,
                          m.b1, m.b2, m.b3, 0,
                          m.c1, m.c2, m.c3, 0,
                          0,    0,    0,    1;
        Transformation tmp(rotationEigMat,TransformationType::ROTATION);
        currentTransfStack.push(tmp);
    }
    if(doubleEpsDiff(translation.x,0) || doubleEpsDiff(translation.y,0) || doubleEpsDiff(translation.z,0))
    {
        Eigen::Matrix4d translationMatrix = Eigen::Matrix4d::Identity();
        translationMatrix(0,3) = translation.x;
        translationMatrix(1,3) = translation.y;
        translationMatrix(2,3) = translation.z;
        Transformation tmp(translationMatrix,TransformationType::TRANSLATION);
        currentTransfStack.push(tmp);
    }
    //DebugLogger::ss << "This transformationmatrix: " << std::endl << transformationMatrix;
    //DebugLogger::log();
    const aiMatrix4x4 newTrans = m*currTrans;

    // draw all meshes assigned to this node
    for (unsigned int n = 0; n < nd->mNumMeshes; ++n) {
        DebugLogger::ss << "Looping over meshes; current: " << n;
        DebugLogger::log();
        Transformation blenderScaling(blenderScalingMat,TransformationType::SCALING);
        Transformation blenderRotation(blenderRotMat,TransformationType::ROTATION);
        currentTransfStack.push(blenderRotation);
        currentTransfStack.push(blenderScaling);
        DebugLogger::ss << "TransformStack created";
        DebugLogger::log();

        //DebugLogger::ss << "Stack-based transformation: " << std::endl << Transformation::getTransformationMatrix(currentTransfStack) << std::endl;
        //aiMatrix4x4 tmpTrans = blenderTransformation*newTrans;
        //Eigen::Matrix4d tmpTransEig;
        //tmpTransEig << tmpTrans.a1, tmpTrans.a2, tmpTrans.a3, tmpTrans.a4,
                      //tmpTrans.b1, tmpTrans.b2, tmpTrans.b3, tmpTrans.b4,
                      //tmpTrans.c1, tmpTrans.c2, tmpTrans.c3, tmpTrans.c4,
                      //tmpTrans.d1, tmpTrans.d2, tmpTrans.d3, tmpTrans.d4;
        //DebugLogger::ss << "Not stack-based: " << std::endl << tmpTransEig;
        //DebugLogger::log();

        aiMesh *mesh = sc->mMeshes[nd->mMeshes[n]];
        std::vector<std::string> splittedName;
        //boost::split(splittedName, std::string(mesh->mName.C_Str()), boost::is_any_of("_."));
        DebugLogger::ss << "Creating object";
        DebugLogger::log();
        std::shared_ptr<Object> o(new Object(/*splittedName[0], */mesh, currentTransfStack/*blenderTransformation*newTrans*/));
        scene.addObject(o);
        //std::cout << "Mesh: " << mesh->mName.C_Str() << std::endl;
        //std::cout << "First  coordinate: (" << mesh->mVertices[0].x << "," << mesh->mVertices[0].y << "," << mesh->mVertices[0].z << ")" << std::endl;
        
        DebugLogger::ss << "Going to next mesh";
        DebugLogger::log();

    }

    // draw all children
    for (unsigned int n = 0; n < nd->mNumChildren; ++n) {
        recursive_scene_setup(scene, sc, nd->mChildren[n], currentTransfStack, newTrans);
    }
}