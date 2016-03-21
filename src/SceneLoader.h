#pragma once

#include <vector>
#include <string>
#include "Scene.h"
#include <assimp/scene.h>
#include <assimp/types.h>
#include "Utilities/Transformation.h"

class SceneLoader
{
public:
    static Scene load(const std::string &path);
private:
    static void recursive_scene_setup(Scene &scene, const aiScene *sc, const aiNode* nd, std::stack<Transformation> currentTransfStack, aiMatrix4x4 currTrans);
};