#pragma once

#include <vector>
#include <string>
#include "Scene.h"

class Input
{
public:
    Input(std::vector<std::string> sceneFiles);
    virtual ~Input();

    virtual std::vector<Scene> getScenes() const;
private:
    std::vector<Scene> m_scenes;
};