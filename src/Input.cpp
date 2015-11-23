#include "Input.h"

#include "SceneLoader.h"
#include <iostream>

Input::Input(std::vector<std::string> sceneFiles) {
    for (std::string sceneFile : sceneFiles) {
        m_scenes.push_back(SceneLoader::load(sceneFile));
    }
}

Input::~Input() {}

std::vector<Scene> Input::getScenes() const {
    return m_scenes;
}