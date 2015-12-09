#pragma once

#define DISPLAY_PATH "../src/Display/"
#define TMP_PATH "/tmp/SceneSynthesis/"
#define OBJ_PATH "objs/"
#define SCENE_PATH "scene.txt"

#include "../Object.h"
#include <vector>

class Display
{
public:
    static void display(const std::vector<std::shared_ptr<Object>> &objects, const std::string &save = std::string(""), bool display = true);
    
};