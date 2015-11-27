#pragma once

#define DISPLAY_PATH "Display/"
#define TMP_PATH "/tmp/SceneSynthesis/"
#define OBJ_PATH "objs/"
#define SCENE_PATH "scene.txt"

#include "../Object.h"
#include <vector>

class Display
{
public:
    static void display(const std::vector<Object> &objects);
    
};