#include "Display.h"

#include <iostream>
#include <string>
#include "writeOBJ.h"
#include "../Debug/DebugTools.h"
#include "../Utilities/Utilities.h"
#include "../Experiments/Configuration.h"

#ifdef __CYGWIN__
#define EXPAND_PATH(path) "$(cygpath.exe -aw "+(path)+")"
#else
#define EXPAND_PATH(path) path
#endif

void Display::display(const std::vector<std::shared_ptr<Object>> &objects, const std::string &save, bool display) {
    std::cout << "Start displaying ..." << std::endl;
    std::string tmpPath = Configuration::getInstance().get("ExperimentTmpPath");

    /* utilities::checkPath(tmpPath); */
    utilities::checkPath(tmpPath+OBJ_PATH);

    // Write all objects to obj and write their path in scene file
    std::ofstream file(tmpPath+SCENE_PATH, std::ofstream::trunc);
    for(std::shared_ptr<Object> obj:objects){
        writeOBJ::write(*obj, tmpPath+OBJ_PATH+obj->getName()+".obj" );
        file << obj->getName() << ".obj" << std::endl;
    }
    file.close();

    // Start blender and show the scene with IBSes
    std::string scenePath = EXPAND_PATH(tmpPath+std::string(SCENE_PATH));
    std::string cleanPath = EXPAND_PATH(std::string(DISPLAY_PATH)+"clean.blend");
    std::string displayPYPath = EXPAND_PATH(std::string(DISPLAY_PATH)+"displayBlender.py");

    DebugLogger::ss << "export SCENE_PATH=" << scenePath << std::string((save == "") ? "" : " SAVE=") << save << " && blender " << std::string(display ? "" : "-b ") << cleanPath << " -P " << displayPYPath;
    std::string command = DebugLogger::ss.str();
    DebugLogger::log();
    system(command.c_str());
}
