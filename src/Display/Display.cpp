#include "Display.h"

#include <iostream>
#include <string>
#include "writeOBJ.h"

void Display::display(const std::vector<Object> &objects) {
    std::cout << "Start displaying ..." << std::endl;

    //TODO fix this hardcoded thing!

    // Write all objects to obj and write their path in scene file
    std::ofstream file(std::string(TMP_PATH)+SCENE_PATH, std::ofstream::trunc);
    for(Object obj:objects){
        writeOBJ::write(obj, std::string(TMP_PATH)+OBJ_PATH+obj.getName()+".obj" );
        file << obj.getName() << ".obj" << std::endl;
    }
    file.close();

    // Start blender and show the scene with IBSes
    system(("export SCENE_PATH=`cygpath.exe -aw "+std::string(TMP_PATH)+std::string(SCENE_PATH)+"` && \
        blender `cygpath.exe -aw "+std::string(DISPLAY_PATH)+"clean.blend` -P `cygpath.exe -aw "+std::string(DISPLAY_PATH)+"displayBlender.py`").c_str());
}