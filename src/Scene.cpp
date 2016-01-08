#include "Scene.h"
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "Debug/DebugTools.h"

Scene::Scene() : objects() {}

Scene::~Scene() {
}

void Scene::addObject(std::shared_ptr<Object> object) {
    std::string lowerName = boost::to_lower_copy(object->getName(false));
    std::vector<std::shared_ptr<Object>>::iterator it;
    bool added = false;
    int idx = 0;
    int currentIdx;
    for(it = objects.begin(), idx = 0 ; it < objects.end(); it++) {
        currentIdx = it-objects.begin();
        if(!added && !(*it)->compareObjOnCentroid(object)){
            objects.insert(it,object);
            // After insert, it is invalid!!
            it = objects.begin() + currentIdx;
            object->setUniqueObjIndex(idx++);
            added = true;
        } else {
            if(boost::to_lower_copy((*it)->getName(false)) == lowerName) {
                if(added)
                    (*it)->setUniqueObjIndex(idx);
                idx++;
            }
        }
    }
    if(!added) {
        objects.push_back(object);
        object->setUniqueObjIndex(idx);
    }
}

std::vector<std::shared_ptr<Object>> Scene::getObjects() const {
    return objects;
}


void Scene::samplePoints() {
    if (objects.empty())
    {
        return;
    }

    // sample based on surface area
    double minArea = objects[0]->getMesh().getSurfaceArea();
    double totalArea = 0;
    for (auto obj:objects)
    {
        if (obj->getMesh().getSurfaceArea() < minArea)
        {
            minArea = obj->getMesh().getSurfaceArea();
        }
        totalArea += obj->getMesh().getSurfaceArea();
    }

    int sampleNum = 2000 * objects.size();
    for (auto obj:objects)
    {
        int num = sampleNum * obj->getMesh().getSurfaceArea() / totalArea;
        if (num < 1000)
        {
            num = 1000;
        }
        /*if (obj->isCentral)
        {
            num = num * 2;
        }*/

        obj->sampling(num);
    }
}