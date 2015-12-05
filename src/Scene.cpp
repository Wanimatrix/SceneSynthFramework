#include "Scene.h"
#include <iostream>

Scene::Scene() : objects() {}

Scene::~Scene() {
}

void Scene::addObject(std::shared_ptr<Object> object) {
    objects.push_back(object);
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