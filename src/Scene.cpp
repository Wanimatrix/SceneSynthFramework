#include "Scene.h"

Scene::Scene() : objects() {}

Scene::~Scene() {}

void Scene::addObject(Object object) {
    objects.push_back(object);
}

std::vector<Object> Scene::getObjects() const {
    return objects;
}

void Scene::SamplePoints() {
    if (objects.empty())
    {
        return;
    }

    // sample based on surface area
    for (auto obj:objects)
    {
        if (obj.getSurfaceArea() == 0)
        {
            obj->computeArea();
        }
    }

    double minArea = objects[0]->surfaceArea;
    double totalArea = 0;
    for (auto obj:objects)
    {
        if (obj->surfaceArea < minArea)
        {
            minArea = obj->surfaceArea;
        }
        totalArea += obj->surfaceArea;
    }

    int sampleNum = 2000 * objects.size();
    for (auto obj:objects)
    {
        int num = sampleNum * obj->surfaceArea / totalArea;
        if (num < 1000)
        {
            num = 1000;
        }
        if (obj->isCentral)
        {
            num = num * 2;
        }
        obj->sampling(num);
    }
}