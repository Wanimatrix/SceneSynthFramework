#include "Scene.h"

Scene::Scene() : objects() {}

Scene::~Scene() {}

void Scene::addObject(Object object) {
    objects.push_back(object);
}

std::vector<Object> Scene::getObjects() const {
    return objects;
}