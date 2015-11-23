#pragma once
#include "Object.h"
#include <vector>

class Scene
{
public:
    Scene();
    virtual ~Scene();

    virtual void addObject(Object object);
    virtual std::vector<Object> getObjects() const;

private:
    std::vector<Object> objects;
};