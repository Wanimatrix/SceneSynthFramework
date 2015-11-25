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
    virtual void samplePoints();

private:
    std::vector<Object> objects;
};