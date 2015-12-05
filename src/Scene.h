#pragma once
#include "Object.h"
#include <vector>

class Scene
{
public:
    Scene();
    virtual ~Scene();

    virtual void addObject(std::shared_ptr<Object> object);
    virtual std::vector<std::shared_ptr<Object>> getObjects() const;
    virtual void samplePoints();

private:
    std::vector<std::shared_ptr<Object>> objects;
};