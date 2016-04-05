/* 
 * Scene.h
 *
 * Author: Wouter Franken
 *
 * CONTENT: 
 *   - Scene class header
 *      + CON-/DE-STRUCTORS
 *      + GETTERS
 *      + SETTERS
 *      + SAMPLING
 */

#pragma once

#include "Object.h"
#include <vector>

//typedef struct SceneGraphNode 
    //Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    //std::vector<SceneGraphNode> children = std::vector<SceneGraphNode(0);
    //std::shared_ptr<Object> obj = 0;
//} SceneGraphNode;

class Scene
{
public:
    // CON-/DE-STRUCTORS
    Scene();
    virtual ~Scene();

    // GETTERS
    virtual std::vector<std::shared_ptr<Object>> getObjects() const {return m_objects;};

    // SETTERS
    virtual void addObject(std::shared_ptr<Object> object) {m_objects.push_back(object);};
    virtual void removeObject(std::shared_ptr<Object> object) 
    {
        m_objects.erase(std::remove(m_objects.begin(), m_objects.end(), object), m_objects.end());
        for(std::shared_ptr<Object> obj : m_objects)
        {
            obj->removeNonUniformSample(object);
        }
    }

    // SAMPLING
    virtual void samplePoints();

private:
    std::vector<std::shared_ptr<Object>> m_objects;
    std::map<std::string,int> m_uniqueObjsCounter;
    //SceneGraphNode m_sceneGraph;
};
