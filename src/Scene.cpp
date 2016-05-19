/* 
 * Scene.cpp
 *
 * Author: Wouter Franken
 *
 * CONTENT: 
 *   - Scene class implementation
 *      + CON-/DE-STRUCTORS
 *      + GETTERS
 *      + SETTERS
 *      + SAMPLING
 */

#include "Scene.h"
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "Debug/DebugTools.h"
#include "Experiments/ConfigurationController.h"


/***********************
 *  CON-/DE-STRUCTORS  *
 ***********************/
Scene::Scene() : m_objects() 
{
}

Scene::~Scene() 
{
}

/**************
 *  SAMPLING  *
 **************/
void Scene::samplePoints() {
    if (m_objects.empty())
    {
        return;
    }

    // sample based on surface area
    double minArea = m_objects[0]->getMesh().getSurfaceArea();
    double totalArea = 0;
    for (auto obj:m_objects)
    {
        if (obj->getMesh().getSurfaceArea() < minArea)
        {
            minArea = obj->getMesh().getSurfaceArea();
        }
        totalArea += obj->getMesh().getSurfaceArea();
    }

    /* int sampleNum = stoi(ConfigurationController::getInstance().getCurrentConfiguration().get("SampleAmount")) * m_objects.size(); */
    //int sampleNum = 2000 * (totalArea/minArea);
    int minSampleAmount = std::stoi(ConfigurationController::getInstance().getCurrentConfiguration().get("SampleAmount"));
    for (auto obj:m_objects)
    {
        int num = std::stoi(ConfigurationController::getInstance().getCurrentConfiguration().get("SampleAmount"));
        /* obj->setAmountRaySamples(std::min(minSampleAmount * (obj->getMesh().getSurfaceArea()/minArea), minSampleAmount*10.0)); */
        obj->setAmountRaySamples(num);
        /* if(ConfigurationController::getInstance().getCurrentConfiguration().get("SampleScheme") == "importance_distance") num /= 2; */
        /* int num = sampleNum * obj->getMesh().getSurfaceArea() / totalArea; */
        /* if (num < 1000) */
        /* { */
        /*     num = 1000; */
        /* } */



        /*if (obj->isCentral)
        {
            num = num * 2;
        }*/

        obj->sampleUniform(num);
    }
}
