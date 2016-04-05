#include "IBSFitEval.h"
#include "../Display/Display.h"
#include "../AnalysisPhase/IbsGenerator.h"
#include "../AnalysisPhase/IBS.h"
#include <cassert>

void IBSFitEval::sceneCleanup(std::shared_ptr<Object> &objectToClear)
{
    m_scene.removeObject(objectToClear);
    //delete objectToClear;
    objectToClear = nullptr;
}

std::shared_ptr<Object> IBSFitEval::setupScene(Individual i)
{
    std::shared_ptr<Object> result(new Object(i.name,m_mesh));
    result->setPosition(i.vals[0],i.vals[1],i.vals[2]);
    m_scene.addObject(result);
    return result;
}

bool IBSFitEval::validate(Individual i)
{
    bool result = true;
    std::shared_ptr<Object> indObj(new Object(i.name,m_mesh));
    indObj->setPosition(i.vals[0],i.vals[1],i.vals[2]);
    std::vector<std::shared_ptr<Object>> sceneObjs = m_scene.getObjects();
    for(std::shared_ptr<Object> obj : sceneObjs)
    {
        if(indObj->intersects(obj))
        {
            result = false;
            break;
        }
    }
    return result;
}

void IBSFitEval::displayIndividualsInScene(std::vector<Individual> individuals, std::string path, std::vector<std::shared_ptr<Object>> extraObjects)
{
    std::vector<std::shared_ptr<Object>> tmpObjects;
    for(int k = 0; k < individuals.size(); k++)
    {
        Individual i = individuals[k];
        std::shared_ptr<Object> obj = setupScene(i);
        tmpObjects.push_back(obj);
    }
    std::vector<std::shared_ptr<Object>> objsToBeDisplayed;
    objsToBeDisplayed.insert(objsToBeDisplayed.end(), extraObjects.begin(), extraObjects.end());
    std::vector<std::shared_ptr<Object>> sceneObjs = m_scene.getObjects();
    DebugLogger::ss << "displayObjs size = " << objsToBeDisplayed.size() << ", sceneObjs size = " << sceneObjs.size();
    DebugLogger::log();
    objsToBeDisplayed.insert(objsToBeDisplayed.end(), sceneObjs.begin(), sceneObjs.end());
    Display::display(objsToBeDisplayed,path,false);
    for(std::shared_ptr<Object> obj : tmpObjects)
    {
        sceneCleanup(obj);
    }
}

std::pair<double,std::shared_ptr<IBS>> IBSFitEval::eval(Individual &i)
{
    assert(m_scene.getObjects().size() == 1); 
    std::shared_ptr<Object> table = m_scene.getObjects()[0];

    DebugLogger::ss << "Current Individual: " << i.vals[0] << ", " << i.vals[1] << ", " << i.vals[2];
    DebugLogger::log();

    // Init scene with individual
    std::shared_ptr<Object> chair = setupScene(i);
    if(chair->intersects(table))
    {
        sceneCleanup(chair);
        return std::pair<double,std::shared_ptr<IBS>>(-std::numeric_limits<double>::infinity(),std::shared_ptr<IBS>(nullptr));
    }
    else
    {
        //Display::display(m_scene.getObjects(),std::string("newSceneWithChair.blend"),false);
        m_scene.samplePoints();

        // Generate IBS between other object and this.
        if(m_sampleScheme == IbsSampleScheme::SampleScheme::IMPORTANCE_DISTANCE) 
        {
            table->sampleNonUniform(table->getUniformSamples().size(),chair);
            chair->sampleNonUniform(chair->getUniformSamples().size(),table);
        }
        IbsGenerator ibsGen;
        std::vector<std::shared_ptr<IBS>> ibses = ibsGen.computeIBS(std::vector<std::shared_ptr<Object>>({chair,table}));
        DebugLogger::ss << "Were they intersecting? " << chair->intersects(table) << std::endl;
        DebugLogger::ss << "Amount ibses: " << ibses.size() << std::endl;
        DebugLogger::ss << "Amount objects in the scene: " << m_scene.getObjects().size();
        DebugLogger::log();
        if(ibses.size() != 1)
            Display::display(m_scene.getObjects(),std::string("newSceneWithChair.blend"),false);
        assert(ibses.size() == 1);

        std::shared_ptr<IBS> ibsPtr = ibses[0];
        for(std::shared_ptr<IBS> ibs : ibses) {
            ibs->computeGeomFeatures();
            ibs->computeTopoFeatures();
        }

        DebugLogger::ss << "Calculating similarity...";
        DebugLogger::log();
        // Calculate similarity
        double similarity = m_ibses[0]->getSimilarity(*ibses[0]);
        DebugLogger::ss << "Similarity 0: " << similarity << std::endl;
        DebugLogger::log();
        for(int i = 1; i < m_ibses.size(); i++) 
        {
            double tmpSimilarity = m_ibses[i]->getSimilarity(*ibses[0]);
            DebugLogger::ss << "Similarity " << i << ": " << tmpSimilarity << std::endl;
            DebugLogger::log();
            if (tmpSimilarity > similarity)
            {
                similarity = tmpSimilarity;
            }
        }
        std::ostringstream oss;
        oss << ibses[0]->ibsObj->getName() << "_" << similarity;
        ibses[0]->ibsObj->setName(oss.str());
        //similarity /= m_ibses.size();
        DebugLogger::ss << "Similarity calculated...";
        DebugLogger::log();

        /* std::ostringstream oss; */
        /* oss << i.name << "_" << similarity; */
        /* i.name = oss.str(); */

        // Clean up the scene.
        sceneCleanup(chair);
        assert(ibsPtr);
        return std::pair<double,std::shared_ptr<IBS>>(similarity,ibsPtr);
    }
}
