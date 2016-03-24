#pragma once
#include "GATypes.h"
#include "../AnalysisPhase/IBS.h"
#include "../Scene.h"
#include "../Mesh.h"

class IBSFitEval 
{
public:
    IBSFitEval(std::vector<std::shared_ptr<IBS>> ibses, Scene currentScene, Mesh m) 
        : m_ibses(ibses), m_scene(currentScene), m_mesh(m) {};
    virtual ~IBSFitEval() {};

    virtual std::pair<double,std::shared_ptr<IBS>> eval(Individual &i);
    virtual void displayIndividualsInScene(std::vector<Individual> i, std::string path, std::vector<std::shared_ptr<Object>> extraObjects = std::vector<std::shared_ptr<Object>>());
private:
    virtual void sceneCleanup(std::shared_ptr<Object> &objectToClear);
    virtual std::shared_ptr<Object> setupScene(Individual i);

    std::vector<std::shared_ptr<IBS>> m_ibses;
    Scene m_scene;
    Mesh m_mesh;
};