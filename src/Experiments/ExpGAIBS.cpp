#include "ExpGAIBS.h"
#include <iostream> 
#include <boost/algorithm/string.hpp>
#include "../Debug/DebugTools.h"
#include "../AnalysisPhase/IbsGenerator.h"
#include "../Display/Display.h"
#include "GA.h"
#include "IBSFitEval.h"

#define EXPTYPE "ga"

std::vector<std::shared_ptr<IBS>> ExpGAIBS::compute(std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>> sets){
    // Compute Ibses between two sets
    std::vector<std::shared_ptr<IBS>> ibses;
    if(m_onePass)
      ibses = computeIBSBetweenTwoSetsWithOneVoronoi(sets.first, sets.second);
    else
      ibses = computeIBSBetweenTwoSets(sets.first, sets.second);

    DebugLogger::ss << "Calculating features...";
    DebugLogger::log();

    // Calculate features (only geometric)
    m_objects = m_scene.getObjects();
    for(std::shared_ptr<IBS> ibs:ibses) {
        m_objects.push_back(ibs->ibsObj);
    }
    DebugTimer timer;
    timer.start();
    computeFeatures(ibses,true);
    timer.stop();
    timer.printElapsedTime(std::string("Feature calculation"));

    assert(sets.second.size() == 1 && sets.second[0]->getName().find("Table") != std::string::npos);

    Scene newScene;
    std::shared_ptr<Object> tableObj(new Object("Table", sets.second[0]->getMesh()));
    newScene.addObject(tableObj);
    //Display::display(newScene.getObjects(),EXP_PATH+"/"+getExpPath()+"/"+std::string("newScene.blend"),false);
    IBSFitEval ibsFitEval(ibses, newScene, sets.first[0]->getMesh(), m_sampleScheme);
    GAOptions options;
    options.logFitness = true;
    options.populationSize = 20;
    options.outputPath = EXP_PATH+"/"+getExpPath()+"/";
    std::vector<Object> sceneObjects;
    DebugLogger::ss << "Scene objects: " << newScene.getObjects().size() << std::endl;
    for(std::shared_ptr<Object> objPtr : newScene.getObjects()) {
        DebugLogger::ss << "Object " << objPtr->getName() << std::endl;
        sceneObjects.push_back(*objPtr);
        //for(Vertex v : objPtr->getMesh().mesh3d->vertices())
            //DebugLogger::ss << "v " << objPtr->getMesh().mesh3d->point(v) << std::endl;
    }
    IsoCub3d bboxCuboid = IsoCub3d(CGAL::bbox_3(std::begin(sceneObjects),std::end(sceneObjects)));
    Eigen::Vector4d centroidNewObj = sets.first[0]->getCentroid();
    Individual min, max;
    min.name = "MinIndividual";
    max.name = "MaxIndividual";
    min.vals[0] = bboxCuboid.xmin()-((bboxCuboid.xmax()-bboxCuboid.xmin())/2);
    //min.vals[1] = bboxCuboid.ymin()-((bboxCuboid.ymax()-bboxCuboid.ymin())/2);
    min.vals[2] = bboxCuboid.zmin()-((bboxCuboid.zmax()-bboxCuboid.zmin())/2);
    min.vals[1] = centroidNewObj(1);
    max.vals[0] = bboxCuboid.xmax()+((bboxCuboid.xmax()-bboxCuboid.xmin())/2);
    //max.vals[1] = bboxCuboid.ymax()+((bboxCuboid.ymax()-bboxCuboid.ymin())/2);
    max.vals[1] = centroidNewObj(1);
    max.vals[2] = bboxCuboid.zmax()+((bboxCuboid.zmax()-bboxCuboid.zmin())/2);
    DebugLogger::ss << "Bbox of scene: " << bboxCuboid << std::endl;
    DebugLogger::ss << "Bbox of obj[0]: " << sceneObjects[0].getBbox() << std::endl;
    DebugLogger::ss << "Minimum individual: " << min.vals[0] << ", " << min.vals[1] << ", " << min.vals[2] << std::endl;
    DebugLogger::ss << "Maximum individual: " << max.vals[0] << ", " << max.vals[1] << ", " << max.vals[2];
    DebugLogger::log();
    options.minMaxIndividual = std::make_pair(min,max);
    EndCondition econd = [](int gen, double fitness) {
        return gen == 10;
    };
    GA ga(options, econd, ibsFitEval);
    ga.run();

    return ibses;
}

void ExpGAIBS::output(std::vector<std::shared_ptr<IBS>> ibses, std::string path){
    //for(int i = 0; i < ibses.size(); i++) {
        //ibses[i]->plotFeatures(path+"IBSFeat_"+ibses[i]->ibsObj->getName()+".png");
    //}
    //std::sort(ibses.begin(),ibses.end(),compareIBS);
    //writeSimilarities(ibses,path+std::string("ibsSimilarities"));

    // Display the result
    //Display::display(m_objects,path+std::string("ibses.blend"),false);
}

std::string ExpGAIBS::getExpPath(){
  std::string folderName = std::string(IbsSampleScheme::getSampleSchemeName(m_sampleScheme));
  if(m_onePass)
    folderName = "OneVoronoi_"+folderName;
  return EXPTYPE+std::string("/")+folderName+"/"+m_ID;

}

