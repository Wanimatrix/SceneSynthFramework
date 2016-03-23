#include "ExpIBSSim.h"
#include <iostream> 
#include <boost/algorithm/string.hpp>
#include "../Debug/DebugTools.h"
#include "../AnalysisPhase/IbsGenerator.h"
#include "../Display/Display.h"

#define EXPTYPE "IBSSimilarity"

namespace IbsSampleScheme {
    SampleScheme getSampleScheme(std::string smplName) {
        if(smplName == "UNIFORM")
            return UNIFORM;
        else if(smplName == "IMPORTANCE_DISTANCE")
            return IMPORTANCE_DISTANCE;
        else if(smplName == "IMPORTANCE_DISTANCE_HULLAPPROX")
            return IMPORTANCE_DISTANCE_HULLAPPROX;
        else
            return UNIFORM;
    }
    std::string getSampleSchemeName(SampleScheme smpl) {
        switch(smpl){
        case SampleScheme::UNIFORM:
            return "uniform";
        case SampleScheme::IMPORTANCE_DISTANCE:
            return "importance_distance";
        case SampleScheme::IMPORTANCE_DISTANCE_HULLAPPROX:
            return "importance_distance_hullapprox";
        }
    }
}

DebugTimer timer;

void ExpIBSSim::writeSimilarities( std::vector<std::shared_ptr<IBS>> &ibses, const std::string &outputFile)
{
    std::sort(ibses.begin(),ibses.end(),
        []( std::shared_ptr<IBS> i,  std::shared_ptr<IBS> j) -> bool {
            return i->ibsObj->getName() < j->ibsObj->getName();
        });
    std::ofstream fs;
    std::vector<boost::tuple<int,std::string,int,std::string,double>> heatMapData;
    std::vector<std::vector<double>> heatMapMat(ibses.size(), std::vector<double>(ibses.size()));
    std::pair<double,double> minMax({5,1});
    fs.open(outputFile+".txt");
    for(int i = 0; i < ibses.size(); i++) {
        auto ibsI = ibses[i];
        for(int j = i; j < ibses.size(); j++) {
            auto ibsJ = ibses[j];
            double sim = ibsI->getSimilarity(*ibsJ);
            fs << ibsI->ibsObj->getName() << " " << ibsJ->ibsObj->getName() << " " << sim << std::endl;
            heatMapMat[i][j] = sim;
            heatMapMat[j][i] = sim;
            if(minMax.first > sim) minMax.first = sim;
        }
    }
    fs.close();

    for(int i = 0; i < heatMapMat.size(); i++) {
        for(int j = 0; j < heatMapMat[i].size(); j++) {
            std::vector<std::string> splittedNameI;
            std::string nameI = ibses[i]->ibsObj->getName();
            boost::split(splittedNameI, nameI, boost::is_any_of("_"));
            std::vector<std::string> splittedNameJ;
            std::string nameJ = ibses[j]->ibsObj->getName();
            boost::split(splittedNameJ, nameJ, boost::is_any_of("_"));
            //heatMapData.push_back(boost::make_tuple(i,nameI,j,nameJ,heatMapMat[i][j]));
            heatMapData.push_back(boost::make_tuple(i,splittedNameI[1],j,splittedNameJ[1],heatMapMat[i][j]));
        }
    }

    Plotter::newWindow(std::string("Similarities Heatmap"),outputFile+".png");
    DebugLogger::ss << "Min: " << minMax.first << ", max: " << minMax.second;
    DebugLogger::log();
    Plotter::plotHeatMap(heatMapData,minMax,"Similarities Heatmap");

}

std::vector<std::shared_ptr<IBS>> ExpIBSSim::computeIBSBetweenTwoSetsWithOneVoronoi(std::vector<std::shared_ptr<Object>> objs1, std::vector<std::shared_ptr<Object>> objs2)
{
    IbsGenerator ibsGen;
    std::vector<std::shared_ptr<IBS>> result;

    std::vector<std::shared_ptr<Object>> sceneObjects;
    sceneObjects.insert(sceneObjects.end(), objs1.begin(),objs1.end());
    sceneObjects.insert(sceneObjects.end(), objs2.begin(),objs2.end());
    result = ibsGen.computeIBS(sceneObjects);
    std::vector<std::shared_ptr<IBS>>::iterator resultIterator = result.begin();
    while (resultIterator != result.end()) {
      int obj2Members = 0;
      for (std::shared_ptr<Object> objFromObjs2 : objs2) {
        if(boost::algorithm::contains((*resultIterator)->ibsObj->getName(), objFromObjs2->getName()))
          obj2Members++;
        if(obj2Members > 1)
          break;
      }
      if (obj2Members != 1)
        resultIterator = result.erase(resultIterator);
      else
        resultIterator++;
    }
    return result;
}

std::vector<std::shared_ptr<IBS>> ExpIBSSim::computeIBSBetweenTwoSets(std::vector<std::shared_ptr<Object>> objs1, std::vector<std::shared_ptr<Object>> objs2)
{
    IbsGenerator ibsGen;
    std::vector<std::shared_ptr<IBS>> result;

    int i;
    for (i = 0; i < objs1.size(); ++i)
    {
        for (int j = 0; j < objs2.size(); ++j)
        {
            timer.start();
            if(m_sampleScheme == IbsSampleScheme::SampleScheme::IMPORTANCE_DISTANCE) {
              //assert(objs1.size()+objs2.size() <= 2);
              objs1[i]->sampleNonUniform(objs1[i]->getUniformSamples().size(),objs2[j]);
              objs2[j]->sampleNonUniform(objs2[j]->getUniformSamples().size(),objs1[i]);
            }
            if(m_sampleScheme == IbsSampleScheme::SampleScheme::IMPORTANCE_DISTANCE_HULLAPPROX) objs1[i]->sampleNonUniform(objs1[i]->getUniformSamples().size(),objs2[j],true);
            std::vector<std::shared_ptr<IBS>> ibses = ibsGen.computeIBS(std::vector<std::shared_ptr<Object>>({objs1[i],objs2[j]}));
            timer.printElapsedTime("IBS");
            result.insert(result.end(),ibses.begin(),ibses.end());
        }
    }
 
    return result;
}

void ExpIBSSim::computeFeatures(const std::vector<std::shared_ptr<IBS>> &ibses, bool plot = false)
{
    int ibsAmount = ibses.size();
    //#pragma omp parallel for 
    for(int i = 0; i < ibsAmount; i++) {
        IBS *ibs = &*ibses[i];
        ibs->computeGeomFeatures();
        ibs->computeTopoFeatures();
    }
}

std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>> ExpIBSSim::handleInput(std::shared_ptr<Input> in, std::string centralObj)
{
  DebugLogger::ss << "Handling input ...";
  DebugLogger::log();
    // We only have 1 scene
    m_scene = in->getScenes()[0];

    // Divide objects in 2 sets: 1 with the plane and another with the blocks
    std::vector<std::shared_ptr<Object>> set1;
    std::vector<std::shared_ptr<Object>> set2;
    for(std::shared_ptr<Object> o : m_scene.getObjects()) {
        if(o->getName(/*false*/).find(centralObj) != std::string::npos) set2.push_back(o);
        else set1.push_back(o);
    }

    // Sample the objects in the scene
    m_scene.samplePoints();
    assert(set2.size() > 0);
    return std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>>(set1,set2);
}

std::vector<std::shared_ptr<IBS>> ExpIBSSim::compute(std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>> sets)
{
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

    return ibses;
}

void ExpIBSSim::output(std::vector<std::shared_ptr<IBS>> ibses, std::string path)
{
    for(int i = 0; i < ibses.size(); i++) {
        ibses[i]->plotFeatures(path+"IBSFeat_"+ibses[i]->ibsObj->getName()+".png");
    }
    //std::sort(ibses.begin(),ibses.end(),compareIBS);
    writeSimilarities(ibses,path+std::string("ibsSimilarities"));

    // Display the result
    Display::display(m_objects,path+std::string("ibses.blend"),false);
}

std::string ExpIBSSim::getExpPath()
{
  std::string folderName = std::string(IbsSampleScheme::getSampleSchemeName(m_sampleScheme));
  if(m_onePass)
    folderName = "OneVoronoi_"+folderName;
  return EXPTYPE+std::string("/")+folderName+"/"+m_ID;

}

