#include "ExpIBSPDF.h"
#include <iostream> 
#include <boost/algorithm/string.hpp>
#include "../Debug/DebugTools.h"
#include "../AnalysisPhase/IbsGenerator.h"
#include "../Display/Display.h"
#include "GA.h"
#include "IBSFitEval.h"

#define EXPTYPE "Pdf"
#define MAX_STEPS_PER_DIMENSION 20

std::vector<std::shared_ptr<IBS>> ExpIBSPDF::compute(std::pair<std::vector<std::shared_ptr<Object>>,std::vector<std::shared_ptr<Object>>> sets){
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

    std::shared_ptr<Object> tableObj(new Object("Table", sets.second[0]->getMesh()));
    std::vector<Object> sceneObjects;
    Eigen::Vector4d centroidNewObj = sets.first[0]->getCentroid();
    sceneObjects.push_back(*tableObj);
    IsoCub3d bboxCuboid = IsoCub3d(CGAL::bbox_3(std::begin(sceneObjects),std::end(sceneObjects)));
    double min[3];
    double max[3];
    min[0] = bboxCuboid.xmin()-((bboxCuboid.xmax()-bboxCuboid.xmin())/2);
    min[2] = bboxCuboid.zmin()-((bboxCuboid.zmax()-bboxCuboid.zmin())/2);
    min[1] = centroidNewObj(1);
    max[0] = bboxCuboid.xmax()+((bboxCuboid.xmax()-bboxCuboid.xmin())/2);
    max[1] = centroidNewObj(1);
    max[2] = bboxCuboid.zmax()+((bboxCuboid.zmax()-bboxCuboid.zmin())/2);


    double largestDimDiff = max[0]-min[0];
    if((max[1]-min[1]) > (max[2]-min[2]) && (max[1]-min[1]) > (max[0]-min[0]))
        largestDimDiff = max[1]-min[1];
    else if((max[2]-min[2]) > (max[1]-min[1]) && (max[2]-min[2]) > (max[0]-min[0]))
        largestDimDiff = max[2]-min[2];
    double step[] = {std::min(largestDimDiff/MAX_STEPS_PER_DIMENSION,max[0]-min[0]),std::min(largestDimDiff/MAX_STEPS_PER_DIMENSION,max[1]-min[1]),std::min(largestDimDiff/MAX_STEPS_PER_DIMENSION,max[2]-min[2])};
    DebugLogger::ss << "Min-Max X: (" << min[0] << "," << max[0] << ")" << std::endl;
    DebugLogger::ss << "Step X: (" << step[0] << ")" << std::endl;
    DebugLogger::ss << "Min-Max Z: (" << min[2] << "," << max[2] << ")" << std::endl;
    DebugLogger::ss << "Step Z: (" << step[2] << ")" << std::endl;
    DebugLogger::log();

    std::vector<double> similarityValues;
    int amountX = 0;
    int amountZ = 0;
    int i = 0;
    Scene tmp;
    tmp.addObject(tableObj);
    for(double x = min[0]; x < max[0]; x += step[0], amountX++) 
    {
        amountZ = 0;
        for(double z = min[2]; z < max[2]; z += step[2], amountZ++) 
        { 
            std::ostringstream oss;
            oss << "Chair" << i++;
            std::shared_ptr<Object> chairObj(new Object(oss.str(), sets.first[0]->getMesh()));
            chairObj->setPosition(x,min[1],z);
            tmp.addObject(chairObj);
            tmp.samplePoints();

            // Generate IBS between other object and this.
            if(m_sampleScheme == IbsSampleScheme::SampleScheme::IMPORTANCE_DISTANCE)
            {
                tableObj->sampleNonUniform(tableObj->getUniformSamples().size(),chairObj);
                chairObj->sampleNonUniform(chairObj->getUniformSamples().size(),tableObj);
            }

            IbsGenerator ibsGen;
            std::vector<std::shared_ptr<IBS>> tmpIbses = ibsGen.computeIBS(std::vector<std::shared_ptr<Object>>({chairObj,tableObj}));
            assert(tmpIbses.size() == 1);

            std::shared_ptr<IBS> ibsPtr = tmpIbses[0];
            for(std::shared_ptr<IBS> ibs : tmpIbses) {
                ibs->computeGeomFeatures();
                ibs->computeTopoFeatures();
            }

            // Calculate similarity
            double similarity = ibses[0]->getSimilarity(*tmpIbses[0]);
            for(int i = 1; i < ibses.size(); i++) 
            {
                double tmpSimilarity = ibses[i]->getSimilarity(*tmpIbses[0]);
                if (tmpSimilarity > similarity)
                {
                    similarity = tmpSimilarity;
                }
            }
            similarityValues.push_back(similarity);
            tmp.removeObject(chairObj);
        }
    }

    DebugLogger::ss << "Steps in Z: " << amountZ;
    DebugLogger::ss << "Steps in X: " << amountX;
    DebugLogger::log();

    std::shared_ptr<Mesh3d> mesh3d(new Mesh3d());

    // add vertices
    std::vector<Mesh3d::Vertex_index> cgal_vertIdxes;
    for(int xId = 0; xId < amountX; xId++) 
    {
        for(int zId = 0; zId < amountZ; zId++) 
        { 
            Point3d vert(min[0]+xId*step[0],similarityValues[zId + xId*amountZ],min[2]+zId*step[2]);
            cgal_vertIdxes.push_back(mesh3d->add_vertex(vert));
        }
    }

    // add faces
    Face f;
    Face f2;
    bool created;
    FaceProperty<Vector3d> faceNormals;
    boost::tie(faceNormals, created) = mesh3d->add_property_map<Face,Vector3d>("f:normal");
    VertexProperty<Point3d> points = mesh3d->points();
    for(int xId = 0; xId < amountX-1; xId++) 
    {
        for(int zId = 0; zId < amountZ-1; zId++) 
        { 
            f = mesh3d->add_face(cgal_vertIdxes[zId + xId*amountZ], 
                                 cgal_vertIdxes[(zId + 1) + xId*amountZ], 
                                 cgal_vertIdxes[zId + (xId + 1)*amountZ]);
            f2 = mesh3d->add_face(cgal_vertIdxes[(zId + 1) + xId*amountZ], 
                                 cgal_vertIdxes[(zId + 1) + (xId + 1)*amountZ], 
                                 cgal_vertIdxes[zId + (xId + 1)*amountZ]);
            if (f != Mesh3d::null_face())
                faceNormals[f] = CGAL::normal(points[cgal_vertIdxes[zId + xId*amountZ]], 
                                 points[cgal_vertIdxes[(zId + 1) + xId*amountZ]], 
                                 points[cgal_vertIdxes[zId + (xId + 1)*amountZ]]);
            if (f2 != Mesh3d::null_face())
                faceNormals[f2] = CGAL::normal(points[cgal_vertIdxes[(zId + 1) + xId*amountZ]], 
                                 points[cgal_vertIdxes[(zId + 1) + (xId + 1)*amountZ]], 
                                 points[cgal_vertIdxes[zId + (xId + 1)*amountZ]]);

            assert(f != Mesh3d::null_face());
            assert(f2 != Mesh3d::null_face());
        }
    }

    std::shared_ptr<Object> pdfObj(new Object("PDFMesh", Mesh(mesh3d)));
    m_objects.push_back(pdfObj);

    return ibses;
}

void ExpIBSPDF::output(std::vector<std::shared_ptr<IBS>> ibses, std::string path){
    //for(int i = 0; i < ibses.size(); i++) {
        //ibses[i]->plotFeatures(path+"IBSFeat_"+ibses[i]->ibsObj->getName()+".png");
    //}
    //std::sort(ibses.begin(),ibses.end(),compareIBS);
    //writeSimilarities(ibses,path+std::string("ibsSimilarities"));

    // Display the result
    Display::display(m_objects,path+std::string("pdf.blend"),false);
}

/* std::string ExpIBSPDF::getExpPath(){ */
/*   std::string folderName = std::string(IbsSampleScheme::getSampleSchemeName(m_sampleScheme)); */
/*   if(m_onePass) */
/*     folderName = "OneVoronoi_"+folderName; */
/*   return EXPTYPE+std::string("/")+folderName+"/"+m_ID; */

/* } */

