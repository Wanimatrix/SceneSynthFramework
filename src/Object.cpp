/* 
 * Object.cpp
 *
 * Author: Wouter Franken
 *
 * CONTENT: Object class
 *   - Object class implementation:
 *      + CON-/DE-STRUCTORS
 *      + INIT FUNCTIONS
 *      + SAMPLING FUNCTIONS
 */

#include "Object.h"

#include <assert.h>
#include <iostream>
#include <CGAL/bounding_box.h>
#include <CGAL/centroid.h>
#include <igl/copyleft/cgal/intersect_other.h>
#include "AnalysisPhase/Sampler.h"
#include <memory>
#include <limits>
#include <boost/algorithm/string.hpp>
#include "Debug/DebugTools.h"
#include "Utilities/split_range.hpp"
#include "Utilities/Utilities.h"
#include "Experiments/Configuration.h"
#include "Display/write.h"

/***********************
 *  CON-/DE-STRUCTORS  *
 ***********************/
Object::Object(const aiMesh *t_mesh, std::stack<Transformation> t_transformationStack/*const aiMatrix4x4 t_transformation*/) : m_mesh(init(t_mesh,t_transformationStack,std::string(t_mesh->mName.C_Str())))
{
    //m_transformation << t_transformation.a1, t_transformation.a2, t_transformation.a3, t_transformation.a4,
                        //t_transformation.b1, t_transformation.b2, t_transformation.b3, t_transformation.b4,
                        //t_transformation.c1, t_transformation.c2, t_transformation.c3, t_transformation.c4,
                        //t_transformation.d1, t_transformation.d2, t_transformation.d3, t_transformation.d4;
    DebugLogger::ss << "Init done, applying transformationstack";
    DebugLogger::log();
    m_transformationStack = t_transformationStack;
    applyCurrentTransformationStack();
    initBbox();
    initCentroid();
    //initEigenMesh();
}

Object::Object(const std::string &t_name, Mesh t_mesh, std::stack<Transformation> t_transformationStack/*const aiMatrix4x4 t_transformation*/) : m_name(t_name), m_mesh(t_mesh.cloneMesh()), m_transformationStack(t_transformationStack)
{
    //std::shared_ptr<Mesh3d> mesh3d = t_mesh.mesh3d;
    //bool exists;
    //VertexProperty<Vector3d> vertexNormals;
    //VertexProperty<Point3d> points;
    //std::vector<Point3d> pointVec;
    //boost::tie(vertexNormals, exists) = mesh3d->property_map<Vertex,Vector3d>("v:normal");
    //points = mesh3d->points();

    //for(Vertex v : mesh3d->vertices()) {
        //Point3d p = mesh3d->point(v);
        //aiVector3D translatedP(p.x(),p.y(),p.z());
        //translatedP = aiMatrix4x4(t_transformation)*translatedP;
        //p = Point3d(translatedP[0],translatedP[1],translatedP[2]);
        //points[v] = p;

        //Vector3d vn = vertexNormals[v];
        //aiVector3D translatedVn(vn.x(),vn.y(),vn.z());
        //translatedVn = aiMatrix3x3(t_transformation)*translatedVn;
        //vn = Vector3d(translatedVn[0],translatedVn[1],translatedVn[2]);
        //vertexNormals[v] = vn;
    //}
    //m_transformation << t_transformation.a1, t_transformation.a2, t_transformation.a3, t_transformation.a4,
                        //t_transformation.b1, t_transformation.b2, t_transformation.b3, t_transformation.b4,
                        //t_transformation.c1, t_transformation.c2, t_transformation.c3, t_transformation.c4,
                        //t_transformation.d1, t_transformation.d2, t_transformation.d3, t_transformation.d4;
    m_transformationStack = t_transformationStack;
    applyCurrentTransformationStack();
    initBbox();
    initCentroid();
    //initEigenMesh();
}

Object::~Object() 
{}

/********************
 *  INIT FUNCTIONS  *
 ********************/
Mesh Object::init(const aiMesh *t_mesh, std::stack<Transformation> t_transformationStack, const std::string &t_name) 
{
    DebugLogger::ss << "Init object...";
    DebugLogger::log();
    m_name = t_name;
    //m_eigenMesh.vertices(t_mesh->mNumVertices,3);
    //m_eigenMesh.faces(t_mesh->mNumFaces,3);
    std::shared_ptr<Mesh3d> mesh3d(new Mesh3d());
    
    DebugLogger::ss << "Adding vertices...";
    DebugLogger::log();
    // Add vertices and vertexNormals to mesh
    bool created;
    VertexProperty<Vector3d> vertexNormals;
    boost::tie(vertexNormals, created) = mesh3d->add_property_map<Vertex,Vector3d>("v:normal");
    std::vector<Vertex> vertexIdxes;
    for (int i = 0; i < t_mesh->mNumVertices; ++i)
    {
        aiVector3D point = t_mesh->mVertices[i]/**t_transf*/;
        aiVector3D meshNormal = t_mesh->mNormals[i]/**aiMatrix3x3(t_transf)*/;

        Point3d p = Point3d(point[0],point[1],point[2]);
        Vertex v_id = mesh3d->add_vertex(p);
        //m_eigenMesh.vertices(v_id,0) = point[0];
        //m_eigenMesh.vertices(v_id,1) = point[1];
        //m_eigenMesh.vertices(v_id,2) = point[2];
        vertexIdxes.push_back(v_id);
        vertexNormals[v_id] = Vector3d(meshNormal[0],meshNormal[1],meshNormal[2]);
    }

    assert(vertexIdxes.size()==t_mesh->mNumVertices);

    DebugLogger::ss << "Adding faces...";
    DebugLogger::log();
    // Add faces to mesh
    for (int f = 0; f < t_mesh->mNumFaces; ++f)
    {
        std::vector<Vertex> faceVertIdxes;
        Face f_id;
        for (int i = 0; i < t_mesh->mFaces[f].mNumIndices; ++i)
        {
            faceVertIdxes.push_back(vertexIdxes[t_mesh->mFaces[f].mIndices[i]]);
        }
        if(t_mesh->mFaces[f].mNumIndices == 3)
            f_id = mesh3d->add_face(faceVertIdxes[0],faceVertIdxes[1],faceVertIdxes[2]);
        else
        { // No support for quads!!!
            //f_id = mesh3d->add_face(faceVertIdxes[0],faceVertIdxes[1],faceVertIdxes[2],faceVertIdxes[3]);
            assert(false);
        }
        //m_eigenMesh.faces(f_id,0) = faceVertIdxes[0];
        //m_eigenMesh.faces(f_id,1) = faceVertIdxes[1];
        //m_eigenMesh.faces(f_id,2) = faceVertIdxes[2];
    }

    return Mesh(mesh3d);
}

void Object::initBbox(std::vector<Point3d> pointVec) 
{
    if(pointVec.size() == 0) {
        pointVec = getObjectPoints();
    }

    Bbox3d bbox = CGAL::bounding_box(std::begin(pointVec),std::end(pointVec)).bbox();
    m_bbox << bbox.xmin(), bbox.xmax(), 
              bbox.ymin(), bbox.ymax(), 
              bbox.zmin(), bbox.zmax(),
              1          , 1          ;
    /* DebugLogger::ss << "Bbox: " << m_bbox; */
    /* DebugLogger::log(); */
}

void Object::initCentroid(std::vector<Point3d> pointVec)
{
    if(pointVec.size() == 0) {
        pointVec = getObjectPoints();
    }

    Point3d centroid = CGAL::centroid(pointVec.begin(), pointVec.end(), CGAL::Dimension_tag<0>());
    m_centroid(0) = centroid.x();
    m_centroid(1) = centroid.y();
    m_centroid(2) = centroid.z();
    m_centroid(3) = 1;
}

//void Object::initEigenMesh()
//{
    //std::shared_ptr<Mesh3d> mesh3dPtr = m_mesh.mesh3d;

    //// Init eigenmesh matrices
    //m_eigenMesh.vertices.resize(mesh3dPtr->vertices().size(),3);
    //m_eigenMesh.faces.resize(mesh3dPtr->faces().size(),3);

    //// Store vertices in eigenMesh
    //for(Vertex v : mesh3dPtr->vertices())
    //{
        //Point3d point = mesh3dPtr->point(v);
        //m_eigenMesh.vertices(v,0) = point.x();
        //m_eigenMesh.vertices(v,1) = point.y();
        //m_eigenMesh.vertices(v,2) = point.z();
    //}

    //// Store faces in eigenMesh
    //for(Face f : mesh3dPtr->faces())
    //{
        //std::vector<Vertex> faceVertices = m_mesh.getVerticesOfFace(f);
        //m_eigenMesh.faces(f,0) = faceVertices[0];
        //m_eigenMesh.faces(f,1) = faceVertices[1];
        //m_eigenMesh.faces(f,2) = faceVertices[2];
    //}
//}

/******************************
 *  TRANSFORMATION FUNCTIONS  *
 ******************************/

void Object::translate(double x, double y, double z) 
{
    Eigen::Matrix4d translMat = Eigen::Matrix4d::Identity();
    translMat(0,3) = x;
    translMat(1,3) = y;
    translMat(2,3) = z;
    applyTransformation(translMat);
    Transformation tmpTransf(translMat,TransformationType::TRANSLATION);
    m_transformationStack.push(tmpTransf);

    m_centroid(0) += x;
    m_centroid(1) += y;
    m_centroid(2) += z;
}

void Object::undoLatestTransformation(std::stack<Transformation> &theStack) 
{
    Eigen::Matrix4d latestInverse = theStack.top().getInverseMatrix();
    applyTransformation(latestInverse);
    theStack.pop();
}

void Object::undoTransformationStack() 
{
    std::stack<Transformation> stackCopy = m_transformationStack;
    while(!stackCopy.empty()) 
    {
        undoLatestTransformation(stackCopy);
    }
}

void Object::clearTransformationStack()
{
    while(!m_transformationStack.empty())
        m_transformationStack.pop();
}

void Object::setPosition(double x, double y, double z)
{
    Eigen::Vector4d backToOrigin = -1*m_centroid;
    double translateX = backToOrigin(0)+x;
    double translateY = backToOrigin(1)+y;
    double translateZ = backToOrigin(2)+z;
    translate(translateX,translateY,translateZ);
}

std::vector<Point3d> Object::getObjectPoints()
{
    std::vector<Point3d> pointVec;
    std::shared_ptr<Mesh3d> mesh3dPtr = m_mesh.mesh3d;
    for(Vertex v : mesh3dPtr->vertices()) {
        Point3d p = mesh3dPtr->point(v);
        pointVec.push_back(p);
    }

    return pointVec;
}

void Object::applyCurrentTransformationStack() 
{
    Eigen::Matrix4d transformation = Transformation::getTransformationMatrix(m_transformationStack);
    applyTransformation(transformation);
}

void Object::applyTransformation(Eigen::Matrix4d t_transformation) 
{
    std::shared_ptr<Mesh3d> mesh3d = m_mesh.mesh3d;
    bool exists;
    VertexProperty<Vector3d> vertexNormals;
    VertexProperty<Point3d> points;
    std::vector<Point3d> pointVec;
    boost::tie(vertexNormals, exists) = mesh3d->property_map<Vertex,Vector3d>("v:normal");
    points = mesh3d->points();

    for(Vertex v : mesh3d->vertices()) {
        Point3d p = mesh3d->point(v);
        Eigen::Vector4d pEig(p.x(),p.y(),p.z(),1);
        pEig = t_transformation*pEig;
        p = Point3d(pEig[0],pEig[1],pEig[2]);
        points[v] = p;

        Vector3d vn = vertexNormals[v];
        Eigen::Vector4d vnEig(vn.x(),vn.y(),vn.z(),0);
        vnEig = t_transformation*vnEig;
        vn = Vector3d(vnEig[0],vnEig[1],vnEig[2]);
        vertexNormals[v] = vn;
    }
    initBbox();
}

/****************************
 *  INTERSECTION FUNCTIONS  *
 ****************************/
bool Object::intersects(std::shared_ptr<Object> other)
{
    //EigenMesh thisEigMesh = m_eigenMesh;
    //EigenMesh otherEigMesh = other->getEigenMesh();
    Eigen::MatrixXi result;

    //return true;
    return igl::copyleft::cgal::intersect_other(m_mesh.getVertices(), 
                                                m_mesh.getFaces(), 
                                                other->getMesh().getVertices(), 
                                                other->getMesh().getFaces(), true, result);
}

/***********************
*  SAMPLING FUNCTIONS  *
************************/
void Object::sampleUniform(int num)
{
    Sampler s(m_mesh);
    m_uniformSamples = s.getSamples(num, 0);

    m_uniformSampleDensity = static_cast<double>(num) / m_mesh.getSurfaceArea();
}

void Object::sampleNonUniform(int num, std::shared_ptr<Object> other, bool convexHullApprox)
{
    DebugLogger::ss << "Sampling started.";
    DebugLogger::log();
    sampleNonUniformRays(num,other);
    return;
    
    std::vector<Object> objs({*this,*other});
    FaceProperty<double> fweight;
    FaceProperty<Point3d> fpt;
    VertexProperty<double> vweight;
    bool exists;
    //IsoCub3d bboxCuboid = IsoCub3d(CGAL::bbox_3(std::begin(objs),std::end(objs)));
    //Poly3d convHullOther; 
    //if(convexHullApprox) convHullOther = other->getConvexHull();
    boost::tie(fweight, exists) = m_mesh.mesh3d->property_map<Face,double>("f:weight");
    if(!exists)
      fweight = m_mesh.mesh3d->add_property_map<Face,double>("f:weight", 0).first;
    boost::tie(fpt, exists) = m_mesh.mesh3d->property_map<Face,Point3d>("f:pt");
    if(!exists)
      fpt = m_mesh.mesh3d->add_property_map<Face,Point3d>("f:pt", Point3d(0,0,0)).first;
    //boost::tie(vweight, exists) = m_mesh.mesh3d->property_map<Vertex,double>("v:weight");
    //if(!exists)
      //vweight = m_mesh.mesh3d->add_property_map<Vertex,double>("v:weight", 0).first;
    //double distThreshold = std::sqrt((bboxCuboid.max()-bboxCuboid.min()).squared_length()) * 0.5;
    std::vector<SamplePoint> otherSamples = other->getUniformSamples();
    DebugLogger::ss << "Faces to loop over: " << m_mesh.mesh3d->faces().size() << std::endl;
    DebugLogger::ss << "Sample amount: " << otherSamples.size();
    DebugLogger::log();

    DebugTimer t;
    t.start();
    #pragma omp parallel
    {
        //boost::iterator_range<Mesh3d::Face_range::iterator> faceRange = 
            //parallelism::split_range_openmp(m_mesh.mesh3d->faces(),m_mesh.mesh3d->number_of_faces());

        //for(Mesh3d::Face_range::iterator fit = boost::begin(faceRange);
            //fit != boost::end(faceRange);
            //++fit) {
        int samplesSize = 0;
        Poly3d::Vertex_iterator vIter;
        Point3d otherP;
        double smallestDistance;
        double currentDistance;
        //if(convexHullApprox) {
            //samplesSize = convHullOther.size_of_vertices();
            //vIter = convHullOther.vertices_begin();
        //} else {
        samplesSize = otherSamples.size();
        std::shared_ptr<Mesh3d> otherMesh = other->getMesh().mesh3d;
        Mesh3d::Vertex_range otherVertices = otherMesh->vertices();
        //}
        //Mesh3d::Vertex_range::iterator ve, vit;
        //for(boost::tie(vit,ve) = m_mesh.mesh3d->vertices(); vit != ve; vit++) {
          //Vertex v_id = *vit;
          //smallestDistance = std::numeric_limits<double>::infinity();
          //for (int i = 0; i < samplesSize; i++) {
            //otherP = otherSamples[i].pos;
            //currentDistance += std::sqrt(CGAL::squared_distance(m_mesh.mesh3d->point(v_id),otherP));
          //}
          //vweight[v_id] = 1/currentDistance;
        //}
        
        boost::iterator_range<Mesh3d::Face_range::iterator> faceRange = 
            parallelism::split_range_openmp(m_mesh.mesh3d->faces(),m_mesh.mesh3d->number_of_faces());
        for(Mesh3d::Face_range::iterator fit = boost::begin(faceRange);
            fit != boost::end(faceRange);
            ++fit) {
        //Mesh3d::Face_range::iterator fe, fit;
        //for(boost::tie(fit,fe) = m_mesh.mesh3d->faces(); fit != fe; fit++) 
        //{
            Face f_id = *fit;
            std::vector<Vertex> faceVertices = m_mesh.getVerticesOfFace(f_id);
            //#pragma omp critical 
            fweight[f_id] = 0;
            double avgDistance;
            smallestDistance = std::numeric_limits<double>::infinity();
            /* for (int i = 0; i < samplesSize; i++) */ 
            /* { */
            for (Mesh3d::Vertex_range::iterator vit = otherVertices.begin(); vit != otherVertices.end(); vit++) 
            {
              /* otherP = otherSamples[i].pos; */
              otherP = otherMesh->point(*vit);
              
              avgDistance = 0;
              for(Vertex v : faceVertices) 
              {
                  avgDistance += std::sqrt(CGAL::squared_distance(m_mesh.mesh3d->point(v),otherP));
              }
              avgDistance /= 3.0;
              if(avgDistance < smallestDistance) 
              {
                smallestDistance = avgDistance;
                //#pragma omp critical 
                fpt[f_id] = otherP;
              }
            }
            //#pragma omp critical 
            fweight[f_id] = std::pow(1/smallestDistance,20);
            //smallestDistance = std::numeric_limits<double>::infinity();
            //double avgDistance;
            //for (int i = 0; i < samplesSize; i++) {
                //if(convexHullApprox)
                    //otherP = vIter->point();
                //else
                    //otherP = otherSamples[i].pos;
                //avgDistance = 0;
                //for(Vertex v : faceVertices) {
                    //avgDistance += std::sqrt(CGAL::squared_distance(m_mesh.mesh3d->point(v),otherP));
                    //m_mesh.getVerticesOfFace(f_id);
                //}
                //if(avgDistance / 3.0 < smallestDistance)
                    //smallestDistance = avgDistance / 3.0;
                //if(convexHullApprox) 
                    //++vIter;
            //}
            //if(getName() == "Plane") {
              //DebugLogger::ss << "SmallestDistance: " << smallestDistance << ", threshold: " << distThreshold;
              //DebugLogger::log();
            //}
            ////#pragma omp critical 
            //{
                ////fweight[f_id] =  (smallestDistance <= distThreshold)? pow(1 - (smallestDistance / distThreshold), 20) : 0;
                //fweight[f_id] = 1/smallestDistance;
            //}
        }
    }
    t.stop("Sampling");
    DebugLogger::ss << "Sampling done.";
    DebugLogger::log();

    if(getName() == "Plane") 
    {
        Mesh3d::Face_range::iterator fe, fit, ftmp;
        DebugLogger::ss << "New PlaneWeights : ";
        for(boost::tie(fit,fe) = m_mesh.mesh3d->faces(); fit != fe; fit++) {
            ftmp = fit;
            DebugLogger::ss << fweight[*fit] << (++ftmp != fe ? ", " : "");
        }
        DebugLogger::log();
    }

    t.start();
    Sampler s(m_mesh, RANDOM_BARYCENTRIC_WEIGHTED_DISTANCE);
    m_nonUniformSamples[other] = s.getSamples(num, 0);
    t.stop("Actual sampling");
    DebugLogger::ss << "Amount of sample collections: " << m_nonUniformSamples.size() << std::endl;
    DebugLogger::ss << "Sampling on " << getName() << " and accounting distance to " << other->getName();
    DebugLogger::log();

    m_nonUniformSampleDensity[other] = static_cast<double>(num) / m_mesh.getSurfaceArea();
}

/* typedef struct Ray */ 
/* { */
/*     Point3d s; */
/*     Vector3d d; */
/* } Ray; */

bool intersection(Bbox3d b, Ray3d r) {
    double tmin = -INFINITY, tmax = INFINITY;
         
    if (r.direction().dx() != 0.0) {
        double tx1 = (b.xmin() - r.source().x())/r.direction().dx();
        double tx2 = (b.xmax() - r.source().x())/r.direction().dx();
                         
        tmin = std::max(tmin, std::min(tx1, tx2));
        tmax = std::min(tmax, std::max(tx1, tx2));
    }
             
    if (r.direction().dy() != 0.0) {
        double ty1 = (b.ymin() - r.source().y())/r.direction().dy();
        double ty2 = (b.ymax() - r.source().y())/r.direction().dy();
                                     
        tmin = std::max(tmin, std::min(ty1, ty2));
        tmax = std::min(tmax, std::max(ty1, ty2));
    }

    if (r.direction().dz() != 0.0) {
        double tz1 = (b.zmin() - r.source().z())/r.direction().dz();
        double tz2 = (b.zmax() - r.source().z())/r.direction().dz();
                                     
        tmin = std::max(tmin, std::min(tz1, tz2));
        tmax = std::min(tmax, std::max(tz1, tz2));
    }
                 
    return tmax >= tmin;
}

std::pair<Point3d,int> uniformSampleBbox(Bbox3d bbox, int sideConstraint)
{
    assert(sideConstraint >= -1 && sideConstraint <= 5);

    double dim[] = {std::abs(bbox.xmax()-bbox.xmin()),std::abs(bbox.ymax()-bbox.ymin()),std::abs(bbox.zmax()-bbox.zmin())};
    double areas[] = {dim[1]*dim[2],dim[0]*dim[2],dim[0]*dim[1]};
    double totAreas = areas[0]+areas[1]+areas[2];
    double unifAreas[] = {areas[0]/totAreas,areas[1]/totAreas,areas[2]/totAreas};
    /* double totDim = dim[0]+dim[1]+dim[2]; */
    /* double unifDim[] = {dim[0]/totDim,dim[1]/totDim,dim[2]/totDim}; */
    int side = 0;
    do
    {
        double randDimChooser = utilities::uniformDouble(0,1);
        if(randDimChooser < unifAreas[0])
        {
            side = (randDimChooser < 0.5*unifAreas[0]) ? 0 : 1;
        } else if(randDimChooser < unifAreas[0]+unifAreas[1])
        {
            side = (randDimChooser < unifAreas[0]+0.5*unifAreas[1]) ? 2 : 3;
        } else 
        {
            side = (randDimChooser < 1-0.5*unifAreas[2]) ? 4 : 5;
        }
    } while (side == sideConstraint);

    double rand1 = utilities::uniformDouble(0,dim[(side/2+1)%3]);
    double rand2 = utilities::uniformDouble(0,dim[(side/2+2)%3]);
    Point3d result;
    if(side/2 == 0)
    {
        result = Point3d((side == 0) ? bbox.xmin() : bbox.xmax(), rand1+bbox.ymin(), rand2+bbox.zmin());
    } else if(side/2 == 1)
    {
        result = Point3d(rand2+bbox.xmin(), (side == 2) ? bbox.ymin() : bbox.ymax(), rand1+bbox.zmin());
    } else
    {
        result = Point3d(rand1+bbox.xmin(), rand2+bbox.ymin(), (side == 4) ? bbox.zmin() : bbox.zmax());
    }

    return std::make_pair(result,side);
}

// IDEA from: [Schmitt2015] A 3D Shape Descriptor based on Depth Complexity
// and Thickness Histograms;
void Object::sampleNonUniformRays(int num, std::shared_ptr<Object> other)
{
    std::vector<SamplePoint> theseSamples;
    std::vector<SamplePoint> otherSamples;

    std::vector<Point3d> sources;
    std::vector<Point3d> targets;

    Bbox3d thisBbox = getBbox();
    Bbox3d otherBbox = other->getBbox();
    Bbox3d totalBbox = thisBbox+otherBbox;

    /* DebugLogger::ss << "This bbox: " << thisBbox << std::endl; */
    /* DebugLogger::ss << "Other bbox: " << otherBbox << std::endl; */
    /* DebugLogger::ss << "Total bbox: " << totalBbox << std::endl; */
    /* DebugLogger::log(); */

    int j = 0;
    while (theseSamples.size() < num || otherSamples.size() < num)
    {
        /* DebugLogger::ss << "WHILE " << j++ << "; samplesSize: " << samples.size() << "; NAME " << m_name << std::endl; */
        /* DebugLogger::log(); */
        std::pair<Point3d,int> sampleResult = uniformSampleBbox(totalBbox, -1);
        Point3d r_s = sampleResult.first;
        std::pair<Point3d,int> sampleResultSecond = uniformSampleBbox(totalBbox, sampleResult.second);
        Point3d r_t = sampleResultSecond.first;

        /* DebugLogger::ss << "Source on side " << sampleResult.second << ": (" << r_s << ")" << std::endl; */
        /* DebugLogger::ss << "Target on side " << sampleResultSecond.second << ": (" << r_t << ")"; */
        /* DebugLogger::log(); */

        Ray3d r(r_s,r_t);
        /* DebugLogger::ss << "Ray: " << r; */
        /* DebugLogger::log(); */
        std::vector<std::pair<Point3d,std::shared_ptr<Object>>> intersections;
        if(intersection(thisBbox,r) && intersection(otherBbox,r))
        {
            Mesh3d::Face_range thisFaces = m_mesh.mesh3d->faces();
            Mesh3d::Face_range otherFaces = other->getMesh().mesh3d->faces();
            std::shared_ptr<Mesh3d> currMesh = m_mesh.mesh3d;
            std::shared_ptr<Object> currObject = shared_from_this();
            int i = 0;
            for(Mesh3d::Face_range::iterator fit = thisFaces.begin();
                fit != otherFaces.end(); fit++, i++) 
            {
                Face f_id = *fit;

                /* DebugLogger::ss << "i: " << i << std::endl; */
                /* DebugLogger::ss << "Current fid: " << *fit << std::endl; */
                /* /1* DebugLogger::ss << "This (" << m_name << ") number of faces: " << m_mesh.mesh3d->number_of_faces() << std::endl; *1/ */
                /* DebugLogger::ss << "Other (" << other->getName() << ") number of faces: " << other->getMesh().mesh3d->number_of_faces() << std::endl; */
                /* DebugLogger::log(); */

                std::vector<Point3d> faceVertices;
                BOOST_FOREACH(Vertex vd,vertices_around_face(currMesh->halfedge(f_id), *currMesh))
                {
                    faceVertices.push_back(currMesh->point(vd));
                }
                if(faceVertices.size() == 3)
                {
                    /* for (int k = 0; k < 3; k++) */
                    /* { */
                    /*     if(std::abs(std::abs(faceVertices[k].x())-1) < 0.00001 ) */
                    /*         faceVertices[k] = Point3d(faceVertices[k].x() < 0 ? -1 : 1, faceVertices[k].y(), faceVertices[k].z()); */
                    /*     if(std::abs(std::abs(faceVertices[k].y())-1) < 0.00001 ) */
                    /*         faceVertices[k] = Point3d(faceVertices[k].x(), faceVertices[k].y() < 0 ? -1 : 1, faceVertices[k].z()); */
                    /*     if(std::abs(std::abs(faceVertices[k].z())-1) < 0.00001 ) */
                    /*         faceVertices[k] = Point3d(faceVertices[k].x(), faceVertices[k].y(), faceVertices[k].z() < 0 ? -1 : 1); */
                    /* } */
                    Triangle3d t(faceVertices[0], faceVertices[1], faceVertices[2]);
                    /* DebugLogger::ss << "Triangle Pts: (" << faceVertices[0] << "), (" << faceVertices[1] << "), (" << faceVertices[2] << ")" << std::endl; */
                    /* DebugLogger::ss << "Triangle area: " << std::sqrt(t.squared_area()) << std::endl; */
                    /* DebugLogger::log(); */
                    if(std::sqrt(t.squared_area()) >= 0.000001)
                    {
                        auto result = CGAL::intersection(r,t);
                        if(result)
                        {
                            sources.push_back(r_s);
                            targets.push_back(r_t);
                            if (const Segment3d* s = boost::get<Segment3d>(&*result)) {
                                intersections.push_back(std::make_pair(s->source(),currObject));
                                intersections.push_back(std::make_pair(s->target(),currObject));
                            } else {
                                const Point3d* p = boost::get<Point3d >(&*result);
                                intersections.push_back(std::make_pair(*p,currObject)); 
                            }
                        }
                        else 
                        {
                            /* DebugLogger::ss << "No real intersection, triangle: " << t; */
                            /* DebugLogger::log(); */
                        }
                    }
                    else
                    {
                        /* DebugLogger::ss << "Triangle area too small: " << t.squared_area(); */
                        /* DebugLogger::log(); */
                    }
                }
                else
                {
                    /* DebugLogger::ss << "Face has more than 3 vertices"; */
                    /* DebugLogger::log(); */
                }

                if(i == m_mesh.mesh3d->number_of_faces()-1)
                {
                    /* DebugLogger::ss << "Switch to other"; */
                    /* DebugLogger::log(); */
                    fit = otherFaces.begin();
                    fit--;
                    currMesh = other->getMesh().mesh3d;
                    currObject = other;
                }

                /* if((fit >= thisFaces.begin() && fit < thisFaces.end()) || (fit >= otherFaces.begin() && fit < otherFaces.end())) */
                /*     fit++; */
                /* if(fit == thisFaces.end()) */
                /* { */
                /*     fit = otherFaces.begin(); */
                /*     currMesh = other->getMesh().mesh3d; */
                /*     currObject = other; */
                /* } */
            }

            if(intersections.size() > 0)
            {

                /* DebugLogger::ss << "IntersectionSet BEFORE {" << std::endl; */
                /* for (std::pair<Point3d,std::shared_ptr<Object>> intersect : intersections) */
                /* { */
                /*     double t = (intersect.first.x()-r.source().x()) / r.direction().vector().x(); */
                /*     DebugLogger::ss << "    t: " << t << " on obj: " << intersect.second << std::endl; */
                /* } */
                /* DebugLogger::ss << "}" << std::endl; */
                /* DebugLogger::log(); */
                std::sort(intersections.begin(), intersections.end(), [&r] (const std::pair<Point3d,std::shared_ptr<Object>> & a, const std::pair<Point3d,std::shared_ptr<Object>> & b) -> bool            
                        {
                            double ta = (a.first.x()-r.source().x()) / r.direction().vector().x();
                            double tb = (b.first.x()-r.source().x()) / r.direction().vector().x();
                            return ta < tb;
                        });
                /* DebugLogger::ss << "IntersectionSet AFTER {" << std::endl; */
                /* for (std::pair<Point3d,std::shared_ptr<Object>> intersect : intersections) */
                /* { */
                /*     double t = (intersect.first.x()-r.source().x()) / r.direction().vector().x(); */
                /*     DebugLogger::ss << "    t: " << t << " on obj: " << intersect.second << std::endl; */
                /* } */
                /* DebugLogger::ss << "}" << std::endl; */
                /* DebugLogger::log(); */
                auto prevInt = intersections[0];
                auto currInt = prevInt;
                bool prevIsSample = false;
                for (int i = 1; i < intersections.size(); i++)
                {
                    currInt = intersections[i];

                    if(currInt.second != prevInt.second)
                    {
                        if(!prevIsSample)
                        {
                            SamplePoint sp;
                            sp.pos = prevInt.first;
                            /* DebugLogger::ss << "SamplePoint: " << sp.pos; */
                            /* DebugLogger::log(); */
                            if(otherSamples.size() < num && prevInt.second == other)
                                otherSamples.push_back(sp);
                            else if(theseSamples.size() < num)
                                theseSamples.push_back(sp);
                        }
                        if(theseSamples.size() >= num && otherSamples.size() >= num) break;
                        SamplePoint sp;
                        sp.pos = currInt.first;
                        /* DebugLogger::ss << "SamplePoint: " << sp.pos; */
                        /* DebugLogger::log(); */
                        if(otherSamples.size() < num && currInt.second == other)
                            otherSamples.push_back(sp);
                        else if(theseSamples.size() < num) 
                            theseSamples.push_back(sp);
                        if(theseSamples.size() >= num && otherSamples.size() >= num) break;
                        prevIsSample = true;
                    } else
                        prevIsSample = false;

                    prevInt = currInt;
                }
                /* DebugLogger::ss << "Current sampleAmount: " << samples.size(); */
                /* DebugLogger::log(); */
            }
            else
            {
                /* DebugLogger::ss << "No actual intersections!"; */
                /* DebugLogger::log(); */
                /* exit(0); */
            }
        }
        else 
        {
            /* DebugLogger::ss << "No intersection with the bounding boxes"; */
            /* DebugLogger::log(); */
        }
    }
    appendNonUniformSamples(theseSamples,other);
    other->appendNonUniformSamples(otherSamples,shared_from_this());

    // OUTPUT RAY SOURCE AND TARGET POINTS
    /* utilities::checkPath(Configuration::getInstance().get("ExperimentTmpPath")+"/raypoints/"); */
    /* write::writePointsToFile(sources, Configuration::getInstance().get("ExperimentTmpPath")+"/raypoints/sourcePts.pts"); */
    /* write::writePointsToFile(targets, Configuration::getInstance().get("ExperimentTmpPath")+"/raypoints/targetPts.pts"); */

}
