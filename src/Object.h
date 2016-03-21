/* 
 * Object.h
 *
 * Author: Wouter Franken
 *
 * CONTENT: 
 *   - EigenBbox to CgalBbox utility function
 *   - Object class header:
 *      + CON-/DESTRUCTORS
 *      + SAMPLING
 *      + GETTERS
 */

#pragma once

#include <assimp/mesh.h>
#include <assimp/matrix4x4.h>
#include <string>
#include <sstream>

#include "AnalysisPhase/Sampler.h"
#include "Mesh.h"
#include "types.h"
#include "Utilities/Transformation.h"

//typedef struct EigenMesh {
    //Eigen::MatrixXd vertices = Eigen::MatrixXd(0,3);
    //Eigen::MatrixXi faces = Eigen::MatrixXi(0,3);
//} EigenMesh;

class Object
{
    friend class Mesh;
private:
    // EigenBbox to CgalBbox utility function
    Bbox3d eigenBboxToCgal (const Eigen::Matrix<double, 4, 2>& bbox) const {
        Bbox3d cgalBbox(bbox(0,0), bbox(1,0), bbox(2,0), bbox(0,1), bbox(1,1), bbox(2,1));
        return cgalBbox;
    }
public:
    // CON-/DESTRUCTORS
    Object(const aiMesh *t_mesh , std::stack<Transformation> transformationStack);
    Object(const std::string &t_name, Mesh t_mesh , std::stack<Transformation> transformationStack = Transformation::emptyTransformationStack());
    //Object(const std::string &t_name, Mesh t_mesh, const Eigen::Matrix4d t_transformation);
    virtual ~Object();


    // SAMPLING
    virtual void sampleUniform(int num);
    virtual void sampleNonUniform(int num, std::shared_ptr<Object> other, bool convexHullApprox = false);

    // INTERSECTION TEST
    virtual bool intersects(std::shared_ptr<Object> other);

    // GETTERS
    virtual std::string getName() const { return m_name;}
    virtual Mesh getMesh() const {return m_mesh;}
    virtual Eigen::Matrix4d getCurrentTransformation() const {return Transformation::getTransformationMatrix(m_transformationStack);}
    virtual Bbox3d getBbox() const { return eigenBboxToCgal(m_bbox);}
    virtual Bbox3d bbox() const {return getBbox();}
    virtual double getUniformSampleDensity() const {return m_uniformSampleDensity;}
    virtual double getNonUniformSampleDensity(std::shared_ptr<Object> objPtr) const {return m_nonUniformSampleDensity.at(objPtr);}
    virtual std::vector<SamplePoint> getUniformSamples() const {return m_uniformSamples;}
    virtual std::vector<SamplePoint> getNonUniformSamples(std::shared_ptr<Object> objPtr) const {return m_nonUniformSamples.at(objPtr);}
    virtual std::vector<SamplePoint> getActiveSamples(std::shared_ptr<Object> objPtr) const 
    {
      
      if(m_nonUniformSamples.find(objPtr) != m_nonUniformSamples.end() && m_nonUniformSamples.at(objPtr).size() > 0) 
      {
        std::vector<SamplePoint> result;
        result.reserve(m_nonUniformSamples.at(objPtr).size() + m_uniformSamples.size());
        result.insert(result.end(),m_nonUniformSamples.at(objPtr).begin(),m_nonUniformSamples.at(objPtr).end());
        result.insert(result.end(),m_uniformSamples.begin(),m_uniformSamples.end());
        return result; 
      } 
      else 
      {
        return m_uniformSamples;
      }
    }
    virtual double getActiveSampleDensity(std::shared_ptr<Object> objPtr) const 
    {
        if(m_nonUniformSamples.at(objPtr).size() > 0) 
            return m_nonUniformSampleDensity.at(objPtr); 
        else 
            return m_uniformSampleDensity;
    }
    virtual Eigen::Vector4d getCentroid() const {return m_centroid;}

    // TRANSFORMATION
    virtual void translate(double x, double y, double z);
    virtual void setPosition(double x, double y, double z);
    virtual void clearTransformationStack();
private:
    // INIT
    virtual Mesh init(const aiMesh *t_mesh, std::stack<Transformation> transformationStack, const std::string &t_name);
    virtual void initBbox(const std::vector<Point3d> pointVec = std::vector<Point3d>(0));
    virtual void initCentroid(const std::vector<Point3d> pointVec = std::vector<Point3d>(0));
    //virtual void initEigenMesh();

    // TRANSFORMATION
    virtual void undoLatestTransformation(std::stack<Transformation> &theStack);
    virtual void undoTransformationStack();
    virtual void applyCurrentTransformationStack();
    virtual void applyTransformation(Eigen::Matrix4d transformation);
    virtual std::vector<Point3d> getObjectPoints();

    // BASIC OBJECT PROPERTIES
    Eigen::Matrix<double, 4,2> m_bbox;
    std::string m_name;
    Mesh m_mesh;
    //EigenMesh m_eigenMesh; // Same as the mesh above, but using Eigen matrices, necessary for igl intersection tests
    std::stack<Transformation> m_transformationStack;
    Eigen::Vector4d m_centroid;

    // SAMPLING
    double m_uniformSampleDensity;
    std::map<std::shared_ptr<Object>,double> m_nonUniformSampleDensity;
    std::vector<SamplePoint> m_uniformSamples;
    std::map<std::shared_ptr<Object>,std::vector<SamplePoint>> m_nonUniformSamples;
    //std::map<std::shared_ptr<Object>,std::vector<SamplePoint>> nonUniformSamples;
};