#pragma once

#include <assimp/mesh.h>
#include <assimp/matrix4x4.h>
#include <string>

#include "AnalysisPhase/Sampler.h"
#include "Mesh.h"
#include "types.h"

class Object
{
public:
    Object(const aiMesh *t_mesh , const aiMatrix4x4 t_transformation);
    Object(const std::string &t_name, Mesh t_mesh , const aiMatrix4x4 t_transformation = aiMatrix4x4());
    virtual ~Object();

    virtual std::string getName() const {return m_name;}
    virtual Mesh getMesh() const {return m_mesh;}
    virtual aiMatrix4x4 getTransformation() const {return m_transformation;}
    virtual Bbox3d getBbox() const {return m_bbox;}
    virtual Bbox3d bbox() const {return getBbox();}
    virtual double getSampleDensity() const {return sampleDensity;}
    virtual std::vector<SamplePoint> getSamples() const {return samples;}

    virtual void sampling(int num);

private:
    virtual Mesh init(const aiMesh *t_mesh);

    std::string m_name;
    Mesh m_mesh;
    aiMatrix4x4 m_transformation;
    Bbox3d m_bbox;
    double sampleDensity;
    std::vector<SamplePoint> samples;
};