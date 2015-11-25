#pragma once

#include <assimp/mesh.h>
#include <assimp/matrix4x4.h>
#include <string>

#include "AnalysisPhase/Sampler.h"
#include "types.h"

class Object
{
public:
    Object(const aiMesh *t_mesh , const aiMatrix4x4 t_transformation);
    virtual ~Object();

    virtual std::string getName() const {return m_name;}
    virtual Mesh3d getMesh() const {return m_mesh;}
    virtual aiMatrix4x4 getTransformation() const {return m_transformation;}
    virtual Bbox3d getBbox() const {return m_bbox;}
    virtual Bbox3d bbox() const {return getBbox();}
    virtual Vector3d getVertexNormal(const Vertex &v_id) const;
    virtual Vector3d getFaceNormal(const Face &f_id) const;
    virtual std::vector<Vertex> getVerticesOfFace(const Face &face) const;
    virtual double getSurfaceArea() const {return surfaceArea;}
    virtual double getSampleDensity() const {return sampleDensity;}
    virtual std::vector<SamplePoint> getSamples() const {return samples;}

    virtual void sampling(int num);

private:
    virtual Mesh3d init(const aiMesh *t_mesh);
    template <typename PropT>
    PropT getProperty(const Vertex &v_id, const std::string &propName) const;
    template <typename PropT>
    PropT getProperty(const Face &f_id, const std::string &propName) const;
    virtual Vector3d calculateFaceNormal(const Face &face) const;
    virtual double calculateFaceArea(const Face &face, const Vector3d &faceNormal) const;

    std::string m_name;
    Mesh3d m_mesh;
    aiMatrix4x4 m_transformation;
    Bbox3d m_bbox;
    double surfaceArea;
    double sampleDensity;
    std::vector<SamplePoint> samples;
};