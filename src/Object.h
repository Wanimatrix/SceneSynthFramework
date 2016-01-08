#pragma once

#include <assimp/mesh.h>
#include <assimp/matrix4x4.h>
#include <string>
#include <sstream>

#include "AnalysisPhase/Sampler.h"
#include "Mesh.h"
#include "types.h"

class Object
{
public:
    Object(const aiMesh *t_mesh , const aiMatrix4x4 t_transformation);
    Object(const std::string &t_name, Mesh t_mesh , const aiMatrix4x4 t_transformation = aiMatrix4x4());
    virtual ~Object();

    virtual std::string getName(bool withUOI = true) const {
        std::ostringstream sstr;
        if(withUOI && uniqueObjIdx >= 0)
            sstr << uniqueObjIdx;

        return m_name+sstr.str();
    
    }
    virtual Mesh getMesh() const {return m_mesh;}
    virtual aiMatrix4x4 getTransformation() const {return m_transformation;}
    virtual Bbox3d getBbox() const {return m_bbox;}
    virtual Bbox3d bbox() const {return getBbox();}
    virtual double getSampleDensity() const {return sampleDensity;}
    virtual std::vector<SamplePoint> getSamples() const {return samples;}
    virtual Point3d getCentroid() const {return Point3d(m_bbox.min(0)+(m_bbox.max(0)-m_bbox.min(0))/2,m_bbox.min(1)+(m_bbox.max(1)-m_bbox.min(1))/2,m_bbox.min(2)+(m_bbox.max(2)-m_bbox.min(2))/2);}
    virtual bool compareObjOnCentroid(const std::shared_ptr<Object> j);
    virtual void setUniqueObjIndex(int uidx);

    virtual void sampling(int num);

private:
    virtual Mesh init(const aiMesh *t_mesh, const aiMatrix4x4 &t_transformation, const std::string &t_name);

    Bbox3d m_bbox;
    std::string m_name;
    aiMatrix4x4 m_transformation;
    Mesh m_mesh;
    double sampleDensity;
    std::vector<SamplePoint> samples;
    int uniqueObjIdx = -1;
};