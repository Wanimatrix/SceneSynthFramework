#include "Object.h"

#include <assert.h>
#include <iostream>
#include <CGAL/bounding_box.h>
#include "AnalysisPhase/Sampler.h"
#include <memory>
#include <boost/algorithm/string.hpp>
#include "Debug/DebugTools.h"

Object::Object(const aiMesh *t_mesh , const aiMatrix4x4 t_transformation) : m_mesh(init(t_mesh,t_transformation,std::string(t_mesh->mName.C_Str()))), m_transformation(t_transformation)
{
}

Object::Object(const std::string &t_name, Mesh t_mesh, const aiMatrix4x4 t_transformation) : m_name(t_name), m_mesh(t_mesh), m_transformation(t_transformation)
{
}

Object::~Object() 
{

}

void Object::setUniqueObjIndex(int uidx) {
    uniqueObjIdx = uidx;
}

bool Object::compareObjOnCentroid(const std::shared_ptr<Object> j) {
    Point3d centrI = getCentroid();
    Point3d centrJ = j->getCentroid();
    if(centrI.z() == centrJ.z()) {
        if(centrI.y() == centrJ.y())
            return centrI.x() < centrJ.x();
        else
            return centrI.y() < centrJ.y();
    } else 
        return centrI.z() < centrJ.z();
}

// Converts aiMesh to Mesh
Mesh Object::init(const aiMesh *t_mesh, const aiMatrix4x4 &t_transformation, const std::string &t_name) {
    std::vector<std::string> splitted;
    boost::split(splitted,t_name,boost::is_any_of("."));
    m_name = splitted[0];
    std::shared_ptr<Mesh3d> mesh3d(new Mesh3d());
    bool created;
    VertexProperty<Vector3d> vertexNormals;
    boost::tie(vertexNormals, created) = mesh3d->add_property_map<Vertex,Vector3d>("v:normal");
    //aiVector3D blenderTranslation;
    //aiVector3D sc;
    //aiQuaternion rot;
    //t_transformation.Decompose(sc,rot,blenderTranslation);

    std::vector<Vertex> vertexIdxes;
    std::vector<Point3d> points;
    for (int i = 0; i < t_mesh->mNumVertices; ++i)
    {
        aiVector3D point = aiMatrix4x4(t_transformation)*t_mesh->mVertices[i];
        aiVector3D meshNormal = aiMatrix3x3(t_transformation)*t_mesh->mNormals[i];
        Point3d p = Point3d(point[0],point[1],point[2]);
        points.push_back(p);
        Vertex v_id = mesh3d->add_vertex(p);
        vertexIdxes.push_back(v_id);
        vertexNormals[v_id] = Vector3d(meshNormal[0],meshNormal[1],meshNormal[2]);
    }

    m_bbox = CGAL::bounding_box(std::begin(points),std::end(points)).bbox();

    assert(vertexIdxes.size()==t_mesh->mNumVertices);

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
            f_id = mesh3d->add_face(faceVertIdxes[0],faceVertIdxes[1],faceVertIdxes[2],faceVertIdxes[3]);
    }

    return Mesh(mesh3d);
}

// Sampling points on this object's mesh
void Object::sampling(int num)
{
    Sampler s(m_mesh);
    samples = s.getSamples(num, 0);

    sampleDensity = static_cast<double>(num) / m_mesh.getSurfaceArea();
}