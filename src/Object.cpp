#include "Object.h"

#include <assert.h>
#include <iostream>
#include <CGAL/bounding_box.h>
#include "AnalysisPhase/Sampler.h"
#include <memory>

Object::Object(const aiMesh *t_mesh , const aiMatrix4x4 t_transformation) : m_name(t_mesh->mName.C_Str()), m_mesh(init(t_mesh)), m_transformation(t_transformation)
{
}

Object::Object(const std::string &t_name, Mesh t_mesh, const aiMatrix4x4 t_transformation) : m_name(t_name), m_mesh(t_mesh), m_transformation(t_transformation)
{
}

Object::~Object() 
{

}

// Converts aiMesh to Mesh
Mesh Object::init(const aiMesh *t_mesh) {
    std::shared_ptr<Mesh3d> mesh3d(new Mesh3d());
    bool created;
    VertexProperty<Vector3d> vertexNormals;
    boost::tie(vertexNormals, created) = mesh3d->add_property_map<Vertex,Vector3d>("v:normal");

    std::vector<Vertex> vertexIdxes;
    std::vector<Point3d> points;
    for (int i = 0; i < t_mesh->mNumVertices; ++i)
    {
        const aiVector3D point = t_mesh->mVertices[i];
        const aiVector3D meshNormal = t_mesh->mNormals[i];
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