#include "Object.h"

#include <assert.h>
#include <iostream>
#include <CGAL/bounding_box.h>

Object::Object(const aiMesh *t_mesh , const aiMatrix4x4 t_transformation) : m_name(t_mesh->mName.C_Str()), m_mesh(init(t_mesh)), m_transformation(t_transformation)
{
}

Object::~Object() 
{

}

Mesh3d Object::init(const aiMesh *t_mesh) {
    Mesh3d mesh;
    bool created;
    Mesh3d::Property_map<Mesh3d::Vertex_index,Vector3d> vertexNormals;
    boost::tie(vertexNormals, created) = mesh.add_property_map<Mesh3d::Vertex_index,Vector3d>("v:normal");
    Mesh3d::Property_map<Mesh3d::Face_index,Vector3d> faceNormals;
    boost::tie(faceNormals, created) = mesh.add_property_map<Mesh3d::Face_index,Vector3d>("f:normal");

    std::vector<Mesh3d::Vertex_index> vertexIdxes;
    std::vector<Point3d> points;
    for (int i = 0; i < t_mesh->mNumVertices; ++i)
    {
        const aiVector3D point = t_mesh->mVertices[i];
        const aiVector3D meshNormal = t_mesh->mNormals[i];
        auto p = Point3d(point[0],point[1],point[2]);
        points.push_back(p);
        Mesh3d::Vertex_index v_id = mesh.add_vertex(p);
        vertexIdxes.push_back(v_id);
        vertexNormals[v_id] = Vector3d(meshNormal[0],meshNormal[1],meshNormal[2]);
    }

    m_bbox = CGAL::bounding_box(std::begin(points),std::end(points)).bbox();

    assert(vertexIdxes.size()==t_mesh->mNumVertices);

    for (int f = 0; f < t_mesh->mNumFaces; ++f)
    {
        std::vector<Mesh3d::Vertex_index> faceVertIdxes;
        Mesh3d::Face_index f_id;
        for (int i = 0; i < t_mesh->mFaces[f].mNumIndices; ++i)
        {
            faceVertIdxes.push_back(vertexIdxes[t_mesh->mFaces[f].mIndices[i]]);
        }
        if(t_mesh->mFaces[f].mNumIndices == 3)
            f_id = mesh.add_face(faceVertIdxes[0],faceVertIdxes[1],faceVertIdxes[2]);
        else
            f_id = mesh.add_face(faceVertIdxes[0],faceVertIdxes[1],faceVertIdxes[2],faceVertIdxes[3]);
        faceNormals[f_id] = calculateFaceNormal(f_id);
    }

    return mesh;
}

Vector3d Object::getVertexNormal(const Mesh3d::Vertex_index &v_id) const {
    return getProperty<Vector3d>(v_id,"normal");
}

Vector3d Object::getFaceNormal(const Mesh3d::Face_index &f_id) const {
    return getProperty<Vector3d>(f_id,"f:normal");
}

template<class PropT>
PropT Object::getProperty(const Mesh3d::Vertex_index &v_id, const std::string &propName) const {
    Mesh3d::Property_map<Mesh3d::Vertex_index,PropT> propMap;
    bool exists;
    boost:tie(propMap,exists) = m_mesh.property_map<Mesh3d::Vertex_index,PropT>("v:"+propName);
    return propMap[v_id];
}

template<class PropT>
PropT Object::getProperty(const Mesh3d::Face_index &f_id, const std::string &propName) const {
    Mesh3d::Property_map<Mesh3d::Face_index,PropT> propMap;
    bool exists;
    boost:tie(propMap,exists) = m_mesh.property_map<Mesh3d::Face_index,PropT>(propName);
    return propMap[f_id];
}

std::vector<Mesh3d::Vertex_index> Object::getVerticesOfFace(const Mesh3d::Face_index &face) const {
    std::vector<Mesh3d::Vertex_index> result;
    CGAL::Vertex_around_face_iterator<Mesh3d> vbegin, vend;
    for(boost::tie(vbegin, vend) = CGAL::vertices_around_face(m_mesh.halfedge(face), m_mesh);
        vbegin != vend; 
        ++vbegin){
        result.push_back(*vbegin);
    }
    return result;
}

Vector3d Object::calculateFaceNormal(const Mesh3d::Face_index &face) const {
    Mesh3d::Property_map<Mesh3d::Vertex_index,Point3d> points = m_mesh.points();
    std::vector<Mesh3d::Vertex_index> faceVerts = getVerticesOfFace(face);

    Vector3d vec0 = points[faceVerts[1]] - points[faceVerts[0]];
    Vector3d vec1 = points[faceVerts[2]] - points[faceVerts[0]];

    Vector3d faceNormal = CGAL::cross_product(vec0,vec1);
    Vector3d vertexNormal = getVertexNormal(faceVerts[0]);
    double dot = faceNormal * vertexNormal;

    if(dot < 0.0)
        return (-1*faceNormal);
    return (faceNormal);
}