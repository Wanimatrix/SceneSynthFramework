#include "Mesh.h"
#include <iostream>

Mesh::Mesh(const aiMesh *t_mesh) {

    for (int i = 0; i < t_mesh->mNumVertices; ++i)
    {
        m_vertices.push_back(Vertex(t_mesh->mVertices[i],t_mesh->mNormals[i]));
    }

    for (int f = 0; f < t_mesh->mNumFaces; ++f)
    {
        Eigen::VectorXd face(t_mesh->mFaces[f].mNumIndices);
        for (int i = 0; i < t_mesh->mFaces[f].mNumIndices; ++i)
        {
            face[i] = t_mesh->mFaces[f].mIndices[i];
        }
        m_faces.push_back(face);
        Eigen::Vector3d faceNormal = calculateFaceNormal(face);
        m_faceNormals.push_back(faceNormal);
        m_faceAreas.push_back(calculateFaceArea(face,faceNormal));
    }
}

Mesh::~Mesh() {}

Eigen::Vector3d Mesh::calculateFaceNormal(const Eigen::VectorXd &face) const {
    Eigen::Vector3d vec0 = m_vertices[face[1]].getPosition() - m_vertices[face[0]].getPosition();
    Eigen::Vector3d vec1 = m_vertices[face[2]].getPosition() - m_vertices[face[0]].getPosition();

    Eigen::Vector3d faceNormal = vec0.cross(vec1);
    Eigen::Vector3d vertexNormal = m_vertices[face[0]].getNormal();
    float dot = faceNormal.dot(vertexNormal);

    if(dot < 0.0f)
        return (-1*faceNormal);
    return (faceNormal);
}

double Mesh::calculateFaceArea(const Eigen::VectorXd &face, const Eigen::Vector3d &faceNormal) const {
    std::vector<Vertex> vertices = getVerticesForFace(face);
    Eigen::Vector3d normal(faceNormal);
    if(vertices.size() < 3) return 0;

    // For testing purposes
    // vertices = std::vector<Vertex>({Vertex(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,1)),Vertex(Eigen::Vector3d(0,1,0),Eigen::Vector3d(0,0,1)),Vertex(Eigen::Vector3d(1,2,0),Eigen::Vector3d(0,0,1)),Vertex(Eigen::Vector3d(1,1,0),Eigen::Vector3d(0,0,1))});
    // normal = Eigen::Vector3d(0,0,1);

    Eigen::Vector3d total(0,0,0);
    for (int i = 0; i < vertices.size(); i++)
    {
        int a = i;
        int b = (i+1) % vertices.size();

        Eigen::Vector3d va = vertices[a].getPosition();
        Eigen::Vector3d vb = vertices[b].getPosition();

        total += va.cross(vb);
    }

    normal.normalize();
    return std::abs(normal.dot(total)/2.0);
}

std::vector<Vertex> Mesh::getVerticesForFace(const Eigen::VectorXd &face) const {
    std::vector<Vertex> vertices;

    for(int i = 0; i < face.rows(); i++) {
        vertices.push_back(m_vertices[face(i)]);
    }

    return vertices;
}