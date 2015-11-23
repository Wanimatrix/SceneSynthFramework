#pragma once

#include <assimp/mesh.h>
#include <vector>
#include <Eigen/Dense>
#include "Vertex.h"
#include "Face.h"

class Mesh
{
public:
    Mesh(const aiMesh *t_mesh);
    virtual ~Mesh();

    virtual std::vector<Eigen::VectorXd> getFaces() const {return m_faces;}
    virtual std::vector<Vertex> getVertices() const {return m_vertices;}
    virtual std::vector<Eigen::Vector3d> getFaceNormals() const {return m_faceNormals;}
    virtual std::vector<double> getFaceAreas() const  {return m_faceAreas;}

private:
    virtual std::vector<Vertex> getVerticesForFace(const Eigen::VectorXd &face) const;
    virtual double calculateFaceArea(const Eigen::VectorXd &face, const Eigen::Vector3d &faceNormal) const;
    virtual Eigen::Vector3d calculateFaceNormal(const Eigen::VectorXd &face) const;

    std::vector<Eigen::VectorXd> m_faces;
    std::vector<Vertex> m_vertices;
    std::vector<Eigen::Vector3d> m_faceNormals;
    std::vector<double> m_faceAreas;
};