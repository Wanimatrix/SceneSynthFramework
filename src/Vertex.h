#pragma once

#include <assimp/types.h>
#include <Eigen/Dense>

class Vertex
{
public:
    Vertex(const aiVector3D &t_position, const aiVector3D &t_normal);
    Vertex(const Eigen::Vector3d &t_position, const Eigen::Vector3d &t_normal);
    ~Vertex();

    virtual Eigen::Vector3d getPosition() const {return m_position;}
    virtual Eigen::Vector3d getNormal() const {return m_normal;}
private:
    const Eigen::Vector3d m_position;
    const Eigen::Vector3d m_normal;
};