#include "Vertex.h"

Vertex::Vertex(const aiVector3D &t_position, const aiVector3D &t_normal) 
    : m_position(t_position[0], t_position[1], t_position[2]),
      m_normal(t_normal[0], t_normal[1], t_normal[2])
{}

Vertex::Vertex(const Eigen::Vector3d &t_position, const Eigen::Vector3d &t_normal)
    : m_position(t_position), m_normal(t_normal) {}

Vertex::~Vertex() {}