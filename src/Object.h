#pragma once

#include <assimp/mesh.h>
#include <assimp/matrix4x4.h>
#include <string>
#include "Mesh.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>


typedef CGAL::Simple_cartesian<double> Kd;
typedef Kd::Point_3 Point3d;
typedef CGAL::Surface_mesh<Point3d> Mesh3d;
typedef Kd::Vector_3 Vector3d;
typedef CGAL::Bbox_3 Bbox3d;
typedef CGAL::Iso_cuboid_3<Kd> IsoCub3d;

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
    virtual Vector3d getVertexNormal(const Mesh3d::Vertex_index &v_id) const;
    virtual Vector3d getFaceNormal(const Mesh3d::Face_index &f_id) const;
    virtual std::vector<Mesh3d::Vertex_index> getVerticesOfFace(const Mesh3d::Face_index &face) const;

private:
    virtual Mesh3d init(const aiMesh *t_mesh);
    template<class PropT>
    PropT getProperty(const Mesh3d::Vertex_index &v_id, const std::string &propName) const;
    template<class PropT>
    PropT getProperty(const Mesh3d::Face_index &f_id, const std::string &propName) const;
    virtual Vector3d calculateFaceNormal(const Mesh3d::Face_index &face) const;

    const std::string m_name;
    const Mesh3d m_mesh;
    const aiMatrix4x4 m_transformation;
    Bbox3d m_bbox;
};