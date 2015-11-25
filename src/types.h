#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Simple_cartesian<double> Kd;
typedef Kd::Point_3 Point3d;
typedef CGAL::Surface_mesh<Point3d> Mesh3d;
typedef Kd::Vector_3 Vector3d;
typedef CGAL::Bbox_3 Bbox3d;
typedef CGAL::Iso_cuboid_3<Kd> IsoCub3d;
typedef Mesh3d::Face_index Face;
typedef Mesh3d::Vertex_index Vertex;
template <typename T>
using FaceProperty = Mesh3d::Property_map<Face,T>;
template <typename T>
using VertexProperty = Mesh3d::Property_map<Vertex,T>;