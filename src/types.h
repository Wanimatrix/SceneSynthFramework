#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_3<K> Triangulation;
typedef CGAL::Simple_cartesian<double> Kd;
typedef CGAL::Cartesian_converter<K,Kd>  K_to_Kd;
typedef CGAL::Cartesian_converter<Kd,K> Kd_to_K;
typedef Kd::Point_3 Point3d;
typedef CGAL::Surface_mesh<Point3d> Mesh3d;
typedef Kd::Vector_3 Vector3d;
typedef CGAL::Bbox_3 Bbox3d;
typedef CGAL::Iso_cuboid_3<Kd> IsoCub3d;
typedef Mesh3d::Face_index Face;
typedef Mesh3d::Vertex_index Vertex;
typedef Mesh3d::Edge_index Edge;
template <typename T>
using FaceProperty = Mesh3d::Property_map<Face,T>;
template <typename T>
using VertexProperty = Mesh3d::Property_map<Vertex,T>;
template <typename T>
using EdgeProperty = Mesh3d::Property_map<Edge,T>;