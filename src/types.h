#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>

typedef struct CellInfo {
    bool seen = false;
    int id = -1;
} CellInfo;

typedef struct VertexInfo {
    bool seen = false;
    unsigned int visitid = 0;
} VertexInfo;
typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Triangulation_cell_base_with_info_3<CellInfo, K> Cb;
typedef CGAL::Triangulation_vertex_base_with_info_3<VertexInfo, K> Vb;
typedef CGAL::Triangulation_data_structure_3<Vb,Cb> Tds;
typedef CGAL::Delaunay_triangulation_3<K,Tds> Triangulation;
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