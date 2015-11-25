/**
This code comes from this paper:

Hu, Ruizhen, et al. "Interaction Context (ICON): Towards a Geometric Functionality Descriptor." ACM Transactions on Graphics 34.
*/

#pragma once

#include "types.h"
#include <iostream>
#include <fstream>

class writeOBJ
{
public:
	static void write(const Mesh3d &mesh, const std::string &filename)
	{
		std::ofstream file(filename, std::ofstream::trunc);
		file << "#Exported OBJ file." << endl;
		file << "#Written by Chenyang Zhu." << endl;

		int Vnum = 0;
		VertexProperty<Point3d> points = mesh.points();

		//for (SurfaceMeshModel::Vertex_iterator vit=mesh->vertices_begin(); vit!=mesh->vertices_end(); ++vit)
		BOOST_FOREACH(Vertex v_id, mesh.vertices())
		{
			const Point3d p = points[v_id];
			file << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl;
			Vnum++;
		}
		file << "# " << Vnum << " vertices" << std::endl << std::endl;

		/*Vnum = 0;
		bool exists;
		VertexProperty<Vector3d> vnormals;
		boost::tie(vnormals, exists) = mesh.property_map<Vertex,Vector3d>("v:normal");
		//SurfaceMeshModel::Vertex_property<SurfaceMeshModel::Point> npoints = mesh->get_vertex_property<Point>("v:normal");
		//for (SurfaceMeshModel::Vertex_iterator vit=mesh->vertices_begin(); vit!=mesh->vertices_end(); ++vit)
		BOOST_FOREACH(Vertex v_id, mesh.vertices())
		{
			const Vector3d v = vnormals[v_id];
			file << "vn " << v.x() << " " << v.y() << " " << v.z() << std::endl;
			Vnum++;
		}
		file << "# " << Vnum << " vertex normals" << std::endl << std::endl;*/

		Vnum = 0;
		//for (SurfaceMeshModel::Face_iterator fit=mesh->faces_begin(); fit!=mesh->faces_end(); ++fit)
		BOOST_FOREACH(Face f_id, mesh.faces())
		{
			//SurfaceMeshModel::Vertex_around_face_circulator fvit=mesh->vertices(fit), fvend=fvit;
			int i = 0;
			file << "f ";
		    CGAL::Vertex_around_face_iterator<Mesh3d> vbegin, vend;
		    for(boost::tie(vbegin, vend) = CGAL::vertices_around_face(mesh.halfedge(f_id), mesh);
		        vbegin != vend; 
		        ++vbegin){
		    	Vertex v = *vbegin;
		    	file << static_cast<int>(v)+1 << "//" << static_cast<int>(v)+1 << " ";
		    }
			/*std::vector<Vertex> fvertices = mesh.getVerticesOfFace(f_id);
			
			for (Vertex v : fvertices)
			{
				
			}*/
			//while (++fvit != fvend);
			file << std::endl;
			Vnum++;
		}
		file << "# " << Vnum << " faces" << std::endl;
	}
};