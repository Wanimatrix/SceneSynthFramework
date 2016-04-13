
#include <iostream>
#include <string>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Simple_cartesian<double> Kd;
typedef Kd::Point_3 Point3d;
typedef CGAL::Surface_mesh<Point3d> Mesh3d;
typedef Kd::Vector_3 Vector3d;
typedef Mesh3d::Face_index Face;
typedef Mesh3d::Vertex_index Vertex;
typedef Mesh3d::Edge_index Edge;

int main()
{
    Mesh3d m;
    Vertex u = m.add_vertex(Point3d(0,1,0));
    Vertex v = m.add_vertex(Point3d(0,0,0));
    Vertex w = m.add_vertex(Point3d(1,0,0));
    Vertex z = m.add_vertex(Point3d(1,1,0));
    m.add_face(u, v, w);
    m.add_face(u, w, z);

    for (Vertex vd : m.vertices())
    {
        CGAL::Halfedge_around_source_circulator<Mesh3d> vhit(m.halfedge(vd),m), vhend(vhit);
        do
        {
            Edge e = m.edge(*vhit);
            Vertex other = (m.vertex(e,0) == vd) ? m.vertex(e,1) : m.vertex(e,0);
            std::cout << "There is an edge between " << vd << " and " << other << std::endl;
        } while (++vhit != vhend);
    }
}
