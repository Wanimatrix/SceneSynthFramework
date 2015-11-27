#include "writeOBJ.h"

void writeOBJ::write(const Object &obj, const std::string &filepath)
{
    std::cout << "Writing "+obj.getName()+" to OBJ ..." << std::endl;

    std::ofstream file(filepath, std::ofstream::trunc);
    file << "#Exported OBJ file of object " << obj.getName() << "." << std::endl;

    int Vnum = 0;
    VertexProperty<Point3d> pts = obj.getMesh().mesh3d->points();
    BOOST_FOREACH(Vertex v_id, obj.getMesh().mesh3d->vertices())
    {
        const Point3d p = pts[v_id];
        file << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl;
        Vnum++;
    }
    file << "# " << Vnum << " vertices" << std::endl << std::endl;

    /*Vnum = 0;
    VertexProperty<Vector3d> vnormals = obj.getMesh().getVertexProperty<Vector3d>("normal");
    BOOST_FOREACH(Vertex v_id, obj.getMesh().mesh3d->vertices())
    {
        const Vector3d vn = vnormals[v_id];
        file << "vn " << vn.x() << " " << vn.y() << " " << vn.z() << std::endl;
        Vnum++;
    }
    file << "# " << Vnum << " vertex normals" << std::endl << std::endl;*/

    Vnum = 0;
    BOOST_FOREACH(Face f_id, obj.getMesh().mesh3d->faces())
    {
        int i = 0;
        file << "f ";
        for(Vertex v:obj.getMesh().getVerticesOfFace(f_id)){
            file << static_cast<int>(v)+1 << "//" << static_cast<int>(v)+1 << " ";
        }

        file << std::endl;
        Vnum++;
    }
    file << "# " << Vnum << " faces" << std::endl;
    file.close();
}