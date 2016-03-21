#include "Mesh.h"

Mesh::Mesh(std::shared_ptr<Mesh3d> t_mesh) : mesh3d(t_mesh) {
    bool vNormalsExist;
    VertexProperty<Vector3d> vNormals;
    bool fNormalsExist;
    FaceProperty<Vector3d> fNormals;
    boost::tie(vNormals, vNormalsExist) = mesh3d->property_map<Vertex,Vector3d>("v:normal");
    boost::tie(fNormals, fNormalsExist) = mesh3d->property_map<Face,Vector3d>("f:normal");

    if(!vNormalsExist) {
        vNormals = mesh3d->add_property_map<Vertex,Vector3d>("v:normal").first;
        BOOST_FOREACH(Vertex v_id, mesh3d->vertices())
            vNormals[v_id] = calculateVertexNormal(v_id);
    }
    if(!fNormalsExist) fNormals = mesh3d->add_property_map<Face,Vector3d>("f:normal").first;

    FaceProperty<double> fAreas;
    surfaceArea = 0;
    fAreas = mesh3d->add_property_map<Face,double>("f:area").first;
    BOOST_FOREACH(Face f_id, mesh3d->faces()) {
        if(!fNormalsExist) fNormals[f_id] = calculateFaceNormal(f_id);
        fAreas[f_id] = calculateFaceArea(f_id, fNormals[f_id]);
        surfaceArea += fAreas[f_id];
    }

    std::vector<Point3d> pts;
    BOOST_FOREACH(Vertex v_id, mesh3d->vertices())
        pts.push_back(mesh3d->point(v_id));

}

Mesh::~Mesh() {
}

Vector3d Mesh::getVertexNormal(const Vertex &v_id) const {
    return getVertexProperty<Vector3d>("normal")[v_id];
}

Vector3d Mesh::getFaceNormal(const Face &f_id) const {
    return getFaceProperty<Vector3d>("normal")[f_id];
}

std::vector<Vertex> Mesh::getVerticesOfFace(const Face &face) const {
    std::vector<Vertex> result;
    CGAL::Vertex_around_face_iterator<Mesh3d> vbegin, vend;
    for(boost::tie(vbegin, vend) = CGAL::vertices_around_face(mesh3d->halfedge(face), *mesh3d);
        vbegin != vend; 
        ++vbegin){
        result.push_back(*vbegin);
    }
    return result;
}

Vector3d Mesh::calculateFaceNormal(const Face &face) const {
    VertexProperty<Point3d> points = mesh3d->points();
    std::vector<Vertex> faceVerts = getVerticesOfFace(face);

    Vector3d vec0 = points[faceVerts[1]] - points[faceVerts[0]];
    Vector3d vec1 = points[faceVerts[2]] - points[faceVerts[0]];

    Vector3d faceNormal = CGAL::cross_product(vec0,vec1);
    faceNormal = faceNormal / std::sqrt(faceNormal.squared_length());

    if (mesh3d->property_map<Vertex,Vector3d>("v:normal").first) {
        Vector3d vertexNormal = getVertexNormal(faceVerts[0]);
        double dot = faceNormal * vertexNormal;

        if(dot < 0.0)
            return (-1*faceNormal);
    } 
    return faceNormal;
}

Vector3d Mesh::calculateVertexNormal(const Vertex &vertex) const {
    Vector3d vertexNormal;

    CGAL::Halfedge_around_target_circulator<Mesh3d> hbegin(mesh3d->halfedge(vertex), *mesh3d), done(hbegin);
    int i = 0;
    do {
        if(!mesh3d->is_border(*hbegin)) {
            Face f = mesh3d->face(*hbegin++);
            vertexNormal = vertexNormal + calculateFaceNormal(f);
            i++;
        }
    } while (hbegin != done);
    vertexNormal = vertexNormal / i;
    return vertexNormal / std::sqrt(vertexNormal.squared_length());
}

double Mesh::calculateFaceArea(const Face &face, const Vector3d &faceNormal) const {
    VertexProperty<Point3d> points = mesh3d->points();
    std::vector<Vertex> vertices = getVerticesOfFace(face);
    Vector3d normal(faceNormal);
    if(vertices.size() < 3) return 0;
    if(vertices.size() == 3) return std::sqrt(CGAL::squared_area(points[vertices[0]],points[vertices[1]],points[vertices[2]]));

    // For testing purposes
    // vertices = std::vector<Vertex>({Vertex(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,1)),Vertex(Eigen::Vector3d(0,1,0),Eigen::Vector3d(0,0,1)),Vertex(Eigen::Vector3d(1,2,0),Eigen::Vector3d(0,0,1)),Vertex(Eigen::Vector3d(1,1,0),Eigen::Vector3d(0,0,1))});
    // normal = Eigen::Vector3d(0,0,1);

    // SUPPORT FOR QUADS, ETC.
    Vector3d total(0,0,0);
    for (int i = 0; i < vertices.size(); i++)
    {
        int a = i;
        int b = (i+1) % vertices.size();

        Point3d pa = points[vertices[a]];
        Point3d pb = points[vertices[b]];

        Vector3d va = Vector3d(pa.x(),pa.y(),pa.z());
        Vector3d vb = Vector3d(pb.x(),pb.y(),pb.z());

        total = total + CGAL::cross_product(va,vb);
    }

    normal = normal / std::sqrt(normal.squared_length());
    return std::abs(normal * total)/2.0;
}


Eigen::MatrixXd Mesh::getVertices()
{
    // Init eigenmesh matrices
    Eigen::MatrixXd vertices(mesh3d->vertices().size(),3);

    // Store vertices in eigenMesh
    for(Vertex v : mesh3d->vertices())
    {
        Point3d point = mesh3d->point(v);
        vertices(v,0) = point.x();
        vertices(v,1) = point.y();
        vertices(v,2) = point.z();
    }

    return vertices;
}

Eigen::MatrixXi Mesh::getFaces()
{
    Eigen::MatrixXi faces(mesh3d->faces().size(),3);

    // Store faces in eigenMesh
    for(Face f : mesh3d->faces())
    {
        std::vector<Vertex> faceVertices = getVerticesOfFace(f);
        faces(f,0) = faceVertices[0];
        faces(f,1) = faceVertices[1];
        faces(f,2) = faceVertices[2];
    }

    return faces;
}

/*Mesh& Mesh::operator=(const Mesh &t_mesh) {
    mesh3d = Mesh3d(t_mesh.mesh3d);
    surfaceArea = t_mesh.getSurfaceArea();

    return *this;
}*/