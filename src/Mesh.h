#pragma once

#include "types.h"
#include <memory>

class Mesh
{
public:
    Mesh(std::shared_ptr<Mesh3d> t_mesh);
    virtual ~Mesh();

    virtual Vector3d getVertexNormal(const Vertex &v_id) const;
    virtual Vector3d getFaceNormal(const Face &f_id) const;
    virtual std::vector<Vertex> getVerticesOfFace(const Face &face) const;
    virtual double getSurfaceArea() const {return surfaceArea;}

    template <typename PropT>
    VertexProperty<PropT> getVertexProperty(const std::string &propName) const {
        VertexProperty<PropT> propMap;
        bool exists;
        boost::tie(propMap,exists) = mesh3d->property_map<Vertex,PropT>("v:"+propName);
        return propMap;
    }

    template <typename PropT>
    FaceProperty<PropT> getFaceProperty(const std::string &propName) const {
        FaceProperty<PropT> propMap;
        bool exists;
        boost::tie(propMap,exists) = mesh3d->property_map<Face,PropT>("f:"+propName);
        return propMap;
    }

    virtual Mesh cloneMesh() const
    {
        //Mesh3d mesh3dCopy = *mesh3d;
        std::shared_ptr<Mesh3d> mesh3dCopyPtr(new Mesh3d(*mesh3d));
        Mesh result(mesh3dCopyPtr);
        assert(mesh3d!=result.mesh3d);
        return result;
    }
    virtual Eigen::MatrixXd getVertices();
    virtual Eigen::MatrixXi getFaces();

    //Mesh& operator= (const Mesh &t_mesh);


private:
    virtual Vector3d calculateFaceNormal(const Face &face) const;
    virtual Vector3d calculateVertexNormal(const Vertex &vertex) const;
    virtual double calculateFaceArea(const Face &face, const Vector3d &faceNormal) const;

public:
    std::shared_ptr<Mesh3d> mesh3d;

private:
    double surfaceArea;
    //Point3d centroid;
};