#include "Sampler.h"


Sampler::Sampler(const Mesh3d &srcMesh, SamplingMethod samplingMethod)
{
	/*if(srcMesh == NULL) 
		return;
	else*/
    mesh = srcMesh;
	method = samplingMethod;
	
	// mesh->update_face_normals(); // TODO
	bool exists;
    boost:tie(faceNormals,exists) = mesh.property_map<Mesh3d::Face_index,Vector3d>("f:normal");

	//SurfaceMeshHelper h(mesh);
    boost:tie(farea,exists) = mesh.property_map<Mesh3d::Face_index,float>("f:area");
    points = mesh.points();

	// FaceBarycenterHelper fh(mesh);
	boost:tie(fcenter,exists) = mesh.property_map<Mesh3d::Face_index,float>("f:center");

	// Sample based on method selected
	if( method == RANDOM_BARYCENTRIC_AREA || method == RANDOM_BARYCENTRIC_WEIGHTED)
	{
		bool created;
        Mesh3d::Property_map<Mesh3d::Face_index,Vector3d> fprobability;
        boost::tie(fprobability, created) = mesh.add_property_map<Mesh3d::Face_index,double>("f:probability");
		if (method == RANDOM_BARYCENTRIC_AREA)
		{
			// Compute all faces area
			// fprobability = mesh->face_property<Scalar>("f:probability", 0);

			double totalMeshArea = 0;
			Surface_mesh::Face_iterator fit, fend = mesh->faces_end();

            BOOST_FOREACH(Mesh3d::Face_index f_id, mesh.faces())
				totalMeshArea += farea[f_id];

			BOOST_FOREACH(Mesh3d::Face_index f_id, mesh.faces())
				fprobability[f_id] = farea[f_id] / totalMeshArea;
		}
		else
		{
			// fprobability = mesh->face_property<Scalar>("f:probability", 0);
            Mesh3d::Property_map<Mesh3d::Face_index,Vector3d> fweight;
            boost::tie(fweight, created) = mesh.property_map<Mesh3d::Face_index,double>("f:weight");
			// ScalarFaceProperty fweight = mesh->get_face_property<Scalar>("f:weight");
			double totalWeight = 0;
			// Surface_mesh::Face_iterator fit, fend = mesh->faces_end();
			BOOST_FOREACH(Mesh3d::Face_index f_id, mesh.faces())
				totalWeight += fweight[fit];

			BOOST_FOREACH(Mesh3d::Face_index f_id, mesh.faces())
				fprobability[fit] = fweight[fit] / totalWeight;
		}

        interval = std::vector<WeightFace>(mesh->num_of_faces() + 1);
        interval[0] = WeightFace(0.0, Mesh3d::Face_index(0));
		int i = 0;

		// Compute mesh area in a cumulative manner
        BOOST_FOREACH(Mesh3d::Face_index f_id, mesh.faces())
		{
            interval[f_id+1] = WeightFace(interval[i].weight + fprobability[f_id], f_id);
			i++;
		}
	}
	else if( method ==  FACE_CENTER_RANDOM || method == FACE_CENTER_ALL )
	{
		// No preparations needed..
	}
}

SamplePoint Sampler::getSample(double weight)
{
	SamplePoint sp;
	double r;
	double b[3];

	if( method == RANDOM_BARYCENTRIC_AREA || RANDOM_BARYCENTRIC_WEIGHTED)
	{
		// r, random point in the area
		r = uniform();

		// Find corresponding face
        std::vector<WeightFace>::iterator it = lower_bound(interval.begin(), interval.end(), WeightFace(qMin(r,interval.back().weight)));
        Mesh3d::Face_index f = it->f;

		// Add sample from that face
		RandomBaricentric(b);

        sp = SamplePoint( getBaryFace(f, b[0], b[1]), fnormal[f], weight, f, b[0], b[1]);
	}
	else if( method ==  FACE_CENTER_RANDOM )
	{
		int fcount = mesh->num_of_faces();

		int randTriIndex = (int) (fcount * (((double)rand()) / (double)RAND_MAX)) ;

		if( randTriIndex >= fcount )
			randTriIndex = fcount - 1;

        Mesh3d::Face_index f(randTriIndex);

		// Get triangle center and normal
        sp = SamplePoint(fcenter[f], fnormal[f], farea[f], f, 1 / 3.0, 1 / 3.0);
	}

	return sp;
}

vector<SamplePoint> Sampler::getSamples(int numberSamples, double weight)
{
	vector<SamplePoint> samples;

	if (method == FACE_CENTER_ALL)
	{
		BOOST_FOREACH(Mesh3d::Face_index f_id, mesh.faces())
			samples.push_back(SamplePoint(fcenter[f_id], fnormal[f_id], 0, f_id.idx(), 1 / 3.0, 1 / 3.0));
	}
	else
	{
		for(int i = 0; i < numberSamples; i++)
		{
			samples.push_back(getSample(weight));
		}
	}
	return samples;
}

Point3d Sampler::getBaryFace( Mesh3d::Face_index f_id, double U, double V )
{
    // vector<Vector3d> v;
    // Surface_mesh::Vertex_around_face_circulator vit = mesh->vertices(f),vend=vit;
    // do{ v.push_back(points[vit]); } while(++vit != vend);
    std::vector<Point3d> v;
    CGAL::Vertex_around_face_iterator<Mesh3d> vbegin, vend;
    for(boost::tie(vbegin, vend) = CGAL::vertices_around_face(m_mesh.halfedge(face), m_mesh);;
        vbegin != vend; 
        ++vbegin){
        result.push_back(m_mesh.point(*vbegin));
    }

    if(U == 1.0) return v[1];
    if(V == 1.0) return v[2];

    double b1 = U;
    double b2 = V;
    double b3 = 1.0 - (U + V);

    Point3d p;
    p.x() = (b1 * v[0].x()) + (b2 * v[1].x()) + (b3 * v[2].x());
    p.y() = (b1 * v[0].y()) + (b2 * v[1].y()) + (b3 * v[2].y());
    p.z() = (b1 * v[0].z()) + (b2 * v[1].z()) + (b3 * v[2].z());
    return p;
}

