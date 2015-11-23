#pragma once

#include "Object.h"

using namespace std;

// Helper structures
struct SamplePoint{
	Vector3d pos, n;			// position and normal
	double weight;
	double u,v;
	int findex;						// index of sampled face
	int flag;

	SamplePoint(const Vector3d& position = Vector3d(), const Vector3d& normal = Vector3d(), 
		double Weight = 0.0, int face_index = -1.0, double U = 0.0, double V = 0.0, int flags = 0)
	{
		pos = position;
		n = normal;
		weight = Weight;
		findex = face_index;
		u = U;
		v = V;
		flag = flags;
	}
};

struct WeightFace{
	double weight;
	Mesh3d::Face_vertex f;

    WeightFace(double a, Mesh3d::Face_vertex face) : weight(a), f(face){}

	bool operator< (const WeightFace & af) const { return weight < af.weight; }
	void setValue (double val) { weight = val; }
};

enum SamplingMethod {  RANDOM_BARYCENTRIC_AREA,  RANDOM_BARYCENTRIC_WEIGHTED, FACE_CENTER_RANDOM, FACE_CENTER_ALL };

// Class definition
class Sampler{

private:

public:
    Sampler(const Mesh3d &srcMesh, SamplingMethod samplingMethod = RANDOM_BARYCENTRIC_AREA );
	//Sampler(void * srcMesh, SamplingMethod samplingMethod);
	
	// Get samples
	SamplePoint getSample(double weight = 0.0);
    vector<SamplePoint> getSamples(int numberSamples, double weight = 0.0);

    Mesh mesh;
	SamplingMethod method;

	// For Monte Carlo
    vector<WeightFace> interval;
   
	Mesh3d::Property_map<Mesh3d::Face_index,float> farea;
    Mesh3d::Property_map<Mesh3d::Face_index,Vector3d> faceNormals;
	Mesh3d::Property_map<Mesh3d::Face_index,Point3d> fcenter;
    Mesh3d::Property_map<Mesh3d::Vertex_index,Point3d> points;

    Vector3d getBaryFace(Mesh3d::Face_vertex f, double U, double V);
};

// Helper functions
double inline uniform(double a = 0.0, double b = 1.0){
    double len = b - a;
    return ((double)rand()/RAND_MAX) * len + a;
}

static inline void RandomBaricentric(double * interp){
	interp[1] = uniform();
	interp[2] = uniform();

	if(interp[1] + interp[2] > 1.0)
	{
		interp[1] = 1.0 - interp[1];
		interp[2] = 1.0 - interp[2];
	}

	interp[0] = 1.0 - (interp[1] + interp[2]);
}
