/**
This code comes from this paper:

Hu, Ruizhen, et al. "Interaction Context (ICON): Towards a Geometric Functionality Descriptor." ACM Transactions on Graphics 34.
*/

#pragma once

#include "../types.h"

// Helper structures
struct SamplePoint{
	Point3d pos;
    Vector3d n;			// position and normal
	double weight;
	double u,v;
	int findex;						// index of sampled face
	int flag;

	SamplePoint(const Point3d& position = Point3d(), const Vector3d& normal = Vector3d(), 
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
	Face f;

    WeightFace(double a = 0.0, Face face = Face(-1)) : weight(a), f(face){}

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
    std::vector<SamplePoint> getSamples(int numberSamples, double weight = 0.0);

    Mesh3d mesh;
	SamplingMethod method;

	// For Monte Carlo
    std::vector<WeightFace> interval;
   
	FaceProperty<double> farea;
    FaceProperty<Vector3d> fnormal;
	FaceProperty<Point3d> fcenter;
    VertexProperty<Point3d> points;

    Point3d getBaryFace(Face f, double U, double V);
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
