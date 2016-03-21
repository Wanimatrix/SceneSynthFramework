/**
This code comes from this paper:

Hu, Ruizhen, et al. "Interaction Context (ICON): Towards a Geometric Functionality Descriptor." ACM Transactions on Graphics 34.
*/

#pragma once

#include "../types.h"
#include "../Mesh.h"
#include <Eigen/Dense>
#include <memory>
#include <random>
#include "../Debug/DebugTools.h"

// Helper structures
struct SamplePoint{
    Point3d pos;
    Vector3d n;            // position and normal
    double weight;
    double u,v;
    int findex;                        // index of sampled face
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

enum SamplingMethod {  RANDOM_BARYCENTRIC_AREA,  RANDOM_BARYCENTRIC_WEIGHTED, RANDOM_BARYCENTRIC_WEIGHTED_DISTANCE, FACE_CENTER_RANDOM, FACE_CENTER_ALL };

// Class definition
class Sampler{

private:

public:
    Sampler(const Mesh &srcMesh, SamplingMethod samplingMethod = RANDOM_BARYCENTRIC_AREA );
    //Sampler(void * srcMesh, SamplingMethod samplingMethod);
    
    // Get samples
    SamplePoint getSample(double weight = 0.0);
    std::vector<SamplePoint> getSamples(int numberSamples, double weight = 0.0);

    std::shared_ptr<Mesh3d> mesh;
    SamplingMethod method;

    // For Monte Carlo
    std::vector<WeightFace> interval;
   
    FaceProperty<double> farea;
    FaceProperty<Vector3d> fnormal;
    FaceProperty<Point3d> fcenter;
    FaceProperty<Point3d> fpt;
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

static inline double getNormalMean(Eigen::VectorXd p, Eigen::VectorXd lb, Eigen::VectorXd le) {
  double A[2] = {p[0]-lb[0],lb[0]-le[0]};
  double B[2] = {p[1]-lb[1],lb[1]-le[1]};
  double C[2] = {p[2]-lb[2],lb[2]-le[2]};

  double q = A[1]*A[1] + B[1]*B[1] + C[1]*C[1];
  double r = 2*(A[0]*A[1] + B[0]*B[1] + C[0]*C[1]);
  double s = A[0]*A[0] + B[0]*B[0] + C[0]*C[0];


  Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(1/0.0001,0,1);
  Eigen::VectorXd D = (q*t.cwiseProduct(t) + r*t).array() + s;
  int index;
  D.minCoeff(&index);
  return t[index];
}

static inline Eigen::VectorXd getClosestPointOnLine(Eigen::VectorXd p, Eigen::VectorXd lb, Eigen::VectorXd le) {
  double Q = lb.cwiseProduct(lb).sum();
  double R = le.cwiseProduct(le).sum();
  double S = lb.cwiseProduct(le).sum();
  double T = p.cwiseProduct(lb).sum();
  double U = p.cwiseProduct(le).sum();

  double t = -(T-U-Q+S) / (Q + R - 2*S);

  return lb + (le - lb)*t;
}

static inline double distanceToLine(Eigen::VectorXd p, Eigen::VectorXd lb, Eigen::VectorXd le) {
  Eigen::VectorXd p2 = getClosestPointOnLine(p,lb,le);
  return std::sqrt((p2-p).cwiseProduct(p2-p).sum());
}

static inline Point3d weightedBaricentric(Eigen::VectorXd p, Eigen::VectorXd v0, Eigen::VectorXd v1, Eigen::VectorXd v2){
  std::default_random_engine generator;
  double stddev = 0.12;

  Eigen::VectorXd lb1;
  Eigen::VectorXd le1;
  Eigen::VectorXd lb2;
  Eigen::VectorXd le2;

  double d0 = distanceToLine(p,v0,v1);
  double d1 = distanceToLine(p,v0,v2);
  double d2 = distanceToLine(p,v1,v2);

  if(d0 < d1 || d0 < d2) {
    lb1 = v0;
    le1 = v1;
    if(d1 < d2) {
      lb2 = v0;
      le2 = v2;
    } else {
      std::swap(lb1,le1);
      lb2 = v1;
      le2 = v2;
    }

    if(!(d0 < d1 && d0 < d2)) {
      std::swap(lb1,lb2);
      std::swap(le1,le2);
    }
  } else {
    lb1 = v2;
    le1 = v0;
    lb2 = v2;
    le2 = v1;
    if(d2 < d1) {
      std::swap(lb1,lb2);
      std::swap(le1,le2);
    }
  }

  //maxTerm = max(D)
  //maxT = t[which.max(D)]
  //minT = t[which.min(D)]
  double meanX = getNormalMean(p,lb1,le1);
  std::normal_distribution<double> distributionX(meanX,stddev);
  Eigen::VectorXd p2 = p - (getClosestPointOnLine(p, lb1, le1) - lb1);
  double meanY = getNormalMean(p2,lb2,le2);
  std::normal_distribution<double> distributionY(meanY,stddev);
  double u = 0;
  double v = 0;
  do {
    u = distributionX(generator);
    v = distributionY(generator);
  } while(u >= 0 && u <= 1 && v >= 0 && v <= 1 && u+v < 1);

  double x = lb1[0] + ((le1[0]-lb1[0])*u + (le2[0]-lb2[0])*v);
  double y = lb1[1] + ((le1[1]-lb1[1])*u + (le2[1]-lb2[1])*v);
  double z = lb1[2] + ((le1[2]-lb1[2])*u + (le2[2]-lb2[2])*v);

  return Point3d(x,y,z);
}
