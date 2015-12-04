/**
This code comes from this paper:

Hu, Ruizhen, et al. "Interaction Context (ICON): Towards a Geometric Functionality Descriptor." ACM Transactions on Graphics 34.
*/

#pragma once

#include "SurfaceMeshModel.h"
#include "Sampler.h"
#include "RenderObjectExt.h"

class FuncRegion;
class Scene;
class Object;
using namespace SurfaceMesh;

class IBS
{

public:
	IBS();
	IBS(Scene *s);
	IBS(std::vector<IBS*> ibsSet, std::vector<bool> reverseNormal);
	~IBS();

public:
	void draw(bool drawIbsSample = false, bool drawIbsWeight = false, QColor color = Qt::red);
	void computeGeomFeatures();			// geometry features
	void computeTopoFeatures();			// topology features
	void computeSampleWeightForTri();

private:
	void sampling(int num);

	void computeDirHist();
	void computeDistHist();
	void computePFH();
	std::vector<double> computePfhForSample(int sIdx, bool reverseNormal);

	void computeBettiNumbers();
	void computeBettiNumbers2();		// transfer original copy from Xi, extremely slow
	void ignoreSmallHoles();

	// implementation for community features
	std::vector<double> combinedPFH(std::vector<IBS*> ibsSet, std::vector<bool> reverseNormal);
	std::vector<double> computePfhForSample(int ibsIdx, int sIdx, std::vector<IBS*> ibsSet, std::vector<bool> reverseNormal);
	std::vector<double> combinedDirHist(std::vector<IBS*> ibsSet, std::vector<bool> reverseNormal);
	std::vector<double> combinedDistHist(std::vector<IBS*> ibsSet);
	std::vector<int> combinedBettiNumber(std::vector<IBS*> ibsSet);

public:
	//Scene *scene;
	std::vector<Object *> objects;
	FuncRegion* region;
	Vector3d upright;

	Object * obj1;
	Object * obj2; // normal points toward obj2 by default; for IBS beteween interacing object and central object, this is always the idx for the central object 

	bool pointToCentralObject; // true is objIdx2 is the central object (objIdx2 = ibsSetScene[i]->obj2-origIdx[0])

	Object* ibsObj;	
	std::vector<std::pair<int, int>> samplePairs;  // the pair of samples on the objects that corresponds the triangle

	// importance-based sampling
	double sampleRatio;  // for smaller IBS, the number of samples should be smaller
	std::vector<SamplePoint> samples;
	//starlab::PointSoup sampleRender;
	double maxWeight;
	double totalWeight;

	// Geometric features
	std::vector<double> pfh;		// point feature histogram
	std::vector<double> dirHist;	// direction histogram
	std::vector<double> distHist;	// distance histogram

	// Topological features
	std::vector<int> bettiNumbers;
};