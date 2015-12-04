/**
This code comes from this paper:

Hu, Ruizhen, et al. "Interaction Context (ICON): Towards a Geometric Functionality Descriptor." ACM Transactions on Graphics 34.
*/

#pragma once

#include <vector>
#include <map>

#include "../Object.h"
// #include "IBS.h"
// #include "Scene.h"
#include "Qhull.h"
#include "../Mesh.h"
#include "../Debug/DebugTools.h"
// #include "SubTreeSimilarity.h"

class IbsGenerator
{

public:
	IbsGenerator();
	~IbsGenerator();

public:
	std::vector<Mesh> computeIBSForEachTwoObjs(std::vector<Object> objs);
	std::vector<Mesh> computeIBS(/*Scene * scene, */std::vector<Object> objects);
	void reset();

private:
	void computeVoronoi();
	void findRidges();
	void buildIBS();
	
	std::vector<Point3d> getInputForVoronoi();	
	int findRidgesAroundVertex(vertexT *atvertex);  // find all unvisited Voronoi ridges for vertex (i.e., an input site)
	Mesh buildIbsMesh(int i,  std::vector<std::pair<int, int>>& samplePairs);

private:
	// Scene *scene;
	// std::vector<IBS*> ibsSet;
	std::vector<Mesh> meshSet;
	//std::vector<int> activeObjIdx;		// useless, can be commented
	std::vector<Object> objects;

private:
	orgQhull::Qhull *qhull;	
	std::vector<Point3d> voronoiVertices;

	// same size with ibsSet
	std::map<std::pair<int, int>, int> objPair2IbsIdx;
	std::vector< std::vector<int> > ibsRidgeIdxs;	// using objPair2IbsIdx to index this std::vector

	std::vector< std::vector<int> > ridges;
	std::vector< int* > ridgeSitePair;

	std::vector<int> sampleObjIdx;				//the corresponding objectIdx for each sample points
	std::vector<int> sampleLocalIdx;

	DebugTimer timer;
};