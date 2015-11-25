/**
This code comes from this paper:

Hu, Ruizhen, et al. "Interaction Context (ICON): Towards a Geometric Functionality Descriptor." ACM Transactions on Graphics 34.
*/

#include <vector>
#include <map>
#include <stack>
#include <iostream>

#pragma once

class OrientHelper
{
public:
	OrientHelper();

public:
	std::vector<std::vector<int>> reorient(std::vector<std::vector<int>> faces, int vNum);

private:
	void getAdjacency();
	void reorientNeighbors(int f);
	void reorientFace(int neighIdx, int f); // reorient the neighbor based on the orientation of f
	int findUnorientedFace();
	int indexOf(const std::vector<int> &vec, const int &value) const;

private:
	int vNum;
	std::vector<std::vector<int>> faces;
	std::vector<bool> isOriented;

	std::vector<std::vector<int>> adjacency;			  // the adjacent faces for each face
	std::vector< std::vector< std::pair<int, int> > >  edges; // the edge between each pair of adjacent faces

					// adjacency : adjacent faces of each face
					// edges	 : adjacent edges of each face				
};