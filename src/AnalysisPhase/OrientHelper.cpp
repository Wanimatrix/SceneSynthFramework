/**
This code comes from this paper:

Hu, Ruizhen, et al. "Interaction Context (ICON): Towards a Geometric Functionality Descriptor." ACM Transactions on Graphics 34.
*/
#include "OrientHelper.h"

#include <algorithm>

OrientHelper::OrientHelper()
{

}

std::vector<std::vector<int>> OrientHelper::reorient( std::vector<std::vector<int>> faces, int vNum)
{
	this->faces = faces;
	this->vNum = vNum;

	getAdjacency();
	
	isOriented.resize(faces.size());

	int idx = findUnorientedFace();

	while ( idx != -1 ) // a small bug needs to be fixed: the orientation of second component may be consistent with the first one...
	{
		isOriented[idx] = true;

		std::stack<std::pair<int, int>> tStack;
		for (int i=0; i<adjacency[0].size(); i++)
		{
			tStack.push(std::pair<int, int>(i, 0));
		}

		while (!tStack.empty())
		{
			std::pair<int, int> t = tStack.top();
			tStack.pop();
			reorientFace(t.first, t.second);		// t.second: original index order, t.first: neighbor
			int f = adjacency[t.second][t.first];
			for (int i=0; i<adjacency[f].size(); i++)
			{
				int neighF = adjacency[f][i];
				if (!isOriented[neighF])
				{
					tStack.push(std::pair<int, int>(i, f));			
				}
			}
		}

		idx = findUnorientedFace();			// traversing all faces
	}

	
	//// other option: use recursive algorithm
	//reorientNeighbors(0);

	return this->faces;
}

void OrientHelper::getAdjacency( )
{
	// 1. find all the edges and the faces sharing the same edge
	std::map<std::pair<int, int>, std::vector<int>> edge2faces;
	for (int i=0; i<faces.size(); i++)
	{
		for (int j=0; j<faces[i].size(); j++)
		{
			int k = (j+1) % faces[i].size();

			int vIdx1 = faces[i][j];
			int vIdx2 = faces[i][k];

			if(vIdx1 > vIdx2)
			{
				int temp = vIdx1;
				vIdx1 = vIdx2;
				vIdx2 = temp;
			}
			// keep only one pair of corresponding edge, no need to consider (vIdx1, vIdx2) and (vIdx2, vIdx1)

			std::pair<int, int> edge(vIdx1, vIdx2);

			std::map<std::pair<int, int>, std::vector<int>>::iterator imap = edge2faces.find(edge);
			if (imap == edge2faces.end())
			{
				std::vector<int> f;
				f.push_back(i);

				edge2faces.insert(std::pair<std::pair<int, int>, std::vector<int>>(edge, f));
			}
			else
			{
				imap->second.push_back(i);

				if(imap->second.size() > 2)
				{
					std::cerr << "ERROR: more than two faces share an edge!!!" << std::endl;
				}
			}
		}
	}

		
	// 2. get adjacent faces and the sharing edge
	adjacency.resize(faces.size());
	edges.resize(faces.size());
	for (std::map<std::pair<int, int>, std::vector<int>>::iterator imap=edge2faces.begin(); imap!=edge2faces.end(); imap++)
	{
		std::vector<int> adjFaces = imap->second;

		if (adjFaces.size() == 2)
		{
			adjacency[adjFaces[0]].push_back(adjFaces[1]);		// adjacent faces of each face
			adjacency[adjFaces[1]].push_back(adjFaces[0]);

			edges[adjFaces[0]].push_back(imap->first);			// adjacent edges of each face
			edges[adjFaces[1]].push_back(imap->first);
		}
		else if (adjFaces.size() == 1)
		{
			//qDebug() << "edge on the boundary.";
		}
		else
		{
			std::cerr << "ERROR: edge shared by more than two faces!!!" << std::endl;
		}
	}
}

void OrientHelper::reorientNeighbors( int f )
{
	for (int i=0; i<adjacency[f].size(); i++)
	{
		int neighF = adjacency[f][i];
		if (!isOriented[neighF])
		{
			reorientFace(i, f);		
			reorientNeighbors(neighF);
		}
	}
}

void OrientHelper::reorientFace( int neighIdx, int f )
{
	int neighF = adjacency[f][neighIdx];		
	std::pair<int, int> edge = edges[f][neighIdx];

	int order1 = indexOf(faces[f],edge.first) - indexOf(faces[f],edge.second);
	order1 = order1 > 1 ? -1 : order1;
	order1 = order1 < -1 ? 1 : order1;
	int order2 = indexOf(faces[neighF],edge.first) - indexOf(faces[neighF],edge.second);
	order2 = order2 > 1 ? -1 : order2;
	order2 = order2 < -1 ? 1 : order2;

	if (order1 * order2 > 0)
	{
		std::vector<int> reverFace;
		for (int i = faces[neighF].size()-1; i>=0; i--)
		{
			reverFace.push_back(faces[neighF][i]);
		}
		faces[neighF] = reverFace;

		//for (int i=0; i<faces[neighF].size()/2; i++)
		//{
		//	int tmp = faces[neighF][i];
		//	faces[neighF][i] = faces[neighF][faces[neighF].size() - 1 - i];
		//	faces[neighF][faces[neighF].size() - 1 - i] = tmp;
		//}
	}

	isOriented[neighF] = true;
}

int OrientHelper::findUnorientedFace()
{
	int idx = -1;
	for (int i=0; i<isOriented.size(); i++)
	{
		if (!isOriented[i])
		{
			idx = i;
			break;
		}
	}

	return idx;
}

int OrientHelper::indexOf(const std::vector<int> &vec, const int &value) const
{
	std::vector<int>::const_iterator vecIt = std::find(vec.begin(),vec.end(),value);
	return (vecIt == vec.end()) ? -1 : (vecIt - vec.begin());
}