/**
This code comes from this paper:

Hu, Ruizhen, et al. "Interaction Context (ICON): Towards a Geometric Functionality Descriptor." ACM Transactions on Graphics 34.
*/
#include "IbsGenerator.h"
#include "QhullFacetList.h"
#include "QhullVertexSet.h"
#include "QhullError.h"
#include "OrientHelper.h"
#include <exception>
#include <memory>

// #include "UtilityGlobal.h"

#define PI 3.14159265359

IbsGenerator::IbsGenerator()
{
	// scene = NULL;
	qhull = NULL;
}

IbsGenerator::~IbsGenerator()
{
	if (qhull)
	{
		delete qhull;
	}
	for(int i = 0; i < ridgeSitePair.size(); i++) {
		if(ridgeSitePair[i]) delete[] ridgeSitePair[i];
	}
}

void IbsGenerator::reset() {
	if(qhull) delete qhull;
	qhull = NULL;
	voronoiVertices.clear();

	objPair2IbsIdx.clear();
	ibsRidgeIdxs.clear();

	ridges.clear();
	for(int i = 0; i < ridgeSitePair.size(); i++) {
		if(ridgeSitePair[i]) delete[] ridgeSitePair[i];
	}
	ridgeSitePair.clear();

	sampleObjIdx.clear();
	sampleLocalIdx.clear();

	meshSet.clear();
	objects.clear();
}

std::vector<Mesh> IbsGenerator::computeIBSForEachTwoObjs(std::vector<Object> objs)
{
	std::vector<Mesh> result;
	int i;
	for (i = 0; i < objs.size(); ++i)
	{
		for (int j = i+1; j < objs.size(); ++j)
		{
			if (qhull) reset();
			timer.start();
			std::vector<Mesh> meshes = computeIBS(std::vector<Object>({objs[i],objs[j]}));
			timer.printElapsedTime("IBS");
			result.insert(result.end(),meshes.begin(),meshes.end());
		}
	}
	
	return result;
}

std::vector<Mesh> IbsGenerator::computeIBS(std::vector<Object> objs)
{
	// scene = s;
	objects = objs;

	computeVoronoi();
	findRidges();
	buildIBS();

	// return ibsSet;
	return meshSet;
}

void IbsGenerator::computeVoronoi()
{
	std::vector<Point3d> points = getInputForVoronoi();
	std::vector<double> pointCoords;

	for(int i = 0; i < points.size()*3; i+=3) {
		pointCoords.push_back(points[i/3].x());
		pointCoords.push_back(points[i/3].y());
		pointCoords.push_back(points[i/3].z());
	}

	// use Qhull to create Voronoi diagram
	if(!qhull) qhull = new orgQhull::Qhull("", 3, (int)points.size(), pointCoords.data(), "v");
	else throw std::invalid_argument( "qhull was already initialized!" );
	//qhull->runQhull("", 3, (int)points.size(), pointCoords.data(), "v");
	
	voronoiVertices.push_back(Point3d(0, 0, 0)); // the index of vertices start from 1, 0 is for the infinite one
	for ( orgQhull::QhullFacet facet = qhull->firstFacet(); facet != qhull->endFacet(); facet=facet.next() )
	{
		orgQhull::QhullPoint p = facet.voronoiVertex();
		voronoiVertices.push_back(Point3d(p[0], p[1], p[2]));
	}

	//foreach(orgQhull::QhullFacet facet, qhull->facetList()) {
	//	orgQhull::QhullPoint p = facet.voronoiVertex(qhull->runId());
	//	voronoiVertices.push_back(Point3d(p[0], p[1], p[2]));
	//}
}

std::vector<Point3d> IbsGenerator::getInputForVoronoi()
{
	sampleObjIdx.clear();
	sampleLocalIdx.clear();
	std::vector<Point3d> points;

	// 1. get the sample points on the objects
	for (int i=0; i<objects.size(); i++)
	{
		Object obj = objects[i];

		int idx = 0;
		for (auto sample : obj.getSamples())
		{
			points.push_back(sample.pos);
			sampleObjIdx.push_back(i);			// index the corresponding object index
			sampleLocalIdx.push_back(idx++);	// index in each object
		}
	}

	// 2. add sample points on the bounding ball
	// Bbox3d bbox = objects[0]->getBbox();
	// for (auto obj:objects)
	// {
	// 	for(int i = 0; i < 3;i++) {
	// 		std::min(bbox.min_coord(i),obj->getBbox().min_coord(i))
	// 		std::max(bbox.max_coord(i),obj->getBbox().max_coord(i))
	// 	}
	// 	bbox.extend(obj->bbox);
	// }
	IsoCub3d bboxCuboid = IsoCub3d(CGAL::bbox_3(std::begin(objects),std::end(objects)));

	double radius = std::sqrt((bboxCuboid.max()-bboxCuboid.min()).squared_length()) * 2 / 3;
	Point3d center = CGAL::midpoint(bboxCuboid.max(),bboxCuboid.min());

	int M = 20; 
	int N = 40;
	double step_z = PI / M;
	double step_xy = 2*PI / N;

	double angle_z = 0.0;
	double angle_xy = 0.0;

	for(int i=0; i<M; i++)
	{
		angle_z = i * step_z;

		for(int j=0; j<N; j++)
		{
			angle_xy = j * step_xy;

			double x = radius * sin(angle_z) * cos(angle_xy);
			double y = radius * sin(angle_z) * sin(angle_xy);
			double z = radius * cos(angle_z);

			points.push_back(Point3d(x + center[0], y + center[1], z + center[2]));
			sampleObjIdx.push_back(-1);				// not on the object, but the bounding sphere, so no local index
		}
	}

	//// 2. add samples on the bounding box
	//Vector3d v_min = bbox.center() - bbox.diagonal() * 1.1;
	//Vector3d v_max = bbox.center() + bbox.diagonal() * 1.1;
	//int stepNum = 10;
	//Vector3d step = (v_max - v_min) / stepNum;
	//
	//for (int i=0; i<=stepNum; i++)
	//{
	//	if (i==0 || i==stepNum)
	//	{
	//		for (int j=0; j<=stepNum; j++)
	//		{
	//			for (int k=0; k<=stepNum; k++)
	//			{
	//				points.push_back(Vector3d(v_min[0]+i*step[0], v_min[1]+j*step[1], v_min[2]+k*step[2]));
	//				objIdx.push_back(-1);
	//			}
	//		}
	//	}
	//	else
	//	{
	//		for (int j=0; j<=stepNum; j+=stepNum)
	//		{
	//			for (int k=0; k<=stepNum; k+=stepNum)
	//			{
	//				points.push_back(Vector3d(v_min[0]+i*step[0], v_min[1]+j*step[1], v_min[2]+k*step[2]));
	//				objIdx.push_back(-1);
	//			}
	//		}
	//	}
	//}

	return points;	
}

// refer the function, qh_printvdiagram in io.c in QHull library.

/*
 * visit all pairs of input sites (vertices) for selected Voronoi vertices
*/

void IbsGenerator::findRidges()
{	
	int numcenters;
	unsigned int isLower;
	facetT *facetlist = qhull->firstFacet().getFacetT();
	setT * vertices = qh_markvoronoi(qhull->qh(),facetlist, 0, qh_True, &isLower, &numcenters);  // vertices are the input point sites, indexed by pointid

	vertexT *vertex;
	int vertex_i, vertex_n;

	qhT *qh = qhull->qh();
	FORALLvertices
		vertex->seen= False;

	int totcount = 0;
	objPair2IbsIdx.clear();

	std::cout << "Amount ridges: " << qh->ridgeoutnum << std::endl;


	FOREACHvertex_i_(qhull->qh(),vertices) 
	{
		if (vertex) 
		{
			if (qhull->qh()->GOODvertex > 0 && qh_pointid(qhull->qh(),vertex->point)+1 != qhull->qh()->GOODvertex)
				continue;
			totcount += findRidgesAroundVertex(vertex);
		}
	}
}

/// libqhull\io.c -> qh_eachvoronoi
int IbsGenerator::findRidgesAroundVertex(vertexT *atvertex)
{
	// parameters
	qh_RIDGE innerouter = qh_RIDGEall;
	printvridgeT printvridge = qh_printvridge;
	boolT inorder = true;
	boolT visitall = !qh_ALL;

	boolT unbounded;
	int count;
	facetT *neighbor, **neighborp, *neighborA, **neighborAp;
	setT *centers;
	setT *tricenters= qh_settemp(qhull->qh(),qhull->qh()->TEMPsize);

	vertexT *vertex, **vertexp;
	boolT firstinf;
	unsigned int numfacets= (unsigned int)qhull->qh()->num_facets;
	int totridges= 0;

	qhull->qh()->vertex_visit++;
	atvertex->seen= True;
	if (visitall) {
		qhT *qh = qhull->qh();
		FORALLvertices
			vertex->seen= False;
	}
	FOREACHneighbor_(atvertex) {
		if (neighbor->visitid < numfacets)
			neighbor->seen= True;
	}
	FOREACHneighbor_(atvertex) {
		if (neighbor->seen) {
			FOREACHvertex_(neighbor->vertices) {
				if (vertex->visitid != qhull->qh()->vertex_visit && !vertex->seen) {
					vertex->visitid= qhull->qh()->vertex_visit;
					count= 0;
					firstinf= True;
					qh_settruncate(qhull->qh(),tricenters, 0);
					FOREACHneighborA_(vertex) {
						if (neighborA->seen) {
							if (neighborA->visitid) {
								if (!neighborA->tricoplanar || qh_setunique(qhull->qh(),&tricenters, neighborA->center))
									count++;
							}else if (firstinf) {
								count++;
								firstinf= False;
							}
						}
					}

					if (count >= qhull->qh()->hull_dim - 1) { // Each ridge has to have at least hull_dim - 1 vertices
						if (firstinf) {
							if (innerouter == qh_RIDGEouter)
								continue;
							unbounded= False;
						}else {
							if (innerouter == qh_RIDGEinner)
								continue;
							unbounded= True;
						}
						totridges++;
						// QhullError(qhull->qh()->ferr, 4017, "qh_eachvoronoi: Voronoi ridge of %d vertices between sites %d and %d\n",
						// 	count, qh_pointid(qhull->qh(),atvertex->point), qh_pointid(qhull->qh(),vertex->point)); // TODO
						if (printvridge) {
							if (inorder && qhull->qh()->hull_dim == 3+1) /* 3-d Voronoi diagram */
								centers= qh_detvridge3(qhull->qh(),atvertex, vertex); // determine 3-d Voronoi ridge from 'seen' neighbors of atvertex and vertex, listed in adjacency order (not oriented)
							else
								centers= qh_detvridge(qhull->qh(),vertex);
							// centers : set of facets (i.e., Voronoi vertices)

							// save the new ridge to our own data structure
							int *sites = nullptr;
							{
								/*std::vector< int > sites; // pair of input sites separated by this ridge
								sites.push_back( qh_pointid(qhull->qh(),atvertex->point) );
								sites.push_back( qh_pointid(qhull->qh(),vertex->point) );*/

								sites = new int[2];
								sites[0] = qh_pointid(qhull->qh(),atvertex->point);
								sites[1] = qh_pointid(qhull->qh(),vertex->point);
								
								bool upperdelaunay = false;
								facetT *facet, **facetp;
								std::vector<int> ridge;
								FOREACHfacet_(centers)			// the bisecting planer has more than one edges, which corresponds one ridge
								{
									ridge.push_back(facet->visitid);
									if ( facet->upperdelaunay )
									{
										upperdelaunay = true;
										break;
									}
								}

								// only add bounded
								if ( !upperdelaunay )
								{					
									int obj_id_1 = sampleObjIdx[sites[0]];
									int obj_id_2 = sampleObjIdx[sites[1]];	

									if ( obj_id_2 < obj_id_1 ) 
									{
										std::swap(obj_id_1, obj_id_2);
										std::swap(sites[0], sites[1]);
									}

									ridgeSitePair.push_back(sites);
									ridges.push_back(ridge);

									if ( obj_id_1 != -1 && obj_id_2 != -1 && ( obj_id_1 != obj_id_2 ) )
									{
										std::pair<int, int> obj_id_pair(obj_id_1, obj_id_2);
										if ( objPair2IbsIdx.find(obj_id_pair) == objPair2IbsIdx.end() ) // check whether there is already ridge found between those two objects
										{
											// create a ridge list for a new ibs separating those two objects
											std::vector<int> ridge_id_list;
											ibsRidgeIdxs.push_back(ridge_id_list);

											// map the new object pair to the ridge list
											objPair2IbsIdx[obj_id_pair] = (int)ibsRidgeIdxs.size()-1;
										}

										//update ridge_id_list in IBS
										int ibsIdx = objPair2IbsIdx[obj_id_pair];
										ibsRidgeIdxs[ibsIdx].push_back(ridges.size()-1);										
									}						
								}
								
							}

							qh_settempfree(qhull->qh(),&centers);
						}
					}
				}
			}
		}
	}
	FOREACHneighbor_(atvertex)
		neighbor->seen= False;
	qh_settempfree(qhull->qh(),&tricenters);
	return totridges;
} 

void IbsGenerator::buildIBS()
{
	// ibsSet.clear();
	meshSet.clear();
	for (int i=0; i<ibsRidgeIdxs.size(); i++)
	{
		// IBS * ibs = new IBS(scene);

		// // 1. set the corresponding pair of objects
		// pair<int, int> objPair = objPair2IbsIdx.key(i);
		// ibs->obj1 = objects[objPair.first];
		// ibs->obj2 = objects[objPair.second];

		// // 2. build the mesh & get the corresponding sample pairs
		// ibs->mesh = buildIbsMesh(i, ibs->samplePairs);	

		// ibsSet.push_back(ibs);
		std::vector<std::pair<int, int>> samplePairs;
		meshSet.push_back(buildIbsMesh(i, samplePairs)); // ibs->samplePairs
	}
}

Mesh IbsGenerator::buildIbsMesh( int i,  std::vector<std::pair<int, int>>& samplePairs )
{
	//////////////////////////////////////////////////////////////////////////
	// 1. re-index the vertices and update the ridges
	std::vector<int> vIdx;							// vertices
	std::vector<int> vIdxNew(voronoiVertices.size(), -1);
	std::vector< std::vector<int> > ridgesNew;			// faces
	std::vector<int> remainRidgeIdx;

	for (int j=0; j < ibsRidgeIdxs[i].size(); j++)
	{
		int ridgeIdx = ibsRidgeIdxs[i][j];
		std::vector<int> ridge = ridges[ridgeIdx];
		std::vector<int> ridgeWithNewId; 

		if(ridge.size() < 3)						// a face must contain more than 2 edges(ridges)
		{
			continue;
		}

		//find all the vertices on this ridge
		for(int k = 0; k < ridge.size(); k++)
		{
			int v_id = ridge[k];					// vertice_id(visited_id)
		
			if(vIdxNew[v_id] == -1)					// duplicated edges are not counted
			{
				vIdxNew[v_id] = vIdx.size();		// counting from 0, locally
				vIdx.push_back(v_id);				// preserve the original index of ridges[k]
			}

			// renew the vertex index
			ridgeWithNewId.push_back(vIdxNew[v_id]);
		}
		
		ridgesNew.push_back(ridgeWithNewId);
		remainRidgeIdx.push_back(ridgeIdx);			// remainRidgeIdx = ibsRidgeIbs[i]
	}
	
	// TODO
	// //////////////////////////////////////////////////////////////////////////
	// // 2. re-orient all faces coherently
	OrientHelper help;
	ridgesNew = help.reorient(ridgesNew, vIdx.size());	// ridgesNew stores new index

	//////////////////////////////////////////////////////////////////////////
	// 3. make the normal point from obj1 to obj2
	// get the first ridge triangle to see if its normal points to obj2
	std::vector<Point3d> fv;
	fv.push_back(voronoiVertices[vIdx[ridgesNew[0][0]]]);
	fv.push_back(voronoiVertices[vIdx[ridgesNew[0][1]]]);
	fv.push_back(voronoiVertices[vIdx[ridgesNew[0][2]]]);
	Vector3d n = CGAL::cross_product((fv[2] - fv[1]),(fv[0] - fv[1])); // .normalized()
	n = n / std::sqrt(n.squared_length());
	Point3d center = Point3d(
		(fv[0].x() + fv[1].x() + fv[2].x())/3.0,
		(fv[0].y() + fv[1].y() + fv[2].y())/3.0,
		(fv[0].z() + fv[1].z() + fv[2].z())/3.0);
	// Vector3d center = (fv[0] + fv[1] + fv[2]) / 3.0;

	int ridge_id = ibsRidgeIdxs[i][0];
	int *pair = ridgeSitePair[ridge_id];
	int sIdx = pair[1];
	Point3d s2 = objects[sampleObjIdx[sIdx]].getSamples()[sampleLocalIdx[sIdx]].pos;
	Vector3d d = (s2 - center); // .normalized();
	d = d / std::sqrt(d.squared_length());

	// n is the normal of Voronoi faces, and d is the direction pointing from center to s2
	bool flip = (n * d) < 0;	// if the normal is not pointing to objs, flip the mesh

	//////////////////////////////////////////////////////////////////////////
	// 4. build the mesh
	std::shared_ptr<Mesh3d> mesh3d(new Mesh3d());

	// add vertices
	std::vector<Mesh3d::Vertex_index> cgal_vertIdxes;
	for (int j=0; j<vIdx.size(); j++) 
	{
		Point3d vert = voronoiVertices[vIdx[j]];
		cgal_vertIdxes.push_back(mesh3d->add_vertex(vert));
	}

	// add faces
	Face f;
	bool created;
	FaceProperty<Vector3d> faceNormals;
    boost::tie(faceNormals, created) = mesh3d->add_property_map<Face,Vector3d>("f:normal");
    VertexProperty<Point3d> points = mesh3d->points();
	for(int j=0; j<ridgesNew.size(); j++)
	{
		int *pair = ridgeSitePair[remainRidgeIdx[j]];
		// add the triangle fan for each polygon
		for (int k=1; k<ridgesNew[j].size()-1; k++)
		{
			if (flip)
			{
				//f = mesh3d->add_face(cgal_vertIdxes[ridgesNew[j][k+1]],cgal_vertIdxes[ridgesNew[ibsRidgeIdxs[i][j]][k]],cgal_vertIdxes[ridgesNew[ibsRidgeIdxs[i][j]][0]]);
				 f = mesh3d->add_face(cgal_vertIdxes[ridgesNew[j][k+1]], cgal_vertIdxes[ridgesNew[j][k]], cgal_vertIdxes[ridgesNew[j][0]]);
				 faceNormals[f] = CGAL::normal(points[cgal_vertIdxes[ridgesNew[j][k+1]]], points[cgal_vertIdxes[ridgesNew[j][k]]], points[cgal_vertIdxes[ridgesNew[j][0]]]);
			}
			else
			{
				//f = mesh3d->add_face(cgal_vertIdxes[ridgesNew[ibsRidgeIdxs[i][j]][0]],cgal_vertIdxes[ridgesNew[ibsRidgeIdxs[i][j]][k]],cgal_vertIdxes[ridgesNew[ibsRidgeIdxs[i][j]][k+1]]);
				f = mesh3d->add_face(cgal_vertIdxes[ridgesNew[j][0]], cgal_vertIdxes[ridgesNew[j][k]], cgal_vertIdxes[ridgesNew[j][k+1]]);
				faceNormals[f] = CGAL::normal(points[cgal_vertIdxes[ridgesNew[j][0]]], points[cgal_vertIdxes[ridgesNew[j][k]]], points[cgal_vertIdxes[ridgesNew[j][k+1]]]);
			}	

			if (f != -1)//(f.is_valid())
			{	
				samplePairs.push_back(std::pair<int, int>(sampleLocalIdx[pair[0]], sampleLocalIdx[pair[1]]));
			}


		}
	}

	if (samplePairs.size() != mesh3d->number_of_faces())
	{
		std::cout << "ERROR: the number of sample pairs does not equal to that of triangles!!!  " << samplePairs.size() << " vs. " << mesh3d->number_of_faces() << std::endl;
	}

	return Mesh(mesh3d);
}
