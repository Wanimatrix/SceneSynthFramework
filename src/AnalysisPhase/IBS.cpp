/**
This code comes from this paper:

Hu, Ruizhen, et al. "Interaction Context (ICON): Towards a Geometric Functionality Descriptor." ACM Transactions on Graphics 34.
*/

#include "IBS.h"
#include "../Debug/DebugTools.h"
// #include "Scene.h"
//#include "QuickMeshDraw.h"
//#include "RenderObjectExt.h"
//#include "SurfaceMeshHelper.h"
//#include <QTime>
//#include "UtilityGlobal.h"

#define PI 3.1415926

IBS::IBS()
{
	// scene = NULL;
	ibsObj = NULL;
	sampleRatio = 1.0;
	maxWeight = 0;
	totalWeight = 0;
	bettiNumbers.push_back(0);
	bettiNumbers.push_back(0);
	bettiNumbers.push_back(0);
}


IBS::IBS(const std::vector<std::shared_ptr<Object>> &objects)
{
	this->objects = objects;

	std::vector<Object> obj2;
	for(auto obj:objects) {
		obj2.push_back(*obj);
	}
	bboxCuboid = IsoCub3d(CGAL::bbox_3(std::begin(obj2),std::end(obj2)));


	ibsObj = NULL;
	sampleRatio = 1.0;
	maxWeight = 0;
	totalWeight = 0;
	bettiNumbers.push_back(0);
	bettiNumbers.push_back(0);
	bettiNumbers.push_back(0);
}

IBS::~IBS()
{
}

/*void IBS::draw(	bool drawIbsSample, bool drawIbsWeight, QColor color)
{
	if (mesh)
	{
		FaceProperty fweight = mesh->getFaceProperty<Scalar>("f:weight");

		if (!drawIbsWeight || !fweight.is_valid())
		{
			color.setAlphaF(0.2);

			glDepthMask(GL_FALSE);
			QuickMeshDraw::drawMeshSolid(mesh, color, false);	
			glDepthMask(GL_TRUE);
		}
		else
		{
			if(!mesh->property("hasNormals").toBool())
			{
				mesh->update_face_normals();
				mesh->update_vertex_normals();
				mesh->setProperty("hasNormals",true);
			}

			glDepthMask(GL_FALSE);
			glEnable (GL_BLEND);
			glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDisable(GL_LIGHTING);		
			
			Vector3VertexProperty points = mesh->vertex_property<Vector3>("v:point");
			Vector3FaceProperty fnormals = mesh->face_property<Vector3>("f:normal");

			Surface_mesh::Face_iterator fit, fend = mesh->faces_end();
			Surface_mesh::Vertex_around_face_circulator fvit, fvend;

			glBegin(GL_TRIANGLES);
			for (fit=mesh->faces_begin(); fit!=fend; ++fit){
				QColor c = starlab::qtJetColor(fweight[fit], 0 , maxWeight);
				c.setAlphaF(0.2);
				glColorQt(c);

				glNormal3( fnormals[fit] );
				fvit = fvend = mesh->vertices(fit);
				do{ glVector3( points[fvit] ); } while (++fvit != fvend);
			}
			glEnd();

			glEnable(GL_LIGHTING);
			glDepthMask(GL_TRUE);
		}		

		if (drawIbsSample)
		{
			sampleRender.draw();
		}		
	}
}*/

void IBS::computeSampleWeightForTri()			// according to Xi's IBS paper
{
	Mesh mesh = ibsObj->getMesh();
	FaceProperty<double> fweight = mesh.getFaceProperty<double>("weight");
	// if (fweight.is_valid()) // TODO
	// {
	// 	return;
	// }	

	//SurfaceMeshHelper h(mesh);
	//Vector3VertexProperty points = h.getVector3VertexProperty(VPOINT);
	VertexProperty<Point3d> points = mesh.mesh3d->points();
	FaceProperty<double> farea = mesh.getFaceProperty<double>("area"); //h.computeFaceAreas();

	//mesh->update_face_normals();
	FaceProperty<Vector3d> fnormal = mesh.getFaceProperty<Vector3d>("normal");
	FaceProperty<double> fdist;
    fdist = mesh.mesh3d->add_property_map<Face,double>("f:dist").first;
	//FaceProperty fdist = mesh->add_face_property<Scalar>("dist");
	
	maxWeight = 0;
	totalWeight = 0;
	//fweight = mesh->face_property<Scalar>("f:weight", 0);
	BOOST_FOREACH(Face f_id, mesh.mesh3d->faces())
	{
		fweight[f_id] = 0;
		int idx = samplePairs[static_cast<int>(f_id)].second;
		Point3d s = obj2->getSamples()[idx].pos;

		Point3d center(0,0,0);
		std::vector<Vertex> faceVertices = mesh.getVerticesOfFace(f_id);
		if(faceVertices.size() == 3)
			center = CGAL::centroid(points[faceVertices[0]],points[faceVertices[1]],points[faceVertices[2]]);
		else
			center = CGAL::centroid(points[faceVertices[0]],points[faceVertices[1]],points[faceVertices[2]],points[faceVertices[3]]);
		/*for(Vertex v : faceVertices) {
			center = Point3d(center.x() + points[v].x(),center.y() + points[v].y(),center.z() + points[v].z());
		}
		center = Point3d(center.x()/3.0,center.y()/3.0,center.z()/3.0);*/

		Vector3d d = s - center;
		Vector3d n = fnormal[f_id];

		double dist = std::sqrt(d.squared_length());
		double angle = std::acos((d * n) / dist);	

		if ( angle >= PI / 2 )
		{
			fdist[f_id] = -dist;
			dist = 0;
			angle = PI - angle;
		}
		else
		{
			fdist[f_id] = dist;
		}
		
		//Eigen::AlignedBox3d bbox = scene->objects[objIdx1]->bbox;
		//bbox.extend(scene->objects[objIdx2]->bbox);
		//double distThreshold = bbox.diagonal().norm() * 0.5;

		// IsoCub3d bboxCuboid = IsoCub3d(CGAL::bbox_3(std::begin(objects),std::end(objects)));

		double distThreshold = std::sqrt((bboxCuboid.max()-bboxCuboid.min()).squared_length()) * 0.5;
		double disWeight = (dist <= distThreshold)? pow(1 - (dist / distThreshold), 20) : 0;
			
		double angleThreshold = PI / 3; // 60 deg
		double angleWeight = (angle <= angleThreshold)? (1 - angle / angleThreshold) : 0 ;

		double areaWeight = farea[f_id];

		fweight[f_id] = disWeight * angleWeight * areaWeight;

		totalWeight += fweight[f_id];
		if (fweight[f_id] > maxWeight)
		{
			maxWeight = fweight[f_id];
		}
	}

	//for (auto f:mesh->faces())
	//{
	//	fweight[f] /= totalWeight;
	//}
	//maxWeight /= totalWeight;
}

void IBS::sampling( int num )
{
	if (samples.size() == num)
	{
		return;
	}

	computeSampleWeightForTri();

	Sampler s(ibsObj->getMesh(), RANDOM_BARYCENTRIC_WEIGHTED);
	samples = s.getSamples(num);
	
	/*sampleRender.clear();
	for (auto sample : samples)
	{
		//sampleRender.addPointNormal(sample.pos, sample.n, QColor(255, 255, 0));
		sampleRender.addPoint(sample.pos, QColor(255, 255, 0));
	}*/
}

void IBS::computeGeomFeatures()
{
	computePFH();
	computeDirHist();
	computeDistHist();
}

void IBS::computeTopoFeatures()
{
	// if (scene->distPara.useTopo)
	// {
		computeBettiNumbers();
	// }
	
}

void IBS::computePFH()
{
	// sample points (using importance sampling)
	int sampleNum = 1000 * sampleRatio;
	sampling(sampleNum);

	// compute PFH, make sure normal points to interacting object
	std::vector<std::vector<double>> hist;
	for (int i = 0; i < samples.size(); ++i)
		hist.push_back(computePfhForSample(i, pointToCentralObject));

	std::vector<double> mean(hist[0].size(), 0);
	for(int i=0; i<mean.size(); i++)
	{
		for (auto h:hist)
		{
			mean[i] += h[i];
		}

		mean[i] /= hist.size();
	}

	std::vector<double> deviation(hist[0].size(), 0);		// standard deviation of the histogram
	for(int i=0; i<deviation.size(); i++)
	{
		for (auto h:hist)
		{
			deviation[i] += pow(h[i] - mean[i], 2);
		}

		deviation[i] /= hist.size();
		deviation[i] = sqrt(deviation[i]);
	}

	pfh.clear();
	//pfh.push_back(mean);
	pfh.insert(pfh.end(),mean.begin(),mean.end());
	pfh.insert(pfh.end(),deviation.begin(),deviation.end());
	// pfh.push_back(deviation);
}

void IBS::computeDirHist()
{
	int sampleNum = 5000 * sampleRatio;
	sampling(sampleNum);

	int bNum = 10;
	std::vector<double> h(bNum, 0);
	for (auto s:samples)
	{
		Vector3d n = s.n;
		if (pointToCentralObject)
		{
			n = -n;
		}

		double angle = acos(upright * n);

		int bIdx = angle / PI * 10;
		bIdx = (bIdx > 9)? 9:bIdx;

		h[bIdx]++;
	}

	for (auto& v:h)
	{
		v /= samples.size();
	}	

	dirHist = h;
}

void IBS::computeDistHist()
{
	int sampleNum = 5000 * sampleRatio;
	sampling(sampleNum);

	int bNum = 10;
	distHist.resize(bNum);
	for (int i=0; i<bNum; i++)
	{
		distHist[i] = 0;
	}

	/*Eigen::AlignedBox3d bbox = obj1->bbox;
	bbox.extend(obj2->bbox);*/

	std::vector<Object> objs({*obj1,*obj2});
	IsoCub3d bboxCuboid = IsoCub3d(CGAL::bbox_3(std::begin(objs),std::end(objs)));
	double th = std::sqrt((bboxCuboid.max()-bboxCuboid.min()).squared_length()) / 8.0;

	for (auto s:samples)
	{
		Point3d site = obj1->getSamples()[samplePairs[s.findex].first].pos;
		double d = std::sqrt((s.pos - site).squared_length());

		int bIdx = (int) (d / th * 10);
		bIdx = bIdx > 9? 9 : bIdx;

		distHist[bIdx]++;
	}

	for (auto& v:distHist)
	{
		v /= samples.size();
	}

}

std::vector<double> IBS::computePfhForSample( int sIdx, bool reverseNormal )
{
	int bNum = 125;
	std::vector<double> samplePFH(bNum, 0);
	Vector3d n1 = samples[sIdx].n;
	if (reverseNormal)
	{
		n1 = -n1;
	}
	for (int i=0; i<samples.size(); i++)
	{
		if (i != sIdx)
		{
			Vector3d n2 =  samples[i].n;
			if (reverseNormal)
			{
				n2 = -n2;
			}
			Vector3d p1p2 = samples[i].pos - samples[sIdx].pos;
			Vector3d v = CGAL::cross_product(p1p2,n1);
			v = v / std::sqrt(v.squared_length());
			Vector3d w = CGAL::cross_product(n1,v);
			w = w / std::sqrt(w.squared_length());
			Vector3d projected_n2 = Vector3d(w * n2, n1 * n2, 0);

			double phi = acos(n1 * p1p2) / std::sqrt(p1p2.squared_length());
			double alpha = acos(n2 * v);

			Vector3d local_e2(0,1,0);
			double theta = acos(local_e2 * projected_n2)/ std::sqrt(projected_n2.squared_length());
			double cross = local_e2[0]*projected_n2[1] - local_e2[1]*projected_n2[0];
			if (cross < 0)
			{
				theta = 2*PI - theta;
			}

			double bWidth1 = PI / 5.0;
			double bWidth2 = 2 * bWidth1; 

			int phiIdx = phi / bWidth1;
			int alphaIdx = alpha / bWidth1;			// alpha \in [0, \pi]
			int thetaIdx = theta / bWidth2;			// theta \in [0, 2*\pi]

			// Each bin represents 1 combination of ranges for (alpha,theta,phi)
			int bIdx = alphaIdx * 25 + thetaIdx * 5 + phiIdx;

			samplePFH[bIdx]++;
		}
	}

	for (auto& h:samplePFH)			// normalized to [0, 1]
	{
		h /= (samples.size() - 1);
	}

	return samplePFH;
}


void IBS::computeBettiNumbers()
{
	DebugTimer t;
	t.start();

	int positive, negative, positiveTri, negativeTri;
	positive = negative = positiveTri = negativeTri = 0;

	std::vector< std::vector<Edge> > complexEdgeIdx;
	std::vector<int> complexOpenEdgeNumber;
	std::shared_ptr<Mesh3d> mesh3d = ibsObj->getMesh().mesh3d;
	EdgeProperty<int> edgeComplexIdx = mesh3d->add_property_map<Edge,int>("e:complexIdx", -1).first;

	//CGAL::Halfedge_around_face_iterator<Mesh3d> feit, feend;
	BOOST_FOREACH(Face f_id, mesh3d->faces())
	{
		std::vector<Edge> consistingEdgeIdx;

		// find the complex idx for each edge
		//Mesh3d::Halfedge_range range = mesh3d->halfedges(f_id);
		//boost::tie(feit, feend) = CGAL::halfedges_around_face(mesh3d.halfedge(f_id),mesh3d);
		CGAL::Halfedge_around_face_circulator<Mesh3d> feit(mesh3d->halfedge(f_id),*mesh3d), feend(feit);
		do 
		{
			Edge edge = mesh3d->edge(*feit);
			consistingEdgeIdx.push_back(edge);			

			if (edgeComplexIdx[edge] != -1) 
			{
				complexOpenEdgeNumber[edgeComplexIdx[edge]]--;
			}
			else// if the edge haven't been checked, find the complex it belongs to
			{	
				int complexIdx = -1;

				// if any of the end point has been used in previous checked edges, copy the corresponding complexID
				std::vector<int> vComplexIdx(2, -1);
				for (int vIdx=0; vIdx<2; vIdx++)
				{
					Vertex v = mesh3d->vertex(edge, vIdx);

					CGAL::Halfedge_around_source_circulator<Mesh3d> vhit(mesh3d->halfedge(v),*mesh3d), vhend(vhit);
					//vhit = vhend = mesh->halfedges(v);
					do 
					{
						if (edgeComplexIdx[mesh3d->edge(*vhit)] != -1)
						{
							vComplexIdx[vIdx] = edgeComplexIdx[mesh3d->edge(*vhit)];
							complexIdx = vComplexIdx[vIdx];
							break;
						}
					} while (++vhit != vhend);
				}

				
				if (complexIdx != -1)  // find connected complex
				{
					// add to one valid complex
					edgeComplexIdx[edge] = complexIdx;
					complexEdgeIdx[complexIdx].push_back(edge);
					complexOpenEdgeNumber[complexIdx]++;					

					if (vComplexIdx[0] == vComplexIdx[1]) // if two end points correspond to same complex 
					{
						positive++;
					}
					else// if two end points correspond to two different complexes
					{
						// if both two complexes are valid, merge them
						if (vComplexIdx[0]!=-1 && vComplexIdx[1]!=-1)
						{
							for (auto e:complexEdgeIdx[vComplexIdx[1]])
							{
								edgeComplexIdx[e] = vComplexIdx[0];
							}
							auto *cei0 = &complexEdgeIdx[vComplexIdx[0]];
							auto *cei1 = &complexEdgeIdx[vComplexIdx[1]];
							cei0->insert(cei0->end(),cei1->begin(),cei1->end());
							//complexEdgeIdx[vComplexIdx[0]] << complexEdgeIdx[vComplexIdx[1]];
							complexOpenEdgeNumber[vComplexIdx[0]] += complexOpenEdgeNumber[vComplexIdx[1]];

							complexEdgeIdx[vComplexIdx[1]].clear();
							complexOpenEdgeNumber[vComplexIdx[1]] = -1;
						}

						negative++;
					}
				}
				else // create a new complex
				{
					std::vector<Edge> newComplex;
					newComplex.push_back(edge);
					edgeComplexIdx[edge] = complexEdgeIdx.size();
					complexEdgeIdx.push_back(newComplex);
					complexOpenEdgeNumber.push_back(1);

					negative++;
				}
			}		

		} while (++feit != feend);	

		// If all three edges belong to the same complex and the complex has no open edge
		std::vector<int> triComplexIdx;
		triComplexIdx.push_back(edgeComplexIdx[consistingEdgeIdx[0]]);
		triComplexIdx.push_back(edgeComplexIdx[consistingEdgeIdx[1]]);
		triComplexIdx.push_back(edgeComplexIdx[consistingEdgeIdx[2]]);
		if (triComplexIdx[0] == triComplexIdx[1] && triComplexIdx[0] == triComplexIdx[2] && complexOpenEdgeNumber[triComplexIdx[0]] == 0)
		{
			positiveTri++;
		}			
		else 
		{
			negativeTri++;
		}	
	}	

	bettiNumbers.clear();
	bettiNumbers.push_back(mesh3d->number_of_vertices() - 1 - negative);
	bettiNumbers.push_back(positive-negativeTri);
	bettiNumbers.push_back(positiveTri);

	// there might be small holes, set a threshold for computing b2
	int eThreshold = 10;

	int complexNum = 1;
	for( int i=0; i<complexOpenEdgeNumber.size(); i++ )
	{
		if( complexOpenEdgeNumber[i] >= 0 ) 
		{
			std::ostringstream ss;
			ss << "Complex " << complexNum++ << ": have " << complexOpenEdgeNumber[i] << "open edges";
			DebugLogger::log(ss);

			if (complexOpenEdgeNumber[i]>0 && complexOpenEdgeNumber[i] <= eThreshold)
			{
				bettiNumbers[2]++;
			}
		}
	}

	std::ostringstream ss;
	ss << "New code --- Time elapsed: " << t.getElapsedTime() << " ms";
	DebugLogger::log(ss);

	//ignoreSmallHoles();
}

// void IBS::ignoreSmallHoles()
// {
// 	Surface_mesh::Halfedge_property<bool> hvisisted = mesh->halfedge_property<bool>("h:visisted", false);

// 	std::vector< std::vector<Halfedge> > holes;
// 	for (auto h:mesh->halfedges())
// 	{
// 		if( !mesh->is_boundary(h) || hvisisted[h] ) continue;

// 		std::vector<Halfedge> hole;

// 		Halfedge hinit = h;
// 		h = mesh->next_halfedge(h);

// 		while( h != hinit ) {
// 			hole.push_back( h );
// 			hvisisted[h] = true;
// 			h = mesh->next_halfedge(h);
// 		}
// 		hole.push_back( h );

// 		holes.push_back(hole);
// 	}

// 	if (!(holes.size()==0 || holes.size() == bettiNumbers[1] + 1))
// 	{
// 		debugBox("Betti number computation error! holeNum = " + QString::number(holes.size()) + " vs Betti[1] = " + QString::number(bettiNumbers[1]));
// 	}
// 	else
// 	{
// 		int eThreshold = 10;
// 		for ( auto h: holes)
// 		{
// 			if ( h.size() < eThreshold)
// 			{
// 				bettiNumbers[1]--;
// 			}
// 		}
// 	}
// }

// // transfer original copy from Xi, extremely slow
// void IBS::computeBettiNumbers2() 
// {
// 	QTime t;
// 	t.start();

// 	int positive, negative, positiveTri, negativeTri;
// 	positive = negative = positiveTri = negativeTri = 0;

// 	std::vector< std::vector<int> > Edges; // Unique edges (v1, v2, complex id, next edge in complex, number of triangles using this edge, edge idx)
// 	std::vector<int> Complexes; // Complexes created by edges, the index of latest edge added into the complex
// 	std::vector<int> ComplexOpenEdges; // Open edges for each complex, number of open edges in that complex
// 	int TrigIDs[6] = {0,1,0,2,1,2}; // Vertex ID combinations, the i-th edge has two end points with idx TrigIDs[2*i] and TrigIDs[2*i+1]

// 	std::vector<int> tmpEdge; // Temporal edge vector
// 	tmpEdge << -1 << -1 << -1 << -1 << 1 << 0;
// 	int TriangleEdges[3]; // the 3 edge idx for this triangle
// 	int tmpComplexID[2]; // Complex IDs for 2 vertices building up an edge

// 	Surface_mesh::Vertex_around_face_circulator fvit, fvend;
// 	for(auto f:mesh->faces())
// 	{
// 		std::vector<int> vIdx;
// 		fvit = fvend = mesh->vertices(f);
// 		do{ vIdx.push_back(Vertex(fvit).idx()); } while (++fvit != fvend);

// 		for(int j=0; j<3; j++)
// 		{
// 			bool UniqueEdge = true;
// 			tmpComplexID[0] = tmpComplexID[1] = -1;
// 			for(int k=0; k < Edges.size(); k++)
// 			{
// 				// if any of the end point has been used in previous edges, copy the corresponding complexID
// 				if(Edges[k][0]==vIdx[TrigIDs[j*2]] || Edges[k][1]==vIdx[TrigIDs[j*2]]) 
// 				{
// 					tmpComplexID[0] = Edges[k][2];
// 				}
// 				if(Edges[k][0]==vIdx[TrigIDs[j*2+1]] || Edges[k][1]==vIdx[TrigIDs[j*2+1]])
// 				{
// 					tmpComplexID[1] = Edges[k][2];
// 				}

// 				// if the edge already exists, update the edge information
// 				if((Edges[k][0]==vIdx[TrigIDs[j*2]] && Edges[k][1]==vIdx[TrigIDs[j*2+1]]) ||
// 					(Edges[k][1]==vIdx[TrigIDs[j*2]] && Edges[k][0]==vIdx[TrigIDs[j*2+1]]))
// 				{
// 					UniqueEdge = false;
// 					TriangleEdges[j] = k;				

// 					// if this edge is shared by two triangles, then it will not be an open edge. The number of open edges in that complex --
// 					Edges[k][4]++;
// 					if(Edges[k][4]==2) 
// 					{
// 						ComplexOpenEdges[Edges[k][2]]--;
// 					}					
// 					break;
// 				}
// 			}
// 			if(UniqueEdge) // if this edge is new, add it to the Edges
// 			{
// 				// correspond this edge to existing complex if found any
// 				if(tmpComplexID[0]==-1)
// 				{
// 					tmpEdge[2] = tmpComplexID[1];
// 				}
// 				if(tmpComplexID[1]==-1)
// 				{
// 					tmpEdge[2] = tmpComplexID[0];
// 				}

// 				//  Add edge to a complex
// 				if(tmpEdge[2]>=0)   // if the complex does exist
// 				{
// 					// if two end points correspond to same complex 
// 					if(tmpComplexID[0] == tmpComplexID[1])
// 					{
// 						tmpEdge[2] = tmpComplexID[0];
// 						tmpEdge[3] = Complexes[tmpComplexID[0]];
// 						Complexes[tmpComplexID[0]] = Edges.size();
// 						ComplexOpenEdges[tmpEdge[2]]++;
// 						positive++;
// 					}
// 					else  // if two end points correspond to two different complexes
// 					{						
// 						if(tmpComplexID[0]==-1)  // Assign to complex 1
// 						{
// 							tmpEdge[2] = tmpComplexID[1];
// 							tmpEdge[3] = Complexes[tmpComplexID[1]];
// 							Complexes[tmpComplexID[1]] = Edges.size();
// 							ComplexOpenEdges[tmpComplexID[1]]++;
// 						}
// 						else if(tmpComplexID[1]==-1)  // Assign to complex 0
// 						{
// 							tmpEdge[2] = tmpComplexID[0];
// 							tmpEdge[3] = Complexes[tmpComplexID[0]];
// 							Complexes[tmpComplexID[0]] = Edges.size();
// 							ComplexOpenEdges[tmpComplexID[0]]++;
// 						}
// 						else  // merge complex 0 and complex 1: move edges from complex 1 to complex 0
// 						{
// 							// chase edges using Edges[k][3] , update the complex id
// 							int FirstEdgeOfComplex = Complexes[tmpComplexID[1]];
// 							int k = FirstEdgeOfComplex;
// 							while(k>=0)
// 							{
// 								Edges[k][2] = tmpComplexID[0];
// 								FirstEdgeOfComplex=k;
// 								k = Edges[k][3];
// 							}

// 							// connect those two complexes and transfer informations
// 							Edges[FirstEdgeOfComplex][3] = Complexes[tmpComplexID[0]];
// 							Complexes[tmpComplexID[0]] = Complexes[tmpComplexID[1]];
// 							Complexes[tmpComplexID[1]] = -1;
// 							ComplexOpenEdges[tmpComplexID[0]] += ComplexOpenEdges[tmpComplexID[1]];
// 							ComplexOpenEdges[tmpComplexID[1]] = -1;

// 							// add the new edge
// 							tmpEdge[2] = tmpComplexID[0];
// 							tmpEdge[3] = Complexes[tmpComplexID[0]];
// 							Complexes[tmpComplexID[0]] = Edges.size();
// 							ComplexOpenEdges[tmpComplexID[0]]++;
// 						}
// 						negative++;
// 					}
// 				}
// 				else  //if there is no such complex
// 				{
// 					// Add a new complex
// 					tmpEdge[2] = Complexes.size();
// 					tmpEdge[3] = -1;
// 					Complexes.push_back(Edges.size());
// 					ComplexOpenEdges.push_back(1);
// 					negative++;
// 				}
// 				TriangleEdges[j] = Edges.size();

// 				tmpEdge[0] = vIdx[TrigIDs[j*2]];
// 				tmpEdge[1] = vIdx[TrigIDs[j*2+1]];
// 				tmpEdge[5] = Edges.size();
// 				Edges.push_back(tmpEdge);
// 			}
// 		}

// 		// If all three edges belong to the same complex
// 		if(Edges[TriangleEdges[0]][2]==Edges[TriangleEdges[1]][2] && Edges[TriangleEdges[1]][2]==Edges[TriangleEdges[2]][2])
// 		{
// 			if(ComplexOpenEdges[Edges[TriangleEdges[2]][2]]==0) // if the complex has no open edges
// 			{
// 				positiveTri++;
// 			}
// 			else // check whether there is a same triangle has been visited 
// 			{
// 				bool SameTriangle = false;
// 				for(int j=0; j<f.idx()-1; j++)
// 				{
// 					std::vector<int> vIdx2;
// 					fvit = fvend = mesh->vertices(Face(j));
// 					do{ vIdx2.push_back(Vertex(fvit).idx()); } while (++fvit != fvend);

// 					if((vIdx[0]==vIdx2[0] && vIdx[1]==vIdx2[1] && vIdx[2]==vIdx2[2]) ||
// 						(vIdx[0]==vIdx2[0] && vIdx[1]==vIdx2[2] && vIdx[2]==vIdx2[1]) ||
// 						(vIdx[0]==vIdx2[1] && vIdx[1]==vIdx2[0] && vIdx[2]==vIdx2[2]) ||
// 						(vIdx[0]==vIdx2[1] && vIdx[1]==vIdx2[2] && vIdx[2]==vIdx2[0]) ||
// 						(vIdx[0]==vIdx2[2] && vIdx[1]==vIdx2[1] && vIdx[2]==vIdx2[0]) ||
// 						(vIdx[0]==vIdx2[2] && vIdx[1]==vIdx2[0] && vIdx[2]==vIdx2[1]))
// 					{
// 						SameTriangle=true;
// 						break;
// 					}
// 				}
// 				if(SameTriangle)
// 				{
// 					positiveTri++;
// 				}
// 				else
// 				{
// 					negativeTri++;
// 				}
// 			}
// 		}
// 		else
// 		{
// 			negativeTri++;
// 		}
// 	}	

// 	bettiNumbers.clear();
// 	bettiNumbers.push_back(mesh->vertices_size() - 1 - negative);
// 	bettiNumbers.push_back(positive-negativeTri);
// 	bettiNumbers.push_back(positiveTri);

// 	qDebug() << "Triangles: " << mesh->faces_size() << "    Edges: " << mesh->edges_size() << "    Vertices: " << mesh->vertices_size();    
// 	qDebug() << "Open edges:";
// 	int obj=1;
// 	for( int i=0; i<(int)ComplexOpenEdges.size(); i++ )
// 	{
// 		if( ComplexOpenEdges[i]>=0 ) 
// 		{
// 			qDebug() << "Complex " << obj++ << ": " << ComplexOpenEdges[i];
// 		}
// 	}

// 	qDebug("Old code --- Time elapsed: %d ms", t.elapsed());
// }

//////////////////////////////////////////////////////////////////////////
// implementation for community features

IBS::IBS(std::vector<std::shared_ptr<IBS>> ibsSet, std::vector<bool> reverseNormal)
{
	objects = ibsSet[0]->objects;
	ibsObj = NULL;
	sampleRatio = 1.0;
	maxWeight = 0;
	totalWeight = 0;

	// make sure that the features of each consisting IBS have been computed
	for (auto ibs : ibsSet)
	{
		if (ibs->dirHist.empty())
		{
			ibs->computeGeomFeatures();
			ibs->computeTopoFeatures();
		}
	}

	if (ibsSet.size() == 1)
	{
		pfh = ibsSet[0]->pfh;
		dirHist = ibsSet[0]->dirHist;
		distHist = ibsSet[0]->distHist;
		bettiNumbers = ibsSet[0]->bettiNumbers;
	}
	else
	{
		// compute combined features
		pfh = combinedPFH(ibsSet, reverseNormal);
		dirHist = combinedDirHist(ibsSet, reverseNormal);
		distHist = combinedDistHist(ibsSet);	
		bettiNumbers = combinedBettiNumber(ibsSet);
	}
}

std::vector<double> IBS::combinedPFH(std::vector<std::shared_ptr<IBS>> ibsSet, std::vector<bool> reverseNormal)
{
	// sampling weight
	double total = 0;
	for (auto ibs : ibsSet)
	{
		total += ibs->totalWeight;
	}

	// sample points
	for (auto ibs : ibsSet)
	{
		int num = ibsSet.size() * 1000 * ibs->totalWeight / total;
		ibs->sampling( num );
	}

	// compute distribution for each point, make sure normal points to the right direction
	std::vector< std::vector<double> > hist;
	for (int ibsIdx=0; ibsIdx < ibsSet.size(); ibsIdx++)
	{
		for (int i=0; i<ibsSet[ibsIdx]->samples.size(); i++)
		{
			hist.push_back(computePfhForSample(ibsIdx, i,  ibsSet, reverseNormal));
		}
	}

	// compute PFH	
	std::vector<double> mean(hist[0].size(), 0);
	for(int i=0; i<mean.size(); i++)
	{
		for (auto h:hist)
		{
			mean[i] += h[i];
		}

		mean[i] /= hist.size();
	}

	std::vector<double> deviation(hist[0].size(), 0);
	for(int i=0; i<deviation.size(); i++)
	{
		for (auto h:hist)
		{
			deviation[i] += pow(h[i] - mean[i], 2);
		}

		deviation[i] /= hist.size();
		deviation[i] = sqrt(deviation[i]);
	}

	std::vector<double> combinedPFH = mean;
	// combinedPFH.push_back(mean);
	combinedPFH.insert(combinedPFH.end(),deviation.begin(),deviation.end());
	// combinedPFH.push_back(deviation);	

	return combinedPFH;
}

std::vector<double> IBS::computePfhForSample(int ibsIdx, int sIdx, std::vector<std::shared_ptr<IBS>> ibsSet, std::vector<bool> reverseNormal)
{
	int bNum = 125;
	std::vector<double> samplePFH(bNum, 0);

	Vector3d n1 = ibsSet[ibsIdx]->samples[sIdx].n;
	if (reverseNormal[ibsIdx])
	{
		n1 = -n1;
	}

	int total = 0;
	for (int k=0; k<ibsSet.size(); k++)
	{
		std::vector<SamplePoint> ibsSamples = ibsSet[k]->samples;
		total += ibsSamples.size();
		for (int i=0; i<ibsSamples.size(); i++)
		{
			if (i != sIdx || k != ibsIdx)
			{
				Vector3d n2 =  ibsSamples[i].n;
				if (reverseNormal[k])
				{
					n2 = -n2;
				}
				Vector3d p1p2 = ibsSamples[i].pos - ibsSet[ibsIdx]->samples[sIdx].pos;

				Vector3d v = CGAL::cross_product(p1p2,n1);
				v = v / std::sqrt(v.squared_length());
				Vector3d w = CGAL::cross_product(n1,v);
				w = w / std::sqrt(w.squared_length());
				Vector3d projected_n2 = Vector3d(w * n2, n1 * n2, 0);

				double phi = acos(n1 * p1p2) / std::sqrt(p1p2.squared_length());
				double alpha = acos(n2 * v);

				Vector3d local_e2(0,1,0);
				double theta = acos(local_e2 * projected_n2)/ std::sqrt(projected_n2.squared_length());
				double cross = local_e2[0]*projected_n2[1] - local_e2[1]*projected_n2[0];
				if (cross < 0)
				{
					theta = 2*PI - theta;
				}

				double bWidth1 = PI / 5.0;
				double bWidth2 = 2 * bWidth1; 

				int phiIdx = phi / bWidth1;
				int alphaIdx = alpha / bWidth1;
				int thetaIdx = theta / bWidth2;

				int bIdx = alphaIdx * 25 + thetaIdx * 5 + phiIdx;

				samplePFH[bIdx]++;
			}
		}

	}
	
	for (auto& h:samplePFH)
	{
		h /= (total- 1);
	}

	return samplePFH;

}

std::vector<double> IBS::combinedDirHist(std::vector<std::shared_ptr<IBS>> ibsSet, std::vector<bool> reverseNormal)
{
	int n = ibsSet[0]->dirHist.size();
	std::vector<double> combinedDirHist(n, 0);

	for (int i=0; i<ibsSet.size(); i++)
	{
		std::shared_ptr<IBS> ibs = ibsSet[i];
		for (int j=0; j<n; j++)
		{
			if (reverseNormal[i])
			{
				combinedDirHist[j] += ibs->dirHist[n-1-j];
			}
			else
			{
				combinedDirHist[j] += ibs->dirHist[j];
			}
		}
	}

	for (int i = 0; i < combinedDirHist.size(); i++)
	{
		combinedDirHist[i] /= ibsSet.size();
	}

	return combinedDirHist;
}

std::vector<double> IBS::combinedDistHist(std::vector<std::shared_ptr<IBS>> ibsSet)
{
	std::vector<double> combinedDistHist(ibsSet[0]->dirHist.size(), 0);

	for (auto ibs : ibsSet)
	{
		for (int i=0; i<combinedDistHist.size(); i++)
		{
			combinedDistHist[i] += ibs->dirHist[i];
		}
	}

	for (int i = 0; i < combinedDistHist.size(); i++)
	{
		combinedDistHist[i] /= ibsSet.size();
	}

	return combinedDistHist;
}

std::vector<int> IBS::combinedBettiNumber(std::vector<std::shared_ptr<IBS>> ibsSet)
{
	std::vector<int> combinedBettiNumber(ibsSet[0]->bettiNumbers.size(), 0);
	return combinedBettiNumber;
}
