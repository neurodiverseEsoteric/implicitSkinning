#include "Table.h"

void meshTableFactory::meshInit(MItMeshVertex & vertexIter){
	_mTable->pointPosTable.reserve(_numElems * vertexIter.count());

	unsigned int offset = 0;
	for ( /* nothing */ ; !vertexIter.isDone(); vertexIter.next()) {
		// push point idx
		_mTable->pointIdxTable.push_back(vertexIter.index());
		
		// push point positions
		MPoint pos = vertexIter.position();
		for(unsigned int i = 0; i < 3; i++){
			_mTable->pointPosTable.push_back(pos[i]);
		}
		_mTable->pointPosTable.push_back(0.0f);

		// push adj points
		MIntArray edges;
		vertexIter.getConnectedEdges(edges);
		std::size_t numEdges = edges.length();
		_mTable->offsetTable.push_back(offset);

		for (unsigned int i = 0; i < numEdges; i++)
		{
			int oppVert = -1;
			vertexIter.getOppositeVertex(oppVert, edges[i]);
			_mTable->adjPtIdxTable.push_back(oppVert);
		}
		offset += numEdges;
	}
}

template<class computeController>
void meshTableFactory::processWeights(){
	// reorder joints' weights for each vertex


	// regroup/reorder the vertex index table by vertex' joint index
	// assign value to ptJointIdxTable

}

template<class computeController>
void jointsTableFactory::addJointTable(MItMeshPolygon & faceIter, std::vector<unsigned int> & faceIdxs, std::vector<double> & faceAreas, computeController & controller){
	// TODO sort the area-face_index pair by area

	// loop through all the triangles and get cdf 

	// inverse cdf 

	// convert to array 

	// random sampling points on triangles, total sample number = sample/perJointMesh  *  joints;

}