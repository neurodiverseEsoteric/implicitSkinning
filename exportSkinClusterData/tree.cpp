#include "tree.h"

bool meshNode::calCoord(){
	using namespace Eigen;

	unsigned int m = 3;   // dimension of each point
	unsigned int n = samplePoints.length();  // number of points 
	// matrix (n x m)
	MatrixXd DataPoints(n, m);  

	// fill the data
	std::shared_ptr<MFloatPointArray> pts = node->points;
	unsigned int nPoints = pts->length();
	for(unsigned int i = 0; i < nPoints; i++) {
		float* ptPtr = &(*pts)[i].x;
		for(unsigned int j=0; j<3; j++) {
			DataPoints(i,j) = ptPtr[j];
		}
	}

	// calculate mean value for each dimension
	double mean; RowVectorXd meanRow(m);
	for (int j = 0; j < DataPoints.cols(); j++){
		mean = (DataPoints.col(j).sum())/n;		//compute mean
		meanRow(j) = mean;
	}

	//   center each point with the mean value
	for (int i = 0; i < DataPoints.rows(); i++){
		DataPoints.row(i) -= meanRow;
	}

	// get the covariance matrix
	MatrixXd Covariance = MatrixXd::Zero(m, m);
	Covariance = (1 / (double) n) * DataPoints.transpose() * DataPoints;
	//  std::cout << Covariance ;	

	// compute the eigenvalue on the covariance Matrix
	EigenSolver<MatrixXd> m_solve(Covariance);
	VectorXd eigenvalues = VectorXd::Zero(m);
	eigenvalues = m_solve.eigenvalues().real();

	MatrixXd eigenVectors = MatrixXd::Zero(n, m);  // matrix (n x m) (points, dims)
	eigenVectors = m_solve.eigenvectors().real();	

	// sort and get the permutation indices
	typedef std::pair<double, int> valueIndex;
	typedef std::vector<valueIndex> PermutationIndices;	

	PermutationIndices eigenValueInOrder;
	for (unsigned int i = 0 ; i < m; i++)
		eigenValueInOrder.push_back(std::make_pair(eigenvalues(i), i));

	sort(eigenValueInOrder.begin(), eigenValueInOrder.end());

	// get the local axis x, y, z, with x as the length axis
	Vector3d axisX = eigenVectors.col(eigenValueInOrder[m-1].second);
	Vector3d axisY, axisZ;
	double y[3] = {0.0, 0.0, 0.0};double z[3] = {0.0, 0.0, 0.0};
	if (fabs(axisX.x()) > fabs(axisX.y())){
		double invLen = 1.0 / sqrt(axisX.x() * axisX.x() + axisX.z() * axisX.z());
		y[0] = - axisX.z() * invLen; y[1] = 0.0; y[2] = axisX.x() * invLen;
		axisY = Vector3d(y);
	}else {
		double invLen = 1.0 / sqrt(axisX.y() * axisX.y() + axisX.z() * axisX.z());
		y[0] = 0.0; y[1] = axisX.z() * invLen; y[2] = - axisX.y() * invLen;
		axisY = Vector3d(y);
	}
	axisZ = axisX.cross(axisY);

	// get the local coord' center
	Vector3d center = meanRow.transpose();

	// get the size of bbox
	MatrixXd rowFeatureVector = MatrixXd::Zero(m, m);
	for (unsigned int i = 0; i < m; i++){
		rowFeatureVector.row(i) = eigenVectors.col(eigenValueInOrder[m-1-i].second).transpose();
	}

	MatrixXd finalData = MatrixXd::Zero(m, n);  // matrix ( m * n ) 
	finalData = rowFeatureVector * DataPoints.transpose();

	Vector3d bbox;
	std::vector<double> vals;
	for (unsigned int j = 0; j < m; j++){
		for (unsigned int i = 0 ; i < n; i++)
			vals.push_back(finalData(j,i));
		sort(vals.begin(), vals.end());
		bbox(j) = std::max(fabs(vals[0]), fabs(vals[n-1]));
		vals.clear();
	}

	// set the tree node' coord member
	node->coord->axisX->x = axisX.x(); node->coord->axisX->y = axisX.y(); node->coord->axisX->z = axisX.z();
	node->coord->axisY->x = axisY.x(); node->coord->axisY->y = axisY.y(); node->coord->axisY->z = axisY.z();
	node->coord->axisZ->x = axisZ.x(); node->coord->axisZ->y = axisZ.y(); node->coord->axisZ->z = axisZ.z();

	// set the tree node's center
	node->coord->center->x = center.x(); node->coord->center->y = center.y(); node->coord->center->z = center.z(); 

	// set the tree node's bbox
	node->coord->bbox->x = bbox.x(); node->coord->bbox->y = bbox.y(); node->coord->bbox->z = bbox.z(); 

	return true;
}


bool meshNode::calRBFs(){
	
	return true;
}


MStatus skinNode::addWeights(){
	MItMeshVertex vertexIter(skinPath);
	MStatus stat;
	for ( /* nothing */ ; !vertexIter.isDone(); vertexIter.next()) {
		MObject vertex = vertexIter.currentItem(&stat);
		if (stat == MStatus::kFailure)
			return stat;

		MDoubleArray wts;
		unsigned int jointCount;
		stat = skc.getWeights(skinPath,vertex,wts,jointCount);
		if (stat == MStatus::kFailure)
			return stat;

		if (0 == jointCount) {
			return MS::kFailure;
		}

		// push the weights into meshTable factory
		for (unsigned int i = 0; i < numJoints; i++)
			weightsTable.push_back(wts[i]);
			idxTable.push_back(vertexIter.index());
	} // vertexIter 
	return MS::kSuccess;
}


MStatus skinNode::addNeighbors(){
	MItMeshVertex vertexIter(skinPath);
	int dummy;
	for (unsigned int i = 0; i < count; i++){
		vertexIter.setIndex(idxTable[i], dummy);

		MIntArray edges;
		vertexIter.getConnectedEdges(edges);
		unsigned int numEdges = edges.length();
		neighborOffsetTable.push_back(numEdges);

		for (unsigned int k = 0; k < numEdges; ++k)
		{
			int oppVert = -1;
			vertexIter.getOppositeVertex(oppVert, edges[k]);
			neighborIdxTable.push_back(oppVert);
		}
	}
}


bool skinNode::processWeights(const computeController & controller){
	// collect all the data needed to be ordered

	// use computeController to reorder the weights for each vertex and get the most affected joint idx

	// partition the vertices indexes into different groups, using the histogram algorithm or thrust histogram

	// make meshNodes list

}


bool skinNode::processSamples(const computeController & controller){
	// convert the vertex into faces for each meshNode 
	
	// calculate samples

	// calculate the local coord and RBF params
	
}


std::shared_ptr<std::list<std::pair<std::shared_ptr<skinClusterNode>, std::shared_ptr<skinNode>>>> sceneList::scene;


MStatus skinClusterNode::iterativeTraverse(std::shared_ptr<jointNode> node, nodeOperator& ope){
	if (node){
		if (ope(node) == MStatus::kFailure)
			return MStatus::kFailure;
	}
	if (node->sibling){	// handle sibling 
		if (iterativeTraverse(node->sibling, ope) == MStatus::kFailure)
			return MStatus::kFailure;
	}
	if(node->child){	// handle child
		if (iterativeTraverse(node->child, ope) == MStatus::kFailure)
			return MStatus::kFailure;
	}
	return MStatus::kSuccess;
}


MStatus skinClusterNode::traverse(nodeOperator& ope){
	if (iterativeTraverse(root, ope))
		return MStatus::kSuccess;
	else
		return MStatus::kFailure;
}


MStatus skinClusterNode::iterativeInsert(std::shared_ptr<jointNode> root, std::shared_ptr<jointNode> node, MDagPath parentPath){
	if (root->jointPath == parentPath){
		if (root->child != NULL){
			std::shared_ptr<jointNode> child = root->child;
			while(child->sibling != NULL)
				child = child->sibling;
			child->sibling = node;
		}else{
			root->child = node;
		}
		return MStatus::kSuccess;
	}
	if(root->child != NULL){	// handle child
		if (iterativeInsert(root->child, node, parentPath))
			return MStatus::kSuccess;
	}
	if (root->sibling != NULL){	// handle sibling 
		if (iterativeInsert(root->sibling, node, parentName))
			return MStatus::kSuccess;
	}
	return MStatus::kFailure;
}


MStatus skinClusterNode::insertNode(std::shared_ptr<jointNode> node, MDagPath parentPath){
	if (iterativeInsert(root, node, parentPath))
		return MStatus::kSuccess;
	else
		return MStatus::kFailure;
}


class printOp : nodeOperator{
public:
	printOp(FILE * f):file(f){};

	MStatus operator()(std::shared_ptr<jointNode> node){
		fprintf(file, "joint %s ' matrix: ", node->name.c_str());
		MMatrix mat = node->matrix;
		for (unsigned int r = 0; r < 4; r++)
		{
			for (unsigned int c = 0; c < 4; c++)
			{
				fprintf(file, " %f ", mat[r][c]);
			}
		}
		return MStatus::kSuccess;
	};
private:
	FILE * file;
};


class findLevelOp : nodeOperator{
public:
	findLevelOp(unsigned int idx):index(idx){};

	MStatus operator()(std::shared_ptr<jointNode> node){
		if (node->index == index)
			return MStatus::kSuccess;
		else 
			return MStatus::kFailure;
	};
private:
	unsigned int index;
};


class checkPointGrouplOp : nodeOperator{
public:
	checkPointGrouplOp(FILE * f):file(f){};

	MStatus operator()(std::shared_ptr<jointNode> node){
		fprintf(file, "joint %s' points include : ", node->name.c_str());
		std::shared_ptr<std::vector<unsigned int>> pts = node->points;
		std::size_t nPoints = pts->size();
		for(std::size_t i=0; i < nPoints; i++) {
			std::size_t ptIdx = (*pts)[i];
			//fprintf(file, " %d ", i);
			//for(int j=0; j<3; j++) {
			//	fprintf(file, " %f ", ptPtr[j]);
			//}
		}
		fprintf(file, "\\n");
		return MStatus::kSuccess;
	};
private:
	FILE * file;
};


bool sceneList::processWeights(const computeController & controller){
	for (std::list<std::pair<std::shared_ptr<skinClusterNode>, std::shared_ptr<skinNode>>>::iterator iter = scene->begin(); iter != scene->end(); iter++){
		iter->second->processWeights(controller);
	}	
}


bool sceneList::processSamples(const computeController & controller){
	for (std::list<std::pair<std::shared_ptr<skinClusterNode>, std::shared_ptr<skinNode>>>::iterator iter = scene->begin(); iter != scene->end(); iter++){
		iter->second->processSamples(controller);
	}
}


MStatus sceneList::nodeFromName(MString name, MObject & obj)
{
	MSelectionList tempList;
	tempList.add( name );
	if ( tempList.length() > 0 ) 
	{
		tempList.getDependNode( 0, obj );
		return MS::kSuccess;
	}
	return MS::kFailure;
}


double sceneList::readSceneStartFrame()
{
	MTime startFrame;

	// Get the render globals node
	int rangeIsSet = 0;
	MObject renderGlobNode;
	if (nodeFromName("defaultRenderGlobals", renderGlobNode) == MS::kSuccess)
	{
		MFnDependencyNode fnRenderGlobals( renderGlobNode );

		// Check if the renderGlobals is used for the frame range
		MPlug animPlug = fnRenderGlobals.findPlug( "animation" );
		short anim;
		animPlug.getValue( anim );

		if ( anim ) {
			float byFrame;
			fnRenderGlobals.findPlug("startFrame").getValue(startFrame);
			rangeIsSet = 1;
		}
	}
	if (!rangeIsSet)	{
		// USE_TIMESLIDER
		startFrame = MAnimControl::minTime();
	}
	return (int) startFrame.as( MTime::uiUnit() );
}