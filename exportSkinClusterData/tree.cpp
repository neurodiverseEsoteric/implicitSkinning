#include "tree.h"

MStatus binaryTree::traverse(std::shared_ptr<jointNode> root, nodeOperator& ope){
	if (root){
		if (ope(root) == MStatus::kFailure)
			return MStatus::kFailure;
	}
	if (root->sibling){	// handle sibling 
		if (traverse(root->sibling, ope) == MStatus::kFailure)
			return MStatus::kFailure;
	}
	if(root->child){	// handle child
		if (traverse(root->child, ope) == MStatus::kFailure)
			return MStatus::kFailure;
	}
	return MStatus::kSuccess;
}

bool binaryTree::iterativeInsert(std::shared_ptr<jointNode> root, std::shared_ptr<jointNode> node, std::string parentName){
	if (root->name == parentName){
		if (root->child != NULL){
			std::shared_ptr<jointNode> child = root->child;
			while(child->sibling != NULL)
				child = child->sibling;
			child->sibling = node;
		}else{
			root->child = node;
		}
		// multiply to get the global transform matrix
		node->matrix = node->matrix * root->matrix;
		return true;
	}
	if(root->child != NULL){	// handle child
		if (iterativeInsert(root->child, node, parentName))
			return true;
	}
	if (root->sibling != NULL){	// handle sibling 
		if (iterativeInsert(root->sibling, node, parentName))
			return true;
	}
	return false;
}

MStatus binaryTree::insertNode(std::shared_ptr<jointNode> node, std::string parentName){
	if (iterativeInsert(root, node, parentName))
		return MStatus::kSuccess;
	else
		return MStatus::kFailure;
}

MStatus setLocalCoordOp::operator()(std::shared_ptr<jointNode> node){
	using namespace Eigen;

	unsigned int m = 3;   // dimension of each point
	unsigned int n = node->points->length();  // number of points 
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

	return MStatus::kSuccess;
}