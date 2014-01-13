#ifndef TREE_H
#define TREE_H

#define NOMINMAX

#include <vector>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <utility>

#include <maya/MPxTransform.h>
#include <maya/MFloatPointArray.h>
#include <maya/MVector.h>
#include <maya/MString.h>
#include <maya/MIOStream.h>
#include <maya/MUintArray.h>

// include Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <algorithm>
#include <math.h>


class localCoord{
public:
	localCoord():center(new MPoint()), axisX(new MVector()), axisY(new MVector()), axisZ(new MVector()), bbox(new MVector()){};

	std::shared_ptr<MPoint>	center;
	std::shared_ptr<MVector> axisX;
	std::shared_ptr<MVector> axisY;
	std::shared_ptr<MVector> axisZ;
	std::shared_ptr<MVector> bbox;
};

class rbfParameters{
public:
	rbfParameters(unsigned int num = 50):samplesNumber(num){};

private:
	unsigned int samplesNumber;
};

class binaryTreeNode{
public:
	typedef  std::pair <unsigned int,double> indexedArea;

	binaryTreeNode(unsigned int i, std::string n, MMatrix m = MMatrix::identity, unsigned int num = 50)
		:index(i), name(n), matrix(m),
		meshIdxs(new std::vector<indexedArea>), points(new MFloatPointArray()), 
		coord(new localCoord()), rbfs(new rbfParameters(num)){};

	unsigned int index;
	std::string	name;
	MMatrix matrix;

	// member to store the tree structure, so do not need use new to initialize
	std::shared_ptr<binaryTreeNode> child;
	std::shared_ptr<binaryTreeNode> sibling;

	// member to store the points grouped and etc, need to be initialized
	
	std::shared_ptr<std::vector<indexedArea>> meshIdxs;
	std::shared_ptr<MFloatPointArray> points;
	std::shared_ptr<localCoord> coord;
	std::shared_ptr<rbfParameters> rbfs;

private:
	MStatus convertPointSetToSamples(MFloatPointArray & pa);
	std::shared_ptr<MFloatPointArray> samples;
};

class nodeOperator{
	MStatus operator()(std::shared_ptr<binaryTreeNode> node){};
};

class printOp : nodeOperator{
public:
	printOp(FILE * f):file(f){};

	MStatus operator()(std::shared_ptr<binaryTreeNode> node){
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

	MStatus operator()(std::shared_ptr<binaryTreeNode> node){
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

	MStatus operator()(std::shared_ptr<binaryTreeNode> node){
		fprintf(file, "joint %s' points include : ", node->name.c_str());
		std::shared_ptr<MFloatPointArray> pts = node->points;
		int nPoints = pts->length();
		for(int i=0; i<nPoints; i++) {
			float* ptPtr = &(*pts)[i].x;
			fprintf(file, " %d ", i);
			for(int j=0; j<3; j++) {
				fprintf(file, " %f ", ptPtr[j]);
			}
		}
		fprintf(file, "\\n");
		return MStatus::kSuccess;
	};
private:
	FILE * file;
};

class setLocalCoordOp : nodeOperator{
public:
	setLocalCoordOp(FILE * f):file(f){};
	MStatus operator()(std::shared_ptr<binaryTreeNode> node);
private:
	FILE * file;
};

class binaryTree {
public:	
	binaryTree():root(new binaryTreeNode(-1, std::string("world"))){
	}

	~binaryTree(){}

	MStatus insertNode(std::shared_ptr<binaryTreeNode> node, std::string parentName);
	
	template <class nodeOperator>
	MStatus traverse(std::shared_ptr<binaryTreeNode> root, nodeOperator& ope);

	template <class nodeOperator>
	MStatus find(std::shared_ptr<binaryTreeNode> root, nodeOperator& ope, unsigned int & level, std::shared_ptr<binaryTreeNode> & result);

	std::shared_ptr<binaryTreeNode> root;
private:
	bool iterativeInsert(std::shared_ptr<binaryTreeNode> root, std::shared_ptr<binaryTreeNode> node, std::string parentName);
};

template <class nodeOperator>
MStatus binaryTree::traverse(std::shared_ptr<binaryTreeNode> root, nodeOperator& ope){
	if (root){
		ope(root);
	}
	if (root->sibling){	// handle sibling 
		traverse(root->sibling, ope);
	}
	if(root->child){	// handle child
		traverse(root->child, ope);
	}
	return MStatus::kSuccess;
}

template <class nodeOperator>
MStatus binaryTree::find(std::shared_ptr<binaryTreeNode> root, nodeOperator& ope, unsigned int & level, std::shared_ptr<binaryTreeNode> & result){
	if (root){
		if (ope(root) == MStatus::kSuccess){
			result = root;
			return MStatus::kSuccess;
		}
	}
	if (root->sibling){	// handle sibling 
		if ( find(root->sibling, ope, level, result) == MStatus::kSuccess )
			return MStatus::kSuccess; 
	}
	if(root->child){	// handle child
		level++;
		if ( find(root->child, ope, level, result) == MStatus::kSuccess )
			return MStatus::kSuccess; 
		else 
			level--;
	}
	return MStatus::kFailure;
}

#endif /* MYHEADER_H */