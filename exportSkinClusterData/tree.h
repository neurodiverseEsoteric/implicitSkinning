#ifndef TREE_H
#define TREE_H

#define NOMINMAX

#include <vector>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <utility>
#include <list>

#include <maya/MPxTransform.h>
#include <maya/MFloatPointArray.h>
#include <maya/MVector.h>
#include <maya/MString.h>
#include <maya/MIOStream.h>
#include <maya/MUintArray.h>
#include <maya/MDagPath.h>
#include <maya/MTime.h>

// include Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <algorithm>
#include <math.h>


class localCoord{
public:
	localCoord():center(new MPoint()), axisX(new MVector()), axisY(new MVector()), axisZ(new MVector()), bbox(new MVector()){};
	
	MPoint center;
	MVector axisX;
	MVector axisY;
	std::shared_ptr<MVector> axisZ;
	std::shared_ptr<MVector> bbox;
};

class meshNode{
public:
	MStatus update();
// data
public:
	localCoord coord;
	std::vector<MVector> rbfPosParams;
	std::vector<MVector> rbfNormalParams;
	std::vector<unsigned int> vertexIdxList;
	std::shared_ptr<jointNode> joint;

private:
	std::vector<MPoint> samplePoints;
};

class jointNode{
public: // method
	jointNode(unsigned int i, MDagPath dagPath, MMatrix matrix = MMatrix::identity)
		:index(i), jointPath(dagPath), startMatrix(matrix), meshList(std::make_shared<meshNode>())
	{};

	MStatus update();

	// observe pattern
	void attach(std::shared_ptr<meshNode> mesh) { meshList.push_back(mesh); }     
	void remove(std::shared_ptr<meshNode> mesh) { meshList.remove(mesh); }      

	void Notify() 
	{  
		for(auto iter = meshList.begin(); iter != meshList.end(); iter++)  
			(*iter)->Update(); 
	} 

// data
public: 
	unsigned int index;		// joint index in skinCluster
	MDagPath jointPath;		// joint name
	MMatrix	startMatrix;	// joint transform matrix in global coord at start frame
	MMatrix	currMatrix;		// joint transform matrix in global coord at current frame
	std::shared_ptr<jointNode> child;
	std::shared_ptr<jointNode> sibling;

private:  
	std::list<std::shared_ptr<meshNode>> meshList; //meshes affected by this joint
};

class nodeOperator{
	MStatus operator()(std::shared_ptr<jointNode> node){};
};

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

class setLocalCoordOp : nodeOperator{
public:
	setLocalCoordOp(FILE * f):file(f){};
	MStatus operator()(std::shared_ptr<jointNode> node);
private:
	FILE * file;
};

class binaryTree {
public:	
	binaryTree():root(std::make_shared<jointNode>(-1, std::string("world"))){}

	~binaryTree(){}

	MStatus insertNode(std::shared_ptr<jointNode> node, std::string parentName);
	
	MStatus traverse(std::shared_ptr<jointNode> root, nodeOperator& ope);

	MStatus find(std::shared_ptr<jointNode> root, nodeOperator& ope, unsigned int & level, std::shared_ptr<jointNode> & result);

	bool iterativeInsert(std::shared_ptr<jointNode> root, std::shared_ptr<jointNode> node, std::string parentName);

	MStatus update();
private:
	std::shared_ptr<jointNode> root;
};



template <class nodeOperator>
MStatus binaryTree::find(std::shared_ptr<jointNode> root, nodeOperator& ope, unsigned int & level, std::shared_ptr<jointNode> & result){
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