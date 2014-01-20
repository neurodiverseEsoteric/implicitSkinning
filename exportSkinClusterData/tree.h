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
#include <maya/MItMeshVertex.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MDoubleArray.h>
#include <maya/MGlobal.h>
#include <MSelectionList.h>

// include Eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <algorithm>
#include <math.h>

#include <computeController.h>

#define CheckError(stat,msg)		\
	if ( MS::kSuccess != stat ) {	\
	displayError(msg);			\
	continue;					\
	}


class localCoord{
public:
	localCoord():center(MPoint()), axisX(MVector()), axisY(MVector()), 
		axisZ(MVector()), bbox(std::make_pair<MVector, MVector>(MVector(),MVector()))
	{};
	
	MPoint center;
	MVector axisX;
	MVector axisY;
	MVector axisZ;
	std::pair<MVector, MVector> bbox;
};

class jointNode;
class meshNode{
public:
	meshNode(MDagPath path):jointPath(path),rbfPosParams(std::vector<MVector>()), rbfNormalParams(std::vector<MVector>()), 
		vertexIdxList(std::vector<unsigned int>())
	{}
	MStatus update();	// get the position of vertices at current time
	bool calCoord();	// use the samplePoints to calculate the coord
	bool calRBFs();		// use the samplePoints to calculate the rbfPosParams and rbfNormalParams
// data
public:
	MDagPath jointPath;
	std::vector<MVector> rbfPosParams;
	std::vector<MVector> rbfNormalParams;
	std::vector<unsigned int> vertexIdxList;
	localCoord coord;
	std::shared_ptr<jointNode> joint;
private:
	MFloatPointArray samplePoints;
};

class skinNode{
public:
	skinNode(const MFnSkinCluster & s, std::shared_ptr<skinClusterNode> sc, 
			MDagPath spath = MDagPath(), unsigned int num = -1)
		:skc(s), scNode(sc), skinPath(spath), numJoints(num), 
		 weightsTable(std::vector<float>()), idxTable(std::vector<unsigned int>())
	{
		MItMeshVertex vertexIter(skinPath);
		unsigned int count = vertexIter.count();
		weightsTable.reserve(count * numJoints);
		neighborOffsetTable.reserve(count);
		idxTable.reserve(count);
	};

	MStatus addWeights();		
	MStatus addNeighbors();
	bool makeMeshNode();		// setup meshNode based on grouped vertex idx according to per vertex weights array
	bool processWeights(const computeController & controller);
	bool processSamples(const computeController & controller);
// data
public:
	MDagPath skinPath;
	std::vector<unsigned int> neighborOffsetTable;		// point valence info offset in valence table
	std::vector<unsigned int> neighborIdxTable;	// adj point idxs table
	std::shared_ptr<skinClusterNode> scNode;
private:
	const MFnSkinCluster & skc;
	std::list<std::shared_ptr<meshNode>> meshList;
	std::vector<float> weightsTable;					// vertex weights table
	std::vector<unsigned int> idxTable;					// vertex index in this mesh
	unsigned int numJoints;								
};

class jointNode{
public: // method
	jointNode(unsigned int i = -1, MDagPath dagPath = MDagPath(), MMatrix matrix = MMatrix::identity)
		:index(i), jointPath(dagPath), startMatrix(matrix), currMatrix()
	{};

	MStatus update();

	// observe pattern
	void attach(std::shared_ptr<meshNode> mesh) { meshList.push_back(mesh); }     
	void remove(std::shared_ptr<meshNode> mesh) { meshList.remove(mesh); }      

	void Notify() 
	{  
		for(std::list<std::shared_ptr<meshNode>>::iterator iter = meshList.begin(); iter != meshList.end(); iter++)  
			(*iter)->update(); 
	} 

// data
public: 
	unsigned int index;		// joint index in skinCluster
	MDagPath jointPath;		// joint name
	MMatrix startMatrix;	// joint transform matrix in global coord at start frame
	MMatrix	currMatrix;		// joint transform matrix in global coord at current frame
	std::shared_ptr<jointNode> child;
	std::shared_ptr<jointNode> sibling;

private:  
	std::list<std::shared_ptr<meshNode>> meshList; //meshes affected by this joint
};

// abstract class for operators
class nodeOperator{
public:
	MStatus operator()(std::shared_ptr<jointNode> node){};
};

class printOp;


class skinClusterNode{
public:	
	skinClusterNode():root(std::make_shared<jointNode>()){}
	MStatus insertNode(std::shared_ptr<jointNode> node, MDagPath parent);
	MStatus traverse(nodeOperator& ope);
	MStatus update();	// calculate the 

// data
private:
	MStatus iterativeInsert(std::shared_ptr<jointNode> root, std::shared_ptr<jointNode> node, MDagPath parent);
	MStatus skinClusterNode::iterativeTraverse(std::shared_ptr<jointNode> node, nodeOperator& ope);
	std::shared_ptr<jointNode> root;
};


// MVC' model part
class sceneList{
public:
	static std::shared_ptr<std::list<std::pair<std::shared_ptr<skinClusterNode>, std::shared_ptr<skinNode>>>> getInstance(){
		if (!scene){
			scene = std::make_shared<std::list<std::pair<std::shared_ptr<skinClusterNode>, std::shared_ptr<skinNode>>>>();
		}
		return scene;
	}

	static bool processWeights(const computeController & controller);
	static bool processSamples(const computeController & controller);
	static bool modifyMeshNodeGroup();			// modify through GUI
	static bool writeToBuffer();

private:
	static std::shared_ptr<std::list<std::pair<std::shared_ptr<skinClusterNode>, std::shared_ptr<skinNode>>>> scene;
};

