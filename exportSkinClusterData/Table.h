#if defined(_MSC_VER)
#pragma once
#endif

#ifndef TABLE_H
#define TABLE_H

#include <vector>
#include <memory>

#include <maya/MItMeshVertex.h>
#include <maya/MDoubleArray.h>
#include <maya/MItMeshPolygon.h>

#include "tree.h"
#include "computeController.h"
#include "Transform.h"

class meshTable{
public:
	meshTable(unsigned int numElems, unsigned int numJoints):
	  _numElems(numElems), pointPosTable(std::vector<float>()), pointIdxTable(std::vector<unsigned int>()), 
		  offsetTable(std::vector<unsigned int>()), adjPtIdxTable(std::vector<unsigned int>())
	{};

	std::vector<float>		  pointPosTable;	// dynamic data, pos( x, y, z ) + original field value (w)

	// static data
	std::vector<unsigned int> pointIdxTable;	// point index to pointPosTable table, all the table below will count on this table
	std::vector<unsigned int> offsetTable;		// point valence info offset in valence table
	std::vector<unsigned int> adjPtIdxTable;	// adj point idxs table
	std::vector<unsigned int> ptJointIdxTable;	// point' joint' index table

	int _numElems;								// point element size

};


class meshTableFactory{
public:
	meshTableFactory(unsigned int numJoints, unsigned int numElems = 4):_numJoints(numJoints), _numElems(numElems)
	{
		_mTable = std::make_shared<meshTable>(numElems, numJoints);
	}

	void meshInit(MItMeshVertex & vertexIter); 

	template<class computeController>
	void processWeights(computeController & controller);

	void addWeights(const MDoubleArray & wts){
		for(unsigned int j = 0; j < _numJoints; j++)
			_weightsTable.push_back(wts[j]);
	}

	inline std::shared_ptr<meshTable> getMeshTable() { return _mTable;}

	unsigned int _numJoints;
	unsigned int _numElems;
	std::vector<float> _weightsTable;		// joint weights table, which will be used to order the pointIdxTable.

private:
	std::shared_ptr<meshTable> _mTable;
};


class jointTable{
public:
	Matrix4x4 matrix;
	localCoord coord;
	std::vector<float> rbfPosParams;
	std::vector<float> rbfNormalParams;
	unsigned int jointIdx;
};


class jointsTableFactory{
public:
	jointsTableFactory(unsigned int numJoints){ _jTableList.reserve(numJoints); }

	template<class computeController>
	void addJointTable(MItMeshPolygon & faceIter, std::vector<unsigned int> & faceIdxs, std::vector<double> & faceAreas, computeController & controller);

	inline std::vector<jointTable *> getJointTable() { return _jTableList;}

private:
	std::vector<jointTable *> _jTableList;
};

#endif