#ifndef SCENEDATA_H
#define SCENEDATA_H

#include <list>

#include "common.h"
#include "meshData.h"
#include "jointData.h"

class sceneData { 
public: 
	typedef std::unique_ptr<jointData> jointPtr;
	typedef std::unique_ptr<meshData> meshPtr;

public:
	static sceneData *getInstance (){ 
		if (0 == _instance) { 
			_instance = new sceneData; 
		} 
		return _instance; 
	}

	static bool	processNeighbours();
	static bool	processSamples();
	static bool	modifyMeshNodeGroup();	// TODO modify the selection of vertices of some segmented mesh
	static bool writeToBuffer();

public:
	static std::list<jointPtr> _joints;
	static std::list<meshPtr> _meshes;
	static unsigned int _jointNum;
	static unsigned int _meshNum;

private:
	sceneData(){};
	static sceneData *_instance;
}; 

sceneData *sceneData::_instance = 0; 

unsigned int sceneData::_meshNum = 0;
unsigned int sceneData::_jointNum = 0;

std::list<jointData> _joints();
std::list<segData> _meshes();

#endif