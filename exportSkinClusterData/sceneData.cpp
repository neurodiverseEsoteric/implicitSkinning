#include "sceneData.h"

sceneData *sceneData::_instance = 0; 

unsigned int sceneData::_meshNum = 0;
unsigned int sceneData::_jointNum = 0;

std::list<sceneData::jointPtr> sceneData::_joints = std::list<sceneData::jointPtr>();
std::list<sceneData::meshPtr> sceneData::_meshes = std::list<sceneData::meshPtr>();


bool sceneData::processNeighbours(){
	
	return true;
}


bool sceneData::processSamples(){
	
	return true;
}


bool sceneData::modifyMeshNodeGroup(){

	return true;
}


bool sceneData::writeToBuffer(){
	return true;
}


bool sceneData::fininalPrep(){
	return true;
}



