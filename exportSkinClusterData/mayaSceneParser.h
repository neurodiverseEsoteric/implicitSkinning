#ifndef MAYASCENEPARSER_H
#define MAYASCENEPARSER_H

#include "common.h"
#include <boost/functional/hash.hpp>

class mayaSceneParser {
public:
	typedef std::unique_ptr<jointData> jDataPtr;
	typedef std::unique_ptr<meshData>  meshPtr;

	mayaSceneParser(){}

	static void setScenePtr(const sceneData* const scene){_scene = scene};

	static MStatus insertJoint(const MFnIkJoint& fnJoint){
		MStatus stat = MStatus::kSuccess;
		insertJointData(fnJoint);

		// find fnJoint' parent
		if ( fnJoint.parentCount() ) { // it is child joint
			// assume any joint can have only one parent
			MFnIkJoint fnParent(fnJoint.parent(0));
			
			std::string parentName	= fnParent.fullPathName().asChar();
			
			// TODO exclude world joint

			std::size_t parentCode	= getHashCode(parentName);
			std::list<jointData>::const_iterator iter = scene->_joints.begin();
			for (;iter != scene->_joints.end(); iter++){
				if (*iter->_hashCode == parentCode ){
					jData->_parentPos = *iter->_index;
					break;
				}
			}
			
			// parent joint does not exist in the list, insert its parent 
			if ( iter == scene->_joints.end() ){
				insertJointData(fnParent);		
			}	

		} else {
			MCheckStatus(stat,"Error: joint has no parent found.");
		}
		
		return stat;
	}


	static MStatus insertMesh(const MDagPath& skinPath){
		MStatus stat;
		meshPtr mPtr(new meshData());

		MItMeshPolygon polyIter(skinPath, MObject::kNullObj, &stat); 
		MCheckStatus(stat,"Error getting geometry MItMeshPolygon.");
		
		//  insert vertices index on each polymesh into meshData
		for ( ; !polyIter.isDone(); polyIter.next()){
			MIntArray vtxlist;
			polyIter.getVertices(vtxlist);

			int length = vtxlist.length();
			for (int i = 0; i < length; i++){
				mPtr->_vtxPtr->push_back(vtxlist[i]);
			}
		}

		_scene->_meshes.push_back(mPtr);

		return stat;
	}

private:
	static std::size_t getHashCode(const std::string& jointName){
		static boost::hash<std::string> string_hash;
		return string_hash(jointName);
	}

	static void insertJointData(const MFnIkJoint& joint){
		jDataPtr jData(new jointData());
		jData->_index			= _scene->_jointNum++;
		jData->_parentPos		= -1;

		std::string jointName	= joint.fullPathName().asChar();
		jData->_hashCode		= getHashCode(jointName);

		MMatrix matrix			= joint.transformation().asMatrix().transpose();
		float fMatrix[4][4];
		matrix.get(fMatrix);
		jData->_transform(fMatrix);

		_scene->_joints.push_back(jData);
	}

	static sceneData* _scene;
};

sceneData* mayaSceneParser::_scene = 0;

#endif