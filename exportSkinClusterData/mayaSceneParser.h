#ifndef MAYASCENEPARSER_H
#define MAYASCENEPARSER_H

#include <boost/functional/hash.hpp>
#include "common.h"
#include "sceneData.h"

class mayaSceneParser {
public:
	typedef std::unique_ptr<jointData> jDataPtr;
	typedef std::unique_ptr<meshData>  meshPtr;

	mayaSceneParser(){}

	static void setScenePtr(sceneData* scene){ _scene = scene;}

	static MStatus insertJoint(const MFnIkJoint& fnJoint);

	static MStatus insertMesh(const MFnSkinCluster& skinCluster, const MDagPath& skinPath);

private:
	static std::size_t getHashCode(const std::string& jointName){
		static boost::hash<std::string> string_hash;
		return string_hash(jointName);
	}

	static jDataPtr insertJointData(const MFnIkJoint& joint);

	static sceneData* _scene;
};


#endif