#include "mayaSceneParser.h"

sceneData* mayaSceneParser::_scene = nullptr;

MStatus mayaSceneParser::insertJoint(const MFnIkJoint& fnJoint){
	MStatus stat = MStatus::kSuccess;
	jDataPtr jData = std::move(insertJointData(fnJoint));

	// find fnJoint' parent
	if( fnJoint.parentCount() ) { // it is child joint
		// assume any joint can have only one parent
		MFnIkJoint fnParent(fnJoint.parent(0));

		std::string parentName	= fnParent.fullPathName().asChar();

		// TODO exclude world joint

		std::size_t parentCode	= getHashCode(parentName);
		std::list<sceneData::jointPtr>::const_iterator iter = _scene->_joints.begin();
		for (;iter != _scene->_joints.end(); iter++){
			if ((*iter)->_hashCode == parentCode ){
				jData->_parentPos = (*iter)->_index;
				break;
			}
		}

		// parent joint does not exist in the list, insert its parent 
		if ( iter == _scene->_joints.end() ){
			insertJointData(fnParent);		
		}	

	} else {
		MCheckStatus(stat,"Error: joint has no parent found.");
	}

	return stat;
}


MStatus mayaSceneParser::insertMesh(const MFnSkinCluster& skinCluster, const MDagPath& skinPath){
	MStatus stat;
	meshPtr mPtr(new meshData());

	//  insert vertices index on each polymesh into meshData
	MItMeshPolygon polyIter(skinPath, MObject::kNullObj, &stat); 
	MCheckStatus(stat,"Error getting geometry MItMeshPolygon.");
	int faceNum = polyIter.count();
	meshData::intVecPtr tmpNeightPtr(new int[faceNum]);
	{
		int count = 0;
		for ( ; !polyIter.isDone(); polyIter.next()){
			MIntArray vtxlist;
			polyIter.getVertices(vtxlist);

			int length = vtxlist.length();
			for (int i = 0; i < length; i++){
				tmpNeightPtr[count++] = vtxlist[i];
			}
		}
	}
	mPtr->_neighbourPtr = std::move(tmpNeightPtr);


	// get the number of vertices
	MFnMesh fnMesh(skinPath, &stat);
	MCheckStatus(stat,"Error getting fnMesh component.");
	MFloatPointArray pts;
	fnMesh.getPoints(pts);
	unsigned int nPoints = pts.length();


	//  insert vertices's positions into meshData
	int posLen = nPoints * 3;
	meshData::floatVecPtr tmpPosPtr(new float[posLen]);
	// SSE enabled. Innter loop will autovectorize. Around 3x faster than the loop below it. It would be faster if first element was
	// guaranteed to be aligned on 16 byte boundary.
	for(unsigned int i=0; i < nPoints; i++) {
		float* ptPtr = &pts[i].x;
		for(unsigned int j=0; j<4; j++) {
			tmpPosPtr[i * 3 + j] = ptPtr[j];
		}
	}
	mPtr->_posPtr = std::move(tmpPosPtr);


	//  insert vertices's weights into meshData
	MItMeshVertex vertIter(skinPath);
	MDagPathArray jointArray;
	unsigned int numJoints = skinCluster.influenceObjects(jointArray, &stat);

	unsigned int wLen = nPoints * numJoints;
	meshData::floatVecPtr tmpWPtr(new float[wLen]);

	for (unsigned int i = 0; i < nPoints; i++) {
		unsigned int tmp;
		int preIndex;

		CHECK_MSTATUS ( vertIter.setIndex( i, preIndex) );
		MObject vertex = vertIter.currentItem(&stat);

		MDoubleArray wts;
		CHECK_MSTATUS ( skinCluster.getWeights(skinPath, vertex, wts, tmp) );

		for (unsigned int j = 0; j < numJoints; j++) {
			tmpWPtr[i * numJoints + j] = (float)wts[j];
		} 
	} // loop over all points
	mPtr->_weightsPtr = std::move(tmpWPtr);

	_scene->_meshes.push_back(std::move(mPtr));
	return stat;
}

mayaSceneParser::jDataPtr mayaSceneParser::insertJointData(const MFnIkJoint& joint){
	jDataPtr jData(new jointData());
	jData->_index			= _scene->_jointNum++;
	jData->_parentPos		= -1;

	std::string jointName	= joint.fullPathName().asChar();
	jData->_hashCode		= getHashCode(jointName);

	MMatrix matrix			= joint.transformation().asMatrix().transpose();
	float fMatrix[4][4];
	matrix.get(fMatrix);
	jData->_transform = Transform(fMatrix);

	_scene->_joints.push_back(std::move(jData));
	return jData;
}