#ifndef COMMON_H
#define COMMON_H

#include <maya/MPxCommand.h>
#include <maya/MStatus.h>
#include <maya/MArgList.h>
#include <maya/MFnPlugin.h>
#include <maya/MObject.h>
#include <maya/MGlobal.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MItSelectionList.h>
#include <maya/MSelectionList.h>
#include <maya/MIntArray.h>
#include <maya/MTime.h>
#include <maya/MAnimControl.h>
#include <maya/MItSelectionList.h>
#include <maya/MDagPathArray.h>

#include <maya/MItMeshPolygon.h>
#include <maya/MFnIkJoint.h>
#include <maya/MFloatPoint.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFnMesh.h>
#include <maya/MItMeshVertex.h>


#include "scene.h"
#include "transform.h"
#include "mayaSceneParser.h"

//#include <limits>       // std::numeric_limits

#define MATCH(str, shortName, longName) \
	(((str)==(shortName))||((str)==(longName)))

#define MCheckStatus(status,message)    \
	if( MStatus::kSuccess != status ) {     \
	cerr << message << "\n";                \
	return status;                                  \
	}

#endif