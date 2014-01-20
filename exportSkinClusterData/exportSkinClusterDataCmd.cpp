#include <math.h>
#include <set>
//#include <limits>       // std::numeric_limits

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
#include <maya/MDoubleArray.h>
#include <maya/MObjectArray.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MItGeometry.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MIOStream.h>

// add more include file
#include <maya/MItMeshPolygon.h>
#include <maya/MFnIkJoint.h>
#include <maya/MFloatPoint.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFnMesh.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItSelectionList.h>
#include <maya/MDagPathArray.h>

#include "tree.h"
#include "Table.h"


class exportSkinClusterData : public MPxCommand
{
public:
	exportSkinClusterData();
	virtual     ~exportSkinClusterData();

	MStatus		parseArgs( const MArgList& args );
	MStatus     doIt ( const MArgList& args );
	MStatus     redoIt ();
	MStatus     undoIt ();
	bool        isUndoable() const;

	static      void* creator();

private:
	FILE*		file;
};

exportSkinClusterData::exportSkinClusterData():
file(NULL)
{
}

exportSkinClusterData::~exportSkinClusterData() {}

void* exportSkinClusterData::creator()
{
	return new exportSkinClusterData;
}

bool exportSkinClusterData::isUndoable() const
{
	return false;
}

MStatus exportSkinClusterData::undoIt()
{
	return MS::kSuccess;
}

MStatus exportSkinClusterData::parseArgs( const MArgList& args )
	//
	// There is one mandatory flag: -f/-file <filename>
	//
{
	MStatus     	stat;
	MString     	arg;
	MString			fileName;
	const MString	fileFlag			("-f");
	const MString	fileFlagLong		("-file");

	// Parse the arguments.
	for ( unsigned int i = 0; i < args.length(); i++ ) {
		arg = args.asString( i, &stat );
		if (!stat)              
			continue;

		if ( arg == fileFlag || arg == fileFlagLong ) {
			// get the file name
			//
			if (i == args.length()-1) {
				arg += ": must specify a file name";
				displayError(arg);
				return MS::kFailure;
			}
			i++;
			args.get(i, fileName);
		}
		else {
			arg += ": unknown argument";
			displayError(arg);
			return MS::kFailure;
		}
	}

	file = fopen(fileName.asChar(),"wb");
	if (!file) {
		MString openError("Could not open: ");
		openError += fileName;
		displayError(openError);
		stat = MS::kFailure;
	}

	return stat;
}


MStatus exportSkinClusterData::doIt( const MArgList& args )
{
	// parse args to get the file name from the command-line
	MStatus stat = parseArgs(args);
	if (stat != MS::kSuccess) {
		return stat;
	}


	// allocate memory by IPC


	// move to start frame to get the matrix at time 0 



	// only one sceneList for the entire scene
	std::shared_ptr<std::list<std::pair<std::shared_ptr<skinClusterNode>, std::shared_ptr<skinNode>>>> scene 
			= sceneList::getInstance();

	// construct compute controller
	computeController controller;

	// Iterate through graph and search for skinCluster nodes
	MItDependencyNodes iter( MFn::kInvalid);
	for ( ; !iter.isDone(); iter.next() ) {
		MObject object = iter.item();
		if (object.apiType() == MFn::kSkinClusterFilter) {
			
			// For each skinCluster node, get the list of joints
			MFnSkinCluster skinCluster(object);
			MDagPathArray jointArray;
			MStatus stat;
			unsigned int numJoints = skinCluster.influenceObjects(jointArray, &stat);
			CheckError(stat,"Error getting influence objects.");
			if (0 == numJoints) {
				stat = MS::kFailure;
				CheckError(stat,"Error: No joints in this skinCluster found.");
			}


			// set up the joint heritage tree for current skinCluster node
			std::shared_ptr<skinClusterNode> scNode = std::make_shared<skinClusterNode>();
			for (unsigned int i = 0; i < numJoints; i++) {
				MFnIkJoint fnJoint(jointArray[i]);
				std::shared_ptr<jointNode> joint = std::make_shared<jointNode>(i, fnJoint.dagPath());
				joint->startMatrix = fnJoint.transformation().asMatrix();

				// Assume any joint can has only one parent parent joint
				if ( fnJoint.parentCount() ) { 
					// it is child joint
					MObject parent = fnJoint.parent(0);
					MFnDagNode fnParent(parent);
					
					// TODO if parent is world, then what is its dagpath
					scNode->insertNode(joint, fnParent.dagPath());

				} else {
					CheckError(stat,"Error: Node has no parent found.");
				}
			}
			// check for joint's heritage matrix 
			#ifdef _DEBUG
			scNode->traverse(printOp(file));
			#endif
			

			// loop through the geometries affected by this cluster
			unsigned int nGeoms = skinCluster.numOutputConnections();
			for (unsigned int i = 0; i < nGeoms; ++i) {

				// push the skin-mesh and skinCluster pair into scene
				unsigned int index = skinCluster.indexForOutputConnection(i,&stat);
				CheckError(stat,"Error getting geometry index.");

				MDagPath skinPath;
				stat = skinCluster.getPathAtIndex(index,skinPath);
				CheckError(stat,"Error getting geometry path.");

				auto skNode = std::make_shared<skinNode>(skinCluster, scNode, skinPath);
				// push vertex weights into table
				skNode->addWeights();
				// construct vertex neight info tables
				skNode->addNeighbors();

				// push the skinClusterNode-skinMeshNode pair into the scene
				scene->push_back(std::pair<skinClusterNode, skinNode>(scNode, skNode));

			} // loop through the geometries

		} // if the node is skincluster

	} // loop through the nodes in the scene 
	
	if (scene->size() == 0) {
		displayError("No skinClusters found in this scene.");
	}
	

	// set up mesnNode list for each skin mesh according to each vertex's weights 
	sceneList::processWeights(controller);

	// TODO let user to interactively adjust vertex group
	sceneList::modifyMeshNodeGroup();

	// set rbf params and local coord based on meshNode' vertexs
	sceneList::processSamples(controller);

	
	// when needed, write the scene object into a file by serialization.
	// otherwise all the object are store by IPC in the memory with tag.
	// sceneList::writeToBuffer();

	fclose(file);
	return MS::kSuccess;
}

MStatus exportSkinClusterData::redoIt()
{
	clearResult();
	setResult( (int) 1);
	return MS::kSuccess;
}

MStatus initializePlugin( MObject obj )
{
	MStatus   status;
	MFnPlugin plugin( obj, PLUGIN_COMPANY, "3.0", "Any");

	status = plugin.registerCommand( "exportSkinClusterData", exportSkinClusterData::creator );
	if (!status) {
		status.perror("registerCommand");
		return status;
	}

	return status;
}

MStatus uninitializePlugin( MObject obj )
{
	MStatus   status;
	MFnPlugin plugin( obj );

	status = plugin.deregisterCommand( "exportSkinClusterData" );
	if (!status) {
		status.perror("deregisterCommand");
	}
	return status;
}
