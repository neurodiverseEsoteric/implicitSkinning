
#include <maya/MDoubleArray.h>
#include <maya/MObjectArray.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MItGeometry.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MIOStream.h>


#include <math.h>
#include <map>
#include <boost/interprocess/managed_shared_memory.hpp> 
//#include <boost/functional/hash.hpp>

#include "common.h"
#include "Table.h"

///////////////////////////////////////////////////////
//
// DESCRIPTION:  implicitSkinningPrep command
//
///////////////////////////////////////////////////////


class implicitSkinningPrep : public MPxCommand
{
public:
	implicitSkinningPrep();
    virtual     ~implicitSkinningPrep();
	static      void* creator();

	bool        isUndoable() const;
	MStatus     undoIt ();

	MStatus     redoIt ();
	MStatus     doIt ( const MArgList& args );

private:
	int			_startFrame;
	int			_endFrame;
	int			_byFrame;

	MStatus		nodeFromName(MString name, MObject & obj) const;
	void		readSceneStartEnd();
	MStatus		parseArgs( const MArgList& args);
	int			intArg(const MArgList& args, unsigned int &indx, int & res);
};

implicitSkinningPrep::implicitSkinningPrep():_startFrame(0), _endFrame(0), _byFrame(1){}


implicitSkinningPrep::~implicitSkinningPrep() {}


void* implicitSkinningPrep::creator()
{
	return new implicitSkinningPrep;
}


bool implicitSkinningPrep::isUndoable() const
{
	return false;
}


MStatus implicitSkinningPrep::undoIt()
{
	return MS::kSuccess;
}


MStatus implicitSkinningPrep::redoIt()
{
	clearResult();
	setResult( (int) 1);
	return MS::kSuccess;
}


MStatus implicitSkinningPrep::nodeFromName(MString name, MObject & obj) const
{
	MSelectionList tempList;
	tempList.add( name );
	if ( tempList.length() > 0 ) 
	{
		tempList.getDependNode( 0, obj );
		return MS::kSuccess;
	}
	return MS::kFailure;
}

void implicitSkinningPrep::readSceneStartEnd()
{
	MTime startFrame;
	MTime endFrame;

	// Get the render globals node
	//
	int rangeIsSet = 0;
	MObject renderGlobNode;
	if (nodeFromName("defaultRenderGlobals", renderGlobNode) == MS::kSuccess)
	{
		MFnDependencyNode fnRenderGlobals( renderGlobNode );

		// Check if the time-slider or renderGlobals is used for 
		// the frame range
		//
		MPlug animPlug = fnRenderGlobals.findPlug( "animation" );
		short anim;
		animPlug.getValue( anim );

		if ( anim ) {
			float byFrame;
			fnRenderGlobals.findPlug( "startFrame"  ).getValue(startFrame);
			fnRenderGlobals.findPlug( "endFrame"    ).getValue(endFrame);
			fnRenderGlobals.findPlug( "byFrameStep" ).getValue(byFrame);
			_byFrame = (int) byFrame;
			rangeIsSet = 1;
		}
	}
	if (!rangeIsSet)	{
		// USE_TIMESLIDER
		startFrame = MAnimControl::minTime();
		endFrame   = MAnimControl::maxTime();
		_byFrame = 1;
	}
	_startFrame = (int) startFrame.as( MTime::uiUnit() );
	_endFrame   = (int)   endFrame.as( MTime::uiUnit() );
}


int implicitSkinningPrep::intArg(const MArgList& args, unsigned int &indx, int & res)
{
	if (indx < args.length())
	{
		MStatus stat;
		indx++;
		res = args.asInt( indx, &stat );
		if (stat == MS::kSuccess) 
			return 1;
	}
	return 0;
}


MStatus implicitSkinningPrep::parseArgs( const MArgList& args )
{
	// Parse the arguments.
	MString arg;
	MStatus stat = MS::kSuccess;
	MString str;
	MObject cameraNode;
	unsigned int i;

	for ( i = 0; i < args.length(); i++ ) {
		arg = args.asString( i, &stat );
		if (stat != MS::kSuccess)
			continue;

		if (MATCH(arg, "-s", "-start"))
			intArg(args, i, _startFrame);
		else if (MATCH(arg, "-e", "-end"))
			intArg(args, i, _endFrame);
		else if (MATCH(arg, "-by", "-byFrame"))
			intArg(args, i, _byFrame);
		else{
			fprintf(stderr, "Unknown argument '%s'\n", arg.asChar());
			fflush(stderr);
		}
	}

	if (_byFrame<=0) fByFrame = 1;

	return MS::kSuccess;
}


MStatus implicitSkinningPrep::doIt( const MArgList& args )
{
	// parse the command arguments
	MStatus stat = parseArgs(args);
	MCheckStatus(stat,"ERROR setting parameters");

	if (_startFrame > _endFrame)
		readSceneStartEnd();

	// Remember the frame the scene was at so we can restore it later.
	MTime currentFrame = MAnimControl::currentTime();

	// move to start frame to get the skeleton matrix at rest pose
	MGlobal::viewFrame(-_startFrame);


	// allocate memory by IPC


	// create one sceneData instance for the entire scene
	sceneData *scene = sceneData::getInstance();
	mayaSceneParser::setScenePtr(scene);

	// Iterate through all selected skinCluster nodes
	MSelectionList sl;
	stat = MGlobal::getActiveSelectionList(sl);
	MCheckStatus(stat,"ERROR select none objects");

	MItSelectionList iter(sl, MFn::kSkinClusterFilter, &stat);
	MCheckStatus(stat,"ERROR filter kSkinClusterFilter objects");

	for ( ; !iter.isDone(); iter.next() ) {
		MObject object;
		iter.getDependNode(object);

		// For each skinCluster node, get the list of joints
		MFnSkinCluster	skinCluster(object);
		MDagPathArray	jointArray;
		unsigned int	numJoints = skinCluster.influenceObjects(jointArray, &stat);
		MCheckStatus(stat,"Error: getting joints.");
		if (0 == numJoints) {
			stat = MS::kFailure;
			MCheckStatus(stat,"Error: No joints in this skinCluster found.");
		}

		// set up the joint linear tree representation for current skinCluster node
		for (unsigned int i = 0; i < numJoints; i++) {
			MFnIkJoint fnJoint(jointArray[i]);
			// parse the joint data and insert joint into the scene and set the parent index
			mayaSceneParser::insertJoint(fnJoint);
		}
			

		// loop through the geometries affected by this cluster
		unsigned int nGeoms = skinCluster.numOutputConnections();
		for (unsigned int i = 0; i < nGeoms; ++i) {

			// push the skin-mesh and skinCluster pair into scene
			unsigned int index = skinCluster.indexForOutputConnection(i, &stat);
			MCheckStatus(stat,"Error getting geometry index.");

			// get the geometry mesh
			MDagPath skinPath;
			stat = skinCluster.getPathAtIndex(index,skinPath);
			MCheckStatus(stat,"Error getting geometry path.");

			// insert vertices index on each polymesh into meshData. 
			// it is to be used to find neighbourhood of each vertex
			mayaSceneParser::insertMesh(skinPath);
			

			skNode->addWeights();
			skNode->addNeighbors();

			// push the skinClusterNode-skinMeshNode pair into the scene
			scene->push_back(std::pair<skinClusterNode, skinNode>(scNode, skNode));

		} // loop through the geometries

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


	// Restore back to the frame we were at before we ran command
	MGlobal::viewFrame (currentFrame);
	return MS::kSuccess;
}








///////////////////////////////////////////////////////
//
// DESCRIPTION:  rbfDeform command
//
///////////////////////////////////////////////////////

class rbfDeform : public MPxCommand
{
public:
	rbfDeform(){};
	virtual     ~rbfDeform(){};

	MStatus     doIt ( const MArgList& args );
	MStatus     redoIt ();
	MStatus     undoIt ();
	bool        isUndoable() const;
	static      void* creator();
};

void* rbfDeform::creator()
{
	return new rbfDeform;
}

bool rbfDeform::isUndoable() const
{
	return false;
}

MStatus rbfDeform::undoIt()
{
	return MS::kSuccess;
}

MStatus rbfDeform::redoIt()
{
	clearResult();
	setResult( (int) 1);
	return MS::kSuccess;
}

MStatus rbfDeform::doIt( const MArgList& args ){
	MStatus   status = MStatus::kSuccess;

	MTime currentFrame = MAnimControl::currentTime();

	// read data from the shared memory or file
	boost::interprocess::managed_shared_memory ipcMemo(boost::interprocess::open_read_only, "sceneMemory");
	// 


	// update the joint matrix
	

	// update the local coord
	


	// filter the vertices of each mesh which is inside of the intersection region of adjacent joints



	// get the position of these vertices
	


	// calculate the position for each vertex ( project and relax )



	// set the final position for each mesh object



	// display result
	MString msg = "rbfDeform deform the mesh at " + (int) currentFrame.asUnits(MTime::uiUnit());
	msg += " frame.\n";
	MGlobal::displayInfo(msg);

	setResult(msg);

	return status;
}


MStatus initializePlugin( MObject obj )
{
	MStatus   status = MStatus::kSuccess;

	MFnPlugin plugin( obj, PLUGIN_COMPANY, "3.0", "Any");
	status = plugin.registerCommand( "exportSkinClusterData", exportSkinClusterData::creator );
	if (!status) {
		status.perror("registerCommand");
		return status;
	}

	status = plugin.registerCommand( "rbfDeform", rbfDeform::creator );
	if (!status) {
		status.perror("deregisterCommand");
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

	status = plugin.deregisterCommand( "rbfDeform" );
	if (!status) {
		status.perror("deregisterCommand");
		return status;
	}

	return status;
}
