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

#include "tree.h"

#define CheckError(stat,msg)		\
	if ( MS::kSuccess != stat ) {	\
	displayError(msg);			\
	continue;					\
	}


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

	unsigned int count = 0;

	// TODO : modify the tree structure, now noly consider the situation 
	//	that only one skincluster exists in the scene
	std::shared_ptr<binaryTree> tree(new binaryTree());

	// Iterate through graph and search for skinCluster nodes
	MItDependencyNodes iter( MFn::kInvalid);
	for ( ; !iter.isDone(); iter.next() ) {
		MObject object = iter.item();
		if (object.apiType() == MFn::kSkinClusterFilter) {
			count++;

			// For each skinCluster node, get the list of influence objects
			MFnSkinCluster skinCluster(object);
			MDagPathArray infs;
			MStatus stat;
			unsigned int nInfs = skinCluster.influenceObjects(infs, &stat);
			CheckError(stat,"Error getting influence objects.");

			if (0 == nInfs) {
				stat = MS::kFailure;
				CheckError(stat,"Error: No influence objects found.");
			}


			// set up joint heritage tree for this skinCluster
			for (unsigned int k = 0; k < nInfs; ++k) {
				MFnIkJoint fnJoint(infs[k]);
				if ( fnJoint.parentCount() ) {
					// Assume any joint can has only one parent parent joint
					MObject parent = fnJoint.parent(0);
					MFnDagNode fnParent(parent);
					std::shared_ptr<binaryTreeNode> node(new binaryTreeNode(k, infs[k].partialPathName().asChar(), fnJoint.transformation().asMatrix()));

					tree->insertNode(node, fnParent.name().asChar());

				} else {
					CheckError(stat,"Error: Node has no parent found.");
				}
			}
			// check for joint's heritage matrix 
			#ifdef _DEBUG
				tree->traverse(tree->root, printOp(file));
			#endif
			

			// loop through the geometries affected by this cluster
			unsigned int nGeoms = skinCluster.numOutputConnections();
			for (unsigned int i = 0; i < nGeoms; ++i) {
				unsigned int index = skinCluster.indexForOutputConnection(i,&stat);
				CheckError(stat,"Error getting geometry index.");

				// get the dag path of the ii'th geometry
				MDagPath skinPath;
				stat = skinCluster.getPathAtIndex(index,skinPath);
				CheckError(stat,"Error getting geometry path.");

				// get the vertex array of this geometry
				MFnMesh fnMesh( skinPath );
				MFloatPointArray vertexList;
				fnMesh.getPoints( vertexList, MSpace::kWorld );

				// initialize the non-repetitive point set for current geom
				std::set<unsigned int> pointSet;
				std::pair<std::set<unsigned int>::iterator,bool> insertResult;

				// iterate through the components of this geometry
				MItMeshPolygon meshIter(skinPath);

				// print out the path name of current skin mesh
				#ifdef _DEBUG
					fprintf(file, "%s %d %u \n",skinPath.partialPathName().asChar(), meshIter.count(), nInfs);
				#endif

				double maxFaceArea = -1;
				for ( /* nothing */ ; !meshIter.isDone(); meshIter.next() ) {
					// Get the weights for this face (one per influence object)
					MObject face = meshIter.currentItem(&stat);
					CheckError(stat,"Error getting component.");

					MDoubleArray wts;
					unsigned int infCount;
					stat = skinCluster.getWeights(skinPath,face,wts,infCount);
					CheckError(stat,"Error getting weights.");

					if (0 == infCount) {
						stat = MS::kFailure;
						CheckError(stat,"Error: 0 influence objects.");
					}

					
					// find the first two max weight values and the corresponding indices
					// Assume that there are at most two biggest weights which may be equal
					
					// #undef min  //for std::numeric_limit<float>::min()
					
					unsigned int resultIndex = 0;
					std::shared_ptr<binaryTreeNode> maxEffectJoint;	

					// current vertex has more than one influence joints
					if (infCount > 1){	
						unsigned int maxIndex1 = 0, maxIndex2 = 1;
						double max1 = wts[0], max2 = wts[1];
						for (unsigned int j = 1; j < infCount ; ++j ) {
							if ( max1 < wts[j]){
								max2 = max1;
								maxIndex2 = maxIndex1;
								max1 = wts[j];
								maxIndex1 = j;
							}
							else if ( max2 < wts[j] ){
								max2 = wts[j];
								maxIndex2 = j;
							}
						}
						std::shared_ptr<binaryTreeNode> result1, result2;
						if (max1 == max2){
							unsigned int level1 = 0, level2 = 0;
							tree->find(tree->root, findLevelOp(maxIndex1), level1, result1);
							tree->find(tree->root, findLevelOp(maxIndex1), level2, result2);
							if (level1 > level2){ // max1' level is higher than max2's 
								resultIndex = maxIndex2;
								maxEffectJoint = result2;
							}
						}
						else {
							resultIndex = maxIndex1;
							unsigned int level1 = 0;
							tree->find(tree->root, findLevelOp(maxIndex1), level1, result1);
							maxEffectJoint = result1;
						}
					}else{ // current vertex has only one influence joints
						maxEffectJoint = tree->root->child;
					}

					// TODO : allow user to interactively add or remove influenced meshes for each joint


					// add current face'points to mfloatpointarray
					MIntArray vertexIdxs;
					meshIter.getVertices( vertexIdxs );


					// TODO : there are some points lost and do not belong to any joint. why? 
					for (unsigned int k = 0;k < vertexIdxs.length(); k++ )
					{
						// process a face' points
						insertResult = pointSet.insert(vertexIdxs[k]);
						if (insertResult.second){ // the point does not exist in the set
							if (maxEffectJoint && maxEffectJoint->points){ // TODO why there will be crash here ? 
								MFloatPoint point = vertexList[vertexIdxs[k]];
								maxEffectJoint->points->append(point);
							}
						}
					}
					
					
					// TODO :  get the biggest face area
					double area = 0;
					if (! meshIter.zeroArea()){
						meshIter.getArea(area);
						if (area > maxFaceArea)
							maxFaceArea = area;
						if (maxEffectJoint && maxEffectJoint->points){
							maxEffectJoint->meshIdxs->push_back(std::make_pair(meshIter.index(),area));
						}
					}
				} // meshIter 

				
				// TODO : sampling points according to polygon face area for each joint
				


				// calculate the local coordinate for each joint by pca method
				



			} // loop through the geometries

		} // if the node is skincluster

	} // loop through the nodes in the scene 

	if (0 == count) {
		displayError("No skinClusters found in this scene.");
	}

	// check for each joint' influenced points
	#ifdef _DEBUG
		tree->traverse(tree->root, checkPointGrouplOp(file));
	#endif

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
