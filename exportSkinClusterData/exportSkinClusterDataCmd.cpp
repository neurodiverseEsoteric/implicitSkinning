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

#include "tree.h"
#include "Table.h"

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

	// only one treeList for the scene
	std::shared_ptr<std::vector<std::shared_ptr<binaryTree>>> treeList;


	// Iterate through graph and search for skinCluster nodes
	MItDependencyNodes iter( MFn::kInvalid);
	for ( ; !iter.isDone(); iter.next() ) {
		MObject object = iter.item();
		if (object.apiType() == MFn::kSkinClusterFilter) {
			
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


			// loop through the geometries affected by this cluster
			unsigned int nGeoms = skinCluster.numOutputConnections();
			for (unsigned int i = 0; i < nGeoms; ++i) {

				// construct a tree node for each geometry
				std::shared_ptr<binaryTree> tree(new binaryTree());
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
				// append to the treeList
				treeList->push_back(tree);


				// get the geometry's info\
				//
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

				// iterate through the components of this geometry
				MItMeshVertex vertexIter(skinPath);

				// set up mesh factory for this polygon mesh
				meshTableFactory mFactory(nInfs);
				mFactory.meshInit(vertexIter);
				std::shared_ptr<meshTable> mTable = mFactory.getMeshTable();

				// set up jointFactory
				jointsTableFactory mJointFactory(nInfs);

				// print out the path name of current skin mesh
				#ifdef _DEBUG
					fprintf(file, "%s %d %u \n",skinPath.partialPathName().asChar(), vertexIter.count(), nInfs);
				#endif

				for ( /* nothing */ ; !vertexIter.isDone(); vertexIter.next()) {

					// Get the weights for this vertex (one weight per influence object)
					MObject vertex = vertexIter.currentItem(&stat);
					CheckError(stat,"Error getting component.");

					MDoubleArray wts;
					unsigned int infCount;
					stat = skinCluster.getWeights(skinPath,vertex,wts,infCount);
					CheckError(stat,"Error getting weights.");

					if (0 == infCount) {
						stat = MS::kFailure;
						CheckError(stat,"Error: 0 influence objects.");
					}
										
					// push the weights into meshTable factory
					mFactory.addWeights(wts);
		
				} // vertexIter 

				// TODO : add concrete derived class of computeController
				computeController controller;
				mFactory.processWeights<computeController>(controller);
				

				//  convert the vertices into meshes, and then take sample 
				// points according to polygon face area for each joint
				MSelectionList singleVertexList;
				std::set<unsigned int> pointSet;
				unsigned int start = 0;
				MString meshName = skinPath.fullPathName();
				for ( unsigned int i = 1; i <= mFactory._numJoints; i++ ){
					singleVertexList.clear();
					pointSet.clear();

					// handle joints one by one, first select current joint' affected most vertex
					unsigned int length = mTable->pointIdxTable.size();
					for ( unsigned int j = start; j < length; j++){
						if ( mTable->ptJointIdxTable[j] == i ){
							MString vertexName = meshName;
							vertexName += ".vtx[";
							pointSet.insert(mTable->pointIdxTable[j]);
							vertexName += mTable->pointIdxTable[j];
							vertexName += "]";
							singleVertexList.add(vertexName);
						}
						else {
							start = j;
							break;
						}
					}
					

					MObject multiVertexComponent, singleVertexComponent;
					int dummyIndex;
					double maxArea = -1;
					MItMeshPolygon faceIter(skinPath);
					std::vector<unsigned int> faceIdxs;
					faceIdxs.reserve(faceIter.count());
					std::vector<double> faceAreas;
					faceAreas.reserve(faceIter.count());
					 
					// loop through the faces which have selected points
					for (MItSelectionList vertexComponentIter(singleVertexList, MFn::kMeshVertComponent); !vertexComponentIter.isDone(); vertexComponentIter.next()){
						// STORE THE DAGPATH, COMPONENT OBJECT AND MESH NAME OF THE CURRENT VERTEX COMPONENT:
						vertexComponentIter.getDagPath(skinPath, multiVertexComponent);

						// VERTEX COMPONENT HAS TO CONTAIN AT LEAST ONE VERTEX:
						if (!multiVertexComponent.isNull()){
							// ITERATE THROUGH EACH "VERTEX" IN THE CURRENT VERTEX COMPONENT:
							for (MItMeshVertex vertexIter(skinPath, multiVertexComponent); !vertexIter.isDone(); vertexIter.next()){

								// get current vertex' connected faces
								MIntArray connectedFacesIndices;
								vertexIter.getConnectedFaces(connectedFacesIndices);

								for (unsigned i=0; i<connectedFacesIndices.length(); i++){
									faceIter.setIndex(connectedFacesIndices[i], dummyIndex);
									MIntArray faceVerticesIndices;
									faceIter.getVertices(faceVerticesIndices);

									unsigned int totalPts = faceVerticesIndices.length();
									unsigned int notFoundNum = 0;
									for (unsigned int k = 0; k < totalPts; k++){
										auto resutl = pointSet.find(faceVerticesIndices[k]);
										if (resutl == pointSet.end()){
											notFoundNum++;
										}
									}
									// TODO : only consider triangle face now. 
									if ( float(notFoundNum)/ totalPts < 0.5 && !faceIter.zeroArea() ){
										// accept the face 
										double area(0);
										faceIter.getArea(area);
										if ( area > maxArea )
											maxArea = area;
										faceIdxs.push_back(faceIter.index());
										faceAreas.push_back(area);
									}
								} // filter every faces connected to current vertex 

							} // for MItMeshVertex

						} // if multiVertexComponent

					}// for vertexComponentIter

					// TODO : allow user to interactively add or remove influenced meshes for each joint


					// normalize the face areas
					for (auto iter = faceAreas.begin(); iter != faceAreas.end(); iter++) {
						(*iter) /= maxArea;
					}

					computeController controller;
					mJointFactory.addJointTable<computeController>(faceIter, faceIdxs, faceAreas, controller);
					
				} // for each joint

			} // loop through the geometries

		} // if the node is skincluster

	} // loop through the nodes in the scene 

	if (treeList->size() == 0) {
		displayError("No skinClusters or influenced geometries found in this scene.");
	}

	// check for each joint' influenced points
	#ifdef _DEBUG
	for (std::vector<std::shared_ptr<binaryTree>>::iterator it = treeList->begin() ; it != treeList->end(); ++it)
		(*it)->traverse((*it)->root, checkPointGrouplOp(file));
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
