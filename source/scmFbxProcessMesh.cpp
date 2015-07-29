#include <deque>
#include <fbxsdk.h>

#include "../include/scmLog.h"
#include "../include/scmUtility.h"
#include "../include/scmScmSerializer.h"
#include "../include/scmFbxProcessMesh.h"

namespace scm
{
	static FbxScene * fbxScene;

	//=================================================================

	static int32_t ProcessFbxSkeletonNodeRecursive(
		FbxNode * fbxSkeletonNode,
		int32_t parentIndex,
		int32_t thisIndex)
	{
		if (!fbxSkeletonNode->GetNodeAttribute())
			return 0;
		if (!fbxSkeletonNode->GetNodeAttribute()->GetAttributeType())
			return 0;
		if (fbxSkeletonNode->GetNodeAttribute()->GetAttributeType() != FbxNodeAttribute::eSkeleton)
			return 0;

		char scmJointName[kSCM_NameLen];
		int32_t scmJointParent;
		int32_t scmJointChildren[kSCM_MaxChildJoints];
		int32_t scmJointGroup;
		int32_t scmJointFlags;
		float_t scmJointCollisionBox[4 * 6];

		memcpy(&scmJointName[0], &fbxSkeletonNode->GetName()[0], sizeof(char)* kSCM_NameLen);
		scmJointParent = parentIndex;
		memset(&scmJointChildren[0], -1, sizeof(int32_t)* kSCM_MaxChildJoints);
		scmJointGroup = 0;
		scmJointFlags = 0;

		// Plane = normal.x, normal.y, normal.z, distance
		scmJointCollisionBox[0 ] =  1.0;	scmJointCollisionBox[1 ] =  0.0;		// Plane 0
		scmJointCollisionBox[2 ] =  0.0;	scmJointCollisionBox[3 ] =  99999.0;

		scmJointCollisionBox[4 ] =  0.0;	scmJointCollisionBox[5 ] =  1.0;		// Plane 1
		scmJointCollisionBox[6 ] =  0.0;	scmJointCollisionBox[7 ] =  99999.0;

		scmJointCollisionBox[8 ] =  0.0;	scmJointCollisionBox[9 ] =  0.0;		// Plane 2
		scmJointCollisionBox[10] =  1.0;	scmJointCollisionBox[11] =  99999.0;

		scmJointCollisionBox[12] = -1.0;	scmJointCollisionBox[13] =  0.0;		// Plane 3
		scmJointCollisionBox[14] =  0.0;	scmJointCollisionBox[15] =  99999.0;

		scmJointCollisionBox[16] =  0.0;	scmJointCollisionBox[17] = -1.0;		// Plane 4
		scmJointCollisionBox[18] =  0.0;	scmJointCollisionBox[19] =  99999.0;

		scmJointCollisionBox[20] =  0.0;	scmJointCollisionBox[21] =  0.0;		// Plane 5
		scmJointCollisionBox[22] = -1.0;	scmJointCollisionBox[23] =  99999.0;
	
		ScmSerializer::AddJoint(
			scmJointName,
			scmJointParent,
			scmJointChildren,
			scmJointGroup,
			scmJointFlags,
			scmJointCollisionBox);

		// Recurse over all children
		int32_t nextIndex = thisIndex;
		int32_t childCount = fbxSkeletonNode->GetChildCount();
		for (int32_t i = 0; i < childCount; i++)
		{
			nextIndex = ProcessFbxSkeletonNodeRecursive(
				fbxSkeletonNode->GetChild(i),
				thisIndex,
				nextIndex + 1);
		}

		return nextIndex;
	}

	//=================================================================

	static void BuildBaseFrameRecursive(FbxNode * fbxSkeletonNode)
	{
		if (!fbxSkeletonNode->GetNodeAttribute())
			return;
		if (!fbxSkeletonNode->GetNodeAttribute()->GetAttributeType())
			return;
		if (fbxSkeletonNode->GetNodeAttribute()->GetAttributeType() != FbxNodeAttribute::eSkeleton)
			return;

		// Grab animation info
		FbxAnimStack * fbxAnimStack = fbxScene->GetSrcObject<FbxAnimStack>(0);
		FbxString fbxAnimStackName = fbxAnimStack->GetName();
		FbxTakeInfo * fbxTakeInfo = fbxScene->GetTakeInfo(fbxAnimStackName);

		FbxTime fbxStart = fbxTakeInfo->mLocalTimeSpan.GetStart();
		FbxLongLong fbxAnimStart = fbxStart.GetFrameCount(FbxTime::eFrames24);

		FbxTime fbxBaseTime;
		fbxBaseTime.SetFrame(fbxAnimStart, FbxTime::eFrames24);

		// Get frame transformation
		FbxAMatrix fbxFrameTransform = fbxSkeletonNode->EvaluateLocalTransform(fbxBaseTime);

		FbxVector4 fbxFrameTranslation	= fbxFrameTransform.GetT();
		FbxVector4 fbxFrameScaling		= fbxFrameTransform.GetS();
		FbxVector4 fbxFrameRotation		= fbxFrameTransform.GetR();

		// Add base frame for this joint
		ScmSerializer::SequenceAddJointFrame(
			fbxSkeletonNode->GetName(),
			fbxFrameTranslation.mData[0],
			fbxFrameTranslation.mData[1],
			fbxFrameTranslation.mData[2],
			fbxFrameScaling.mData[0],
			fbxFrameScaling.mData[1],
			fbxFrameScaling.mData[2],
			fbxFrameRotation.mData[0],
			fbxFrameRotation.mData[1],
			fbxFrameRotation.mData[2]);

		// Recurse over all children
		int32_t childCount = fbxSkeletonNode->GetChildCount();
		for (int32_t i = 0; i < childCount; i++)
		{
			BuildBaseFrameRecursive(fbxSkeletonNode->GetChild(i));
		}
	}

	//=================================================================

	static void ProcessFbxSkeletonNode(FbxNode * fbxSkeletonNode)
	{
		scmprint("\tProcessing Fbx Skeleton Node\r\n");

		// Build skeleton
		ProcessFbxSkeletonNodeRecursive(fbxSkeletonNode, -1, 0);
		ScmSerializer::BuildSkeletonChildren();

		// Add baseframe sequence so that it's guaranteed first
		ScmSerializer::SequenceBegin("baseframe", 10.0);
		BuildBaseFrameRecursive(fbxSkeletonNode);
		ScmSerializer::SequenceEnd();
	}

	//=================================================================

	static const int8_t CheckFbxMeshIsValid(FbxNode * fbxMeshNode)
	{
		if (!fbxMeshNode) {
			scmprint("Mesh not valid: null pointer received\r\n");
			return 0;
		}
		if (!fbxMeshNode->GetNodeAttribute()) {
			scmprint("Mesh not valid: no node attribute received\r\n");
			return 0;
		}
		if (fbxMeshNode->GetNodeAttribute()->GetAttributeType() != FbxNodeAttribute::eMesh) {
			scmprint("Mesh not valid: Non-mesh FbxNode received\r\n");
			return 0;
		}

		FbxMesh * fbxMesh = fbxMeshNode->GetMesh();
		if (!fbxMesh) {
			scmprint("Mesh not valid: Failed to retrieve FbxMesh from FbxNode\r\n");
			return 0;
		}
		if (!fbxMesh->IsTriangleMesh()) {
			scmprint("Mesh not valid: Mesh must be triangulated before converting\r\n");
			return 0;
		}
		return 1;
	}

	//=================================================================

	static const uint32_t ExtractFbxMeshControlPoints(FbxNode * fbxMeshNode, float_t *& outPositionsAlloc)
	{
		scmprint("Extracting Fbx mesh control points\r\n");

		FbxMesh * fbxMesh = fbxMeshNode->GetMesh();

		uint32_t fbxCtrlPtCount = fbxMesh->GetControlPointsCount();

		outPositionsAlloc = (float_t *)malloc(sizeof(float_t)* 3 * fbxCtrlPtCount);

		for (uint32_t i = 0; i < fbxCtrlPtCount; i++)
		{
			FbxVector4 fbxCtrlPtPosition = fbxMesh->GetControlPointAt(i);
			uint32_t scmIndexV0 = i * 3;
			outPositionsAlloc[scmIndexV0] = (float_t)fbxCtrlPtPosition.mData[0];
			outPositionsAlloc[scmIndexV0 + 1] = (float_t)fbxCtrlPtPosition.mData[1];
			outPositionsAlloc[scmIndexV0 + 2] = (float_t)fbxCtrlPtPosition.mData[2];
		}

		return fbxCtrlPtCount;
	}

	//=================================================================

	static void ExtractFbxMeshVertexJointsOffsetsWeights(
		FbxNode * fbxMeshNode,
		float_t *& controlPoints, uint32_t numCtrlPts,
		int8_t *& outJoints1Alloc, float_t *& outOffsets1Alloc, float_t *& outWeights1Alloc,
		int8_t *& outJoints2Alloc, float_t *& outOffsets2Alloc, float_t *& outWeights2Alloc)
	{
		scmprint("Extracting Fbx mesh offsets, weights and joints\r\n");

		FbxMesh * fbxMesh = fbxMeshNode->GetMesh();

		outJoints1Alloc		= (int8_t *)malloc(sizeof(int8_t)* numCtrlPts);
		outOffsets1Alloc	= (float_t *)malloc(sizeof(float_t)* 3 * numCtrlPts);
		outWeights1Alloc	= (float_t *)malloc(sizeof(float_t)* numCtrlPts);
		outJoints2Alloc		= (int8_t *)malloc(sizeof(int8_t)* numCtrlPts);
		outOffsets2Alloc	= (float_t *)malloc(sizeof(float_t)* 3 * numCtrlPts);
		outWeights2Alloc	= (float_t *)malloc(sizeof(float_t)* numCtrlPts);

		memset(&outJoints1Alloc[0], -1, sizeof(int8_t)* numCtrlPts);
		memset(&outOffsets1Alloc[0], 0, sizeof(float_t)* 3 * numCtrlPts);
		memset(&outWeights1Alloc[0], 0, sizeof(float_t)* numCtrlPts);
		memset(&outJoints2Alloc[0], -1, sizeof(int8_t)* numCtrlPts);
		memset(&outOffsets2Alloc[0], 0, sizeof(float_t)* 3 * numCtrlPts);
		memset(&outWeights2Alloc[0], 0, sizeof(float_t)* numCtrlPts);

		uint32_t numDeformers = fbxMesh->GetDeformerCount();
		scmprint("\tNumDeformers: %d\r\n", numDeformers);
		for (uint32_t i = 0; i < numDeformers; i++)
		{
			FbxSkin * fbxSkin = (FbxSkin *)fbxMesh->GetDeformer(i, FbxDeformer::eSkin);
			if (!fbxSkin)
			{
				scmprint("\t\tNo skin retrieved\r\n");
				continue;
			}

			uint32_t numClusters = fbxSkin->GetClusterCount();
			scmprint("\t\tNumClusters: %d\r\n", numClusters);
			for (uint32_t j = 0; j < numClusters; j++)
			{
				FbxCluster	* fbxCluster = fbxSkin->GetCluster(j);

				uint32_t	numIndices = fbxCluster->GetControlPointIndicesCount();
				double_t	* fbxJointWeights = fbxCluster->GetControlPointWeights();
				int32_t		* fbxJointIndices = fbxCluster->GetControlPointIndices();
				const char	* fbxJointName = fbxCluster->GetLink()->GetName();
				uint32_t	fbxJointIndex = ScmSerializer::GetJointIndexFromName(fbxJointName);
				scmprint("\t\t\tJoint: %s [%d]\r\n", fbxJointName, fbxJointIndex);

				FbxAMatrix fbxJointTransform;
				FbxAMatrix fbxJointTransformLink;
				FbxAMatrix fbxJointGlobalBindPoseInverse;

				fbxCluster->GetTransformMatrix(fbxJointTransform);
				fbxCluster->GetTransformLinkMatrix(fbxJointTransformLink);
				fbxJointGlobalBindPoseInverse = fbxJointTransformLink.Inverse() * fbxJointTransform;

				scmprint("\t\t\tNumIndices: %d\r\n", numIndices);
				for (uint32_t k = 0; k < numIndices; k++)
				{
					int8_t	& joint1 = outJoints1Alloc[fbxJointIndices[k]];
					int8_t	& joint2 = outJoints2Alloc[fbxJointIndices[k]];

					float_t * controlPoint = &controlPoints[fbxJointIndices[k] * 3];

					float_t & weight1 = outWeights1Alloc[fbxJointIndices[k]];
					float_t	& weight2 = outWeights2Alloc[fbxJointIndices[k]];

					if (joint1 == -1)
					{
						float_t * offset1 = &outOffsets1Alloc[fbxJointIndices[k] * 3];
					
						joint1		= fbxJointIndex;

						FbxVector4 fbxOffset1;
						fbxOffset1.mData[0] = controlPoint[0];
						fbxOffset1.mData[1] = controlPoint[1];
						fbxOffset1.mData[2] = controlPoint[2];
						fbxOffset1 = fbxJointGlobalBindPoseInverse.MultT(fbxOffset1);
						offset1[0] = fbxOffset1.mData[0];
						offset1[1] = fbxOffset1.mData[1];
						offset1[2] = fbxOffset1.mData[2];

						weight1		= (float_t)fbxJointWeights[k];
					}
					else if (joint2 == -1)
					{
						float_t	* offset2 = &outOffsets2Alloc[fbxJointIndices[k] * 3];
					
						joint2		= fbxJointIndex;
					
						FbxVector4 fbxOffset2;
						fbxOffset2.mData[0] = controlPoint[0];
						fbxOffset2.mData[1] = controlPoint[1];
						fbxOffset2.mData[2] = controlPoint[2];
						fbxOffset2 = fbxJointGlobalBindPoseInverse.MultT(fbxOffset2);
						offset2[0] = fbxOffset2.mData[0];
						offset2[1] = fbxOffset2.mData[1];
						offset2[2] = fbxOffset2.mData[2];
					
						weight2		= (float_t)fbxJointWeights[k];
					}
					else
					{
						continue;
					}
				}
			}
		}
	}

	//=================================================================

	static const uint32_t ExtractFbxMeshTriangles(
		FbxNode * fbxMeshNode,
		uint16_t *& outAIndicesAlloc, float_t *& outATextureUAlloc, float_t *& outATextureVAlloc,
		uint16_t *& outBIndicesAlloc, float_t *& outBTextureUAlloc, float_t *& outBTextureVAlloc,
		uint16_t *& outCIndicesAlloc, float_t *& outCTextureUAlloc, float_t *& outCTextureVAlloc,
		int8_t *& outPolyGroupsAlloc)
	{
		scmprint("Extracting Fbx mesh triangles\r\n");

		FbxMesh * fbxMesh = fbxMeshNode->GetMesh();

		uint32_t fbxTriCount = fbxMesh->GetPolygonCount();

		outAIndicesAlloc	= (uint16_t *)malloc(sizeof(uint16_t)* fbxTriCount);
		outATextureUAlloc	= (float_t *)malloc(sizeof(float_t)* fbxTriCount);
		outATextureVAlloc	= (float_t *)malloc(sizeof(float_t)* fbxTriCount);
		outBIndicesAlloc	= (uint16_t *)malloc(sizeof(uint16_t)* fbxTriCount);
		outBTextureUAlloc	= (float_t *)malloc(sizeof(float_t)* fbxTriCount);
		outBTextureVAlloc	= (float_t *)malloc(sizeof(float_t)* fbxTriCount);
		outCIndicesAlloc	= (uint16_t *)malloc(sizeof(uint16_t)* fbxTriCount);
		outCTextureUAlloc	= (float_t *)malloc(sizeof(float_t)* fbxTriCount);
		outCTextureVAlloc	= (float_t *)malloc(sizeof(float_t)* fbxTriCount);
		outPolyGroupsAlloc	= (int8_t *)malloc(sizeof(int8_t)* fbxTriCount);

		// Get UV set name
		char fbxUVSetName[256];
		memset(&fbxUVSetName[0], 0, sizeof(char)* 256);
		fbxUVSetName[0] = '\0';

		int32_t fbxLayerCount = fbxMesh->GetLayerCount();
		for (int32_t i = 0; i < fbxLayerCount; i++)
		{
			FbxLayer * fbxLayer = fbxMesh->GetLayer(i);
			int32_t fbxUVSetCount = fbxLayer->GetUVSetCount();
			FbxArray<FbxLayerElementUV const *> fbxEleUVs = fbxLayer->GetUVSets();
			for (int32_t j = 0; j < fbxUVSetCount; j++)
			{
				FbxLayerElementUV const * fbxEleUV = fbxEleUVs[j];
				if (fbxEleUV)
				{
					const char * fbxEleUVSetName = fbxEleUV->GetName();
					int32_t len = 0;
					while (fbxEleUVSetName[len] != '\0' && len < 254)
					{
						fbxUVSetName[len] = fbxEleUVSetName[len];
						len++;
					}
					fbxUVSetName[len] = '\0';
				}
			}
		}

		// Extract triangle data
		for (uint32_t i = 0; i < fbxTriCount; i++)
		{
			FbxVector2 uvA; bool unmappedA;
			if (!fbxMesh->GetPolygonVertexUV(i, 0, fbxUVSetName, uvA, unmappedA))
				uvA.mData[0] = uvA.mData[1] = 0.0;
			outAIndicesAlloc[i] = fbxMesh->GetPolygonVertex(i, 0);
			outATextureUAlloc[i] = uvA[0];
			outATextureVAlloc[i] = uvA[1];

			FbxVector2 uvB; bool unmappedB;
			if (!fbxMesh->GetPolygonVertexUV(i, 1, fbxUVSetName, uvB, unmappedB))
				uvB.mData[0] = uvB.mData[1] = 0.0;
			outBIndicesAlloc[i] = fbxMesh->GetPolygonVertex(i, 1);
			outBTextureUAlloc[i] = uvB[0];
			outBTextureVAlloc[i] = uvB[1];

			FbxVector2 uvC; bool unmappedC;
			if (!fbxMesh->GetPolygonVertexUV(i, 2, fbxUVSetName, uvC, unmappedC))
				uvC.mData[0] = uvC.mData[1] = 0.0;
			outCIndicesAlloc[i] = fbxMesh->GetPolygonVertex(i, 2);
			outCTextureUAlloc[i] = uvC[0];
			outCTextureVAlloc[i] = uvC[1];

			outPolyGroupsAlloc[i] = 0;
		}

		return fbxTriCount;
	}

	//=================================================================

	static void ExtractFbxMeshPolyGroups(
		FbxNode * fbxMeshNode,
		uint16_t outPGSkinPages[kSCM_NumPolyGroups],
		int32_t outPGFlags[kSCM_NumPolyGroups],
		int8_t outPGDamageJoints[kSCM_NumPolyGroups],
		char outPGNames[kSCM_NumPolyGroups][kSCM_NameLen])
	{
		scmprint("Extracting Fbx mesh poly groups\r\n");

		for (uint32_t i = 0; i < kSCM_NumPolyGroups; i++)
		{
			memset(&outPGSkinPages[i], 0, sizeof(uint16_t));
			memset(&outPGFlags[i], 0, sizeof(int32_t));
			memset(&outPGDamageJoints[i], 0, sizeof(int8_t));
			memset(&outPGNames[i][0], 0, sizeof(char)* kSCM_NameLen);
		}
	}

	//=================================================================

	static void ProcessFbxMeshNode(FbxNode * fbxMeshNode)
	{
		scmprint("\tProcessing Fbx Mesh Node\r\n");

		if (!CheckFbxMeshIsValid(fbxMeshNode))
			return;

		// Extract all control points (vertices)
		uint32_t scmVertCount = 0;
		float_t * scmVertPositions = 0;

		scmVertCount = ExtractFbxMeshControlPoints(fbxMeshNode, scmVertPositions);

		// Determine weights, joints and offsets
		int8_t * scmVertJoints1 = 0; float_t * scmVertOffsets1 = 0; float_t * scmVertWeights1 = 0;
		int8_t * scmVertJoints2 = 0; float_t * scmVertOffsets2 = 0; float_t * scmVertWeights2 = 0;

		ExtractFbxMeshVertexJointsOffsetsWeights(
			fbxMeshNode,
			scmVertPositions, scmVertCount,
			scmVertJoints1, scmVertOffsets1, scmVertWeights1,
			scmVertJoints2, scmVertOffsets2, scmVertWeights2);

		// Triangles
		uint32_t scmTriCount = 0;
		uint16_t * scmTriAIndices = 0; float_t * scmTriATextureU = 0; float_t * scmTriATextureV = 0;
		uint16_t * scmTriBIndices = 0; float_t * scmTriBTextureU = 0; float_t * scmTriBTextureV = 0;
		uint16_t * scmTriCIndices = 0; float_t * scmTriCTextureU = 0; float_t * scmTriCTextureV = 0;
		int8_t * scmTriPolyGroups = 0;

		scmTriCount = ExtractFbxMeshTriangles(
			fbxMeshNode,
			scmTriAIndices, scmTriATextureU, scmTriATextureV,
			scmTriBIndices, scmTriBTextureU, scmTriBTextureV,
			scmTriCIndices, scmTriCTextureU, scmTriCTextureV,
			scmTriPolyGroups);

		// PolyGroups
		uint16_t	scmPGSkinPages[kSCM_NumPolyGroups];
		int32_t		scmPGFlags[kSCM_NumPolyGroups];
		int8_t		scmPGDamageJoints[kSCM_NumPolyGroups];
		char		scmPGNames[kSCM_NumPolyGroups][kSCM_NameLen];

		ExtractFbxMeshPolyGroups(
			fbxMeshNode,
			scmPGSkinPages,
			scmPGFlags,
			scmPGDamageJoints,
			scmPGNames);

		// Pass the mesh to Scm
		ScmSerializer::AddMesh(
			fbxMeshNode->GetName(),
			scmVertCount,
			scmVertOffsets1, scmVertJoints1, scmVertWeights1,
			scmVertOffsets2, scmVertJoints2, scmVertWeights2,
			scmTriCount,
			scmTriAIndices, scmTriATextureU, scmTriATextureV,
			scmTriBIndices, scmTriBTextureU, scmTriBTextureV,
			scmTriCIndices, scmTriCTextureU, scmTriCTextureV,
			scmTriPolyGroups,
			scmPGSkinPages,
			scmPGFlags,
			scmPGDamageJoints,
			scmPGNames);

		// Release allocations
		if (scmVertPositions)	free(scmVertPositions);
		if (scmVertJoints1)		free(scmVertJoints1);
		if (scmVertOffsets1)	free(scmVertOffsets1);
		if (scmVertWeights1)	free(scmVertWeights1);
		if (scmVertJoints2)		free(scmVertJoints2);
		if (scmVertOffsets2)	free(scmVertOffsets2);
		if (scmVertWeights2)	free(scmVertWeights2);
		if (scmTriAIndices)		free(scmTriAIndices);
		if (scmTriATextureU)	free(scmTriATextureU);
		if (scmTriATextureV)	free(scmTriATextureV);
		if (scmTriBIndices)		free(scmTriBIndices);
		if (scmTriBTextureU)	free(scmTriBTextureU);
		if (scmTriBTextureV)	free(scmTriBTextureV);
		if (scmTriCIndices)		free(scmTriCIndices);
		if (scmTriCTextureU)	free(scmTriCTextureU);
		if (scmTriCTextureV)	free(scmTriCTextureV);
		if (scmTriPolyGroups)	free(scmTriPolyGroups);

	}

	//=================================================================

	void FbxProcessMesh::ProcessFbxMesh(void * scene, const char * meshFileName)
	{
		scmprint("Processing Fbx Mesh\r\n");

		fbxScene = (FbxScene *)scene;

		// Update export file name
		ScmSerializer::SetExportPath(meshFileName);

		// Order child nodes so we process Skeletons first
		std::deque<FbxNode *> nodeQueue;
		
		FbxNode * fbxRootNode = fbxScene->GetRootNode();
		int32_t numChildren = fbxRootNode->GetChildCount();
		for (int32_t i = 0; i < numChildren; i++)
		{
			FbxNode * fbxChildNode = fbxRootNode->GetChild(i);
			FbxNodeAttribute::EType fbxAttributeType;
			fbxAttributeType = fbxChildNode->GetNodeAttribute()->GetAttributeType();
		
			switch (fbxAttributeType)
			{
			case FbxNodeAttribute::eMesh:
				nodeQueue.push_back(fbxChildNode);
				break;
		
			case FbxNodeAttribute::eSkeleton:
				nodeQueue.push_front(fbxChildNode);
				break;
			}
		}

		// Now process all children in order
		for (int32_t i = 0; i < nodeQueue.size(); i++)
		{
			FbxNodeAttribute::EType fbxAttributeType;
			fbxAttributeType = nodeQueue[i]->GetNodeAttribute()->GetAttributeType();
		
			switch (fbxAttributeType)
			{
			case FbxNodeAttribute::eMesh:
				ProcessFbxMeshNode(nodeQueue[i]);
				break;
		
			case FbxNodeAttribute::eSkeleton:
				ProcessFbxSkeletonNode(nodeQueue[i]);
				break;
			}
		}

		fbxScene = 0;
	}
}