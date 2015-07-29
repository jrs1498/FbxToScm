#include <fbxsdk.h>

#include "../include/scmLog.h"
#include "../include/scmScmSerializer.h"
#include "../include/scmFbxProcessSequence.h"

namespace scm
{
	static FbxScene * fbxScene;

	//=================================================================

	static void ProcessFbxSkeletonNodeRecursive(FbxNode * fbxSkeletonNode)
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
		FbxTime fbxEnd = fbxTakeInfo->mLocalTimeSpan.GetStop();

		FbxLongLong fbxAnimEnd = fbxEnd.GetFrameCount(FbxTime::eFrames24);
		FbxLongLong fbxAnimStart = fbxStart.GetFrameCount(FbxTime::eFrames24);
		FbxLongLong fbxAnimLength = fbxAnimEnd - fbxAnimStart + 1;

		// Extract joint transformation for each frame
		for (FbxLongLong i = fbxAnimStart; i < fbxAnimEnd; i++)
		{
			FbxTime fbxCurrTime;
			fbxCurrTime.SetFrame(i, FbxTime::eFrames24);

			// Get frame transformation and account for joint orientation
			FbxAMatrix fbxFrameTransform = fbxSkeletonNode->EvaluateLocalTransform(fbxCurrTime);

			FbxVector4 fbxFrameTranslation	= fbxFrameTransform.GetT();
			FbxVector4 fbxFrameScaling		= fbxFrameTransform.GetS();
			FbxVector4 fbxFrameRotation		= fbxFrameTransform.GetR();

			// Add this frame for this joint
			ScmSerializer::SequenceAddJointFrame(
				fbxSkeletonNode->GetName(),
				fbxFrameTranslation.Buffer()[0],
				fbxFrameTranslation.Buffer()[1],
				fbxFrameTranslation.Buffer()[2],
				fbxFrameScaling.Buffer()[0],
				fbxFrameScaling.Buffer()[1],
				fbxFrameScaling.Buffer()[2],
				fbxFrameRotation.Buffer()[0],
				fbxFrameRotation.Buffer()[1],
				fbxFrameRotation.Buffer()[2]);
		}

		// Recurse over all child joints
		int32_t childCount = fbxSkeletonNode->GetChildCount();
		for (int32_t i = 0; i < childCount; i++)
		{
			ProcessFbxSkeletonNodeRecursive(fbxSkeletonNode->GetChild(i));
		}
	}

	//=================================================================

	static void ProcessFbxSkeletonNode(FbxNode * fbxSkeletonNode)
	{
		scmprint("\tProcessing Fbx Skeleton Node\r\n");

		ProcessFbxSkeletonNodeRecursive(fbxSkeletonNode);
	}

	//=================================================================

	void FbxProcessSequence::ProcessFbxSequence(void * scene, const char * sequenceFileName)
	{
		scmprint("Processing Fbx Sequence\r\n");

		fbxScene = (FbxScene *)scene;

		FbxNode * fbxRootNode = fbxScene->GetRootNode();
		int32_t numChildren = fbxRootNode->GetChildCount();

		// Get the frame rate
		double_t fbxFrameRate = 10.0;

		char sequenceNameBuffer[256];
		
		if (isdigit(sequenceFileName[0]))
		{
			char frameRateBuffer[32];
			int32_t crsr1 = 0;
			int32_t crsr2 = 0;

			while (isdigit(sequenceFileName[crsr1]) && crsr1 < 30)
			{
				frameRateBuffer[crsr1] = sequenceFileName[crsr1];
				crsr1++;
			}
			frameRateBuffer[crsr1] = '\0';

			while (sequenceFileName[crsr1] != '\0')
			{
				sequenceNameBuffer[crsr2] = sequenceFileName[crsr1];
				crsr1++;
				crsr2++;
			}
			sequenceNameBuffer[crsr2] = '\0';

			fbxFrameRate = atof(frameRateBuffer);
			fbxFrameRate = fbxFrameRate < 0 ? 10.0 : fbxFrameRate;
			fbxFrameRate = fbxFrameRate > 1000.0 ? 1000.0 : fbxFrameRate;
		}
		else
		{
			strcpy_s(sequenceNameBuffer, sequenceFileName);
		}

		for (int32_t i = 0; i < numChildren; i++)
		{
			FbxNode * fbxChildNode = fbxRootNode->GetChild(i);
			FbxNodeAttribute::EType fbxAttributeType;
			fbxAttributeType = fbxChildNode->GetNodeAttribute()->GetAttributeType();

			// Sequences are only concerned with joint transformations
			if (fbxAttributeType != FbxNodeAttribute::eSkeleton)
				continue;

			ScmSerializer::SequenceBegin(sequenceNameBuffer, fbxFrameRate);
			ProcessFbxSkeletonNode(fbxChildNode);
			ScmSerializer::SequenceEnd();
		}

		fbxScene = 0;
	}
}