#include <vector>
#include <string>
#include <iostream>
#include "../include/scmPreReq.h"
#include "../include/scmLog.h"
#include "../include/scmUtility.h"
#include "../include/scmScmSerializer.h"

namespace scm
{
	static char scmExportPath[kSCM_LongNameLen];
	static char scmResourcePath[kSCM_LongNameLen];
	static int32_t scmModelType;

	typedef std::vector<scmskin_t> scmskinarray_t;
	typedef std::vector<uint8_t *> scmskinbmparray_t;
	static scmskinarray_t scmSkins;
	static scmskinbmparray_t scmSkinBmps;

	typedef std::vector<scmmeshname_t> scmmeshnamearray_t;
	typedef std::vector<scmtriangle_t> scmtrianglearray_t;
	typedef std::vector<scmtrianglearray_t> scmtriangle2Darray_t;
	typedef std::vector<scmvertex_t> scmvertexarray_t;
	typedef std::vector<scmvertexarray_t> scmvertex2Darray_t;
	typedef std::vector<scmpolygroup_t> scmpolygrouparray_t;
	typedef std::vector<scmpolygrouparray_t> scmpolygroup2Darray_t;
	static scmmeshnamearray_t scmMeshNames;
	static scmtriangle2Darray_t scmMeshTriangles;
	static scmvertex2Darray_t scmMeshVertices;
	static scmpolygroup2Darray_t scmMeshPolyGroups;

	typedef std::vector<scmjoint_t> scmjointarray_t;
	static scmjointarray_t scmJoints;

	typedef std::vector<scmevent_t> scmeventarray_t;
	static scmeventarray_t scmEvents;

	typedef std::vector<scmsequence_t> scmsequencearray_t;
	typedef std::vector<scmseqdata_t> scmseqdataarray_t;
	typedef std::vector<scmseqdataarray_t> scmseqdata2Darray_t;
	typedef std::vector<int8_t> scmseqcompressedarray_t;
	typedef std::vector<scmseqcompressedarray_t> scmseqcompressed2Darray_t;
	typedef std::vector<scmframe_t> scmframearray_t;
	static scmsequencearray_t scmSequences;
	static scmseqdata2Darray_t scmSequenceJointFrames;
	static scmseqcompressed2Darray_t scmSequencesCompressed;
	static int8_t scmSequenceActive;
	static scmframearray_t scmFrames;

	static FVECTOR scmCumulativeScale;

	//=================================================================

	const int32_t ConvertCStrToScmStr(const char * str, char * outBuffer, int32_t bufferLen)
	{
		memset(&outBuffer[0], 0, sizeof(char)* bufferLen);
		int32_t len = 0;
		while (str[len] != '\0' && len < bufferLen - 1)
		{
			outBuffer[len] = str[len];
			len++;
		}
		outBuffer[len] = '\0';
		return len;
	}

	//=================================================================

	void ScmSerializer::Initialize(void)
	{
		// Header
		memcpy(&scmExportPath[0], &kSCM_DefaultExportFile[0], sizeof(char)* kSCM_LongNameLen);
		memcpy(&scmResourcePath[0], &kSCM_DefaultResourceFile[0], sizeof(char)* kSCM_LongNameLen);
		scmModelType = MODELTYPE_NORMAL;

		// Skins
		scmSkins.clear();
		scmSkinBmps.clear();

		// Meshes
		scmMeshNames.clear();
		for (int32_t i = 0; i < scmMeshTriangles.size(); i++)
			scmMeshTriangles[i].clear();
		scmMeshTriangles.clear();
		for (int32_t i = 0; i < scmMeshVertices.size(); i++)
			scmMeshVertices[i].clear();
		scmMeshVertices.clear();
		for (int32_t i = 0; i < scmMeshPolyGroups.size(); i++)
			scmMeshPolyGroups[i].clear();
		scmMeshPolyGroups.clear();
		
		// Joints
		scmJoints.clear();

		// Events
		scmEvents.clear();
		AddEvent("None");

		// Sequences
		scmSequences.clear();
		for (int32_t i = 0; i < scmSequenceJointFrames.size(); i++)
			scmSequenceJointFrames[i].clear();
		scmSequenceJointFrames.clear();
		for (int32_t i = 0; i < scmSequencesCompressed.size(); i++)
			scmSequencesCompressed[i].clear();
		scmSequencesCompressed.clear();
		scmSequenceActive = 0;
		scmFrames.clear();

		// Cumulative scale
		scmCumulativeScale.x = 1.0;
		scmCumulativeScale.y = 1.0;
		scmCumulativeScale.z = 1.0;
	}

	//=================================================================

	void ScmSerializer::SetExportPath(char path[kSCM_LongNameLen])
	{
		memcpy(&scmExportPath[0], &path[0], sizeof(uint8_t)* kSCM_LongNameLen);
	}

	void ScmSerializer::SetExportPath(const char * path)
	{
		memset(&scmExportPath[0], 0, sizeof(char)* kSCM_LongNameLen);
		int32_t len = 0;
		while (path[len] != '\0' && len < kSCM_LongNameLen - 1)
		{
			scmExportPath[len] = path[len];
			len++;
		}
		scmExportPath[len] = '\0';
	}

	//=================================================================

	void ScmSerializer::SetResourcePath(char path[kSCM_LongNameLen])
	{
		memcpy(&scmResourcePath[0], &path[0], sizeof(uint8_t)* kSCM_LongNameLen);
	}

	void ScmSerializer::SetResourcePath(const char * path)
	{
		memset(&scmResourcePath[0], 0, sizeof(char)* kSCM_LongNameLen);
		int32_t len = 0;
		while (path[len] != '\0' && len < kSCM_LongNameLen - 1)
		{
			scmResourcePath[len] = path[len];
			len++;
		}
		scmResourcePath[len] = '\0';
	}

	//=================================================================

	void ScmSerializer::AddSkin(
		char name[kSCM_NameLen],
		uint8_t palette[768],
		int32_t bmpWidth,
		int32_t bmpHeight,
		uint8_t * bmpData)
	{
		scmskin_t skin;
		memcpy(&skin.SkinName[0], &name[0], sizeof(uint8_t)* kSCM_NameLen);
		memcpy(&skin.palette[0], &palette[0], sizeof(uint8_t)* 768);
		skin.BitmapWidth = bmpWidth;
		skin.BitmapHeight = bmpHeight;
		skin.Bitmap[0] = 0;

		uint8_t * skinBmpData = (uint8_t *)malloc(sizeof(uint8_t)* bmpWidth * bmpHeight);
		memcpy(&skinBmpData[0], &bmpData[0], sizeof(uint8_t)* bmpWidth * bmpHeight);

		scmSkins.push_back(skin);
		scmSkinBmps.push_back(skinBmpData);
	}

	//=================================================================

	const int32_t ScmSerializer::GetNumSkins(void)
	{
		return scmSkins.size();
	}

	//=================================================================

	void ScmSerializer::AddMesh(
		const char * name,
		uint32_t numVertices,
		float_t * vertOffsets1, int8_t * vertJoints1, float_t * vertWeights1,
		float_t * vertOffsets2, int8_t * vertJoints2, float_t * vertWeights2,
		uint32_t numTriangles,
		uint16_t * triAVertices, float_t * triATextureUs, float_t * triATextureVs,
		uint16_t * triBVertices, float_t * triBTextureUs, float_t * triBTextureVs,
		uint16_t * triCVertices, float_t * triCTextureUs, float_t * triCTextureVs,
		int8_t * trianglePolyGroups,
		uint16_t skinPages[kSCM_NumPolyGroups],
		int32_t polyGroupFlags[kSCM_NumPolyGroups],
		int8_t damageJoints[kSCM_NumPolyGroups],
		char groupNames[kSCM_NumPolyGroups][kSCM_NameLen])
	{
		scmmeshname_t meshName;
		scmvertexarray_t vertices;
		scmtrianglearray_t triangles;
		scmpolygrouparray_t polygroups;

		// Name
		char scmName[kSCM_NameLen];
		ConvertCStrToScmStr(&name[0], &scmName[0], kSCM_NameLen);

		memcpy(&meshName.MeshName[0], &scmName[0], sizeof(char)* kSCM_NameLen);
		scmMeshNames.push_back(meshName);

		// Offsets, Joints, Weights
		for (uint32_t i = 0; i < numVertices; i++)
		{
			scmvertex_t vertex;
			int32_t indexv = i * 3;

			float_t * offset1 = &vertOffsets1[indexv];
			vertex.VertOffset1.x = offset1[0];
			vertex.VertOffset1.y = offset1[1];
			vertex.VertOffset1.z = offset1[2];
			vertex.WeightJoint1 = vertJoints1[i];
			vertex.WeightFactor1 = Utility::NormalizedFloatToUnsignedByte(vertWeights1[i]);
			//vertex.WeightFactor1 = vertWeights[i];
			//vertex.WeightFactor1 = -1;

			float_t * offset2 = &vertOffsets2[indexv];
			vertex.VertOffset2.x = offset2[0];
			vertex.VertOffset2.y = offset2[1];
			vertex.VertOffset2.z = offset2[2];
			vertex.WeightJoint2 = vertJoints2[i];
			vertex.WeightFactor2 = Utility::NormalizedFloatToUnsignedByte(vertWeights2[i]);

			vertices.push_back(vertex);
		}
		scmMeshVertices.push_back(vertices);

		// Triangles
		for (uint32_t i = 0; i < numTriangles; i++)
		{
			scmtriangle_t triangle;

			triangle.A		= triAVertices[i];
			triangle.A_S	= Utility::NormalizedFloatToUnsignedByte(triATextureUs[i]);
			triangle.A_T	= Utility::NormalizedFloatToUnsignedByte(1.0 - triATextureVs[i]);


			triangle.B		= triBVertices[i];
			triangle.B_S	= Utility::NormalizedFloatToUnsignedByte(triBTextureUs[i]);
			triangle.B_T	= Utility::NormalizedFloatToUnsignedByte(1.0 - triBTextureVs[i]);

			triangle.C		= triCVertices[i];
			triangle.C_S	= Utility::NormalizedFloatToUnsignedByte(triCTextureUs[i]);
			triangle.C_T	= Utility::NormalizedFloatToUnsignedByte(1.0 - triCTextureVs[i]);


			triangle.polygroup = trianglePolyGroups[i];

			triangles.push_back(triangle);
		}
		scmMeshTriangles.push_back(triangles);

		// Polygroups
		for (uint32_t i = 0; i < kSCM_NumPolyGroups; i++)
		{
			scmpolygroup_t polygroup;

			polygroup.SkinPage = skinPages[i];
			polygroup.PolyGroupFlags = polyGroupFlags[i];
			polygroup.DamageJoint = damageJoints[i];
			memcpy(&polygroup.GroupName[0], &groupNames[i][0], sizeof(char)* kSCM_NameLen);

			polygroups.push_back(polygroup);
		}
		scmMeshPolyGroups.push_back(polygroups);
	}

	//=================================================================

	const int32_t ScmSerializer::GetNumMeshes(void)
	{
		return scmMeshNames.size();
	}

	const int32_t ScmSerializer::GetNumMeshTriangles(int32_t i)
	{
		return scmMeshTriangles[i].size();
	}

	const int32_t ScmSerializer::GetNumMeshVertices(int32_t i)
	{
		return scmMeshVertices[i].size();
	}

	//=================================================================

	void ScmSerializer::AddJoint(
		char name[kSCM_NameLen],
		int32_t parent,
		int32_t children[kSCM_MaxChildJoints],
		int32_t jointGroup,
		int32_t flags,
		float_t collisionBox[24])
	{
		scmprint("Adding joint: %s\r\n", name);

		// Add this joint
		scmjoint_t joint;

		memcpy(&joint.JointName[0], &name[0], sizeof(char)* kSCM_NameLen);
		memcpy(&joint.Parent, &parent, sizeof(int32_t));
		memcpy(&joint.Children[0], &children[0], sizeof(int32_t)* kSCM_MaxChildJoints);
		memcpy(&joint.JointGroup, &jointGroup, sizeof(int32_t));
		memcpy(&joint.flags, &flags, sizeof(int32_t));
		memcpy(&joint.CollisionBox[0], &collisionBox[0], sizeof(float_t)* 4);
		memcpy(&joint.CollisionBox[1], &collisionBox[4], sizeof(float_t)* 4);
		memcpy(&joint.CollisionBox[2], &collisionBox[8], sizeof(float_t)* 4);
		memcpy(&joint.CollisionBox[3], &collisionBox[12], sizeof(float_t)* 4);
		memcpy(&joint.CollisionBox[4], &collisionBox[16], sizeof(float_t)* 4);
		memcpy(&joint.CollisionBox[5], &collisionBox[20], sizeof(float_t)* 4);

		scmJoints.push_back(joint);

		// Add joint frame data array
		scmseqdataarray_t jointFrameData;
		scmSequenceJointFrames.push_back(jointFrameData);
	}

	//=================================================================

	void ScmSerializer::BuildSkeletonChildren(void)
	{
		scmprint("Building skeleton children\r\n");

		for (int32_t i = 0; i < GetNumJoints(); i++)
		{
			scmjoint_t & joint = scmJoints[i];
			if (joint.Parent < 0)
				continue;
			scmjoint_t & parent = scmJoints[joint.Parent];
			for (int32_t j = 0; j < kSCM_MaxChildJoints; j++)
			{
				if (parent.Children[j] == -1)
				{
					parent.Children[j] = i;
					break;
				}
			}
		}
	}

	//=================================================================

	const int32_t ScmSerializer::GetJointIndexFromName(const char * jointName)
	{
		for (int32_t i = 0; i < GetNumJoints(); i++)
		{
			if (!strcmp(jointName, scmJoints[i].JointName))
				return i;
		}
		return -1;
	}

	const int32_t ScmSerializer::GetNumJoints(void)
	{
		return scmJoints.size();
	}

	//=================================================================

	void ScmSerializer::AddEvent(char name[kSCM_MediumNameLen])
	{
		scmevent_t event;

		memcpy(&event.EventName[0], &name[0], sizeof(char)* kSCM_MediumNameLen);

		scmEvents.push_back(event);
	}

	//=================================================================

	const int32_t ScmSerializer::GetNumEvents(void)
	{
		return scmEvents.size();
	}

	//=================================================================

	void ScmSerializer::SequenceBegin(const char * name, float_t linearVel)
	{
		// Check if there's already an active sequence
		if (scmSequenceActive) {
			scmprint("SequenceEnd must be called before SequenceBegin\r\n");
			return;
		}
		// Convert name
		char scmName[kSCM_NameLen];
		ConvertCStrToScmStr(&name[0], &scmName[0], kSCM_NameLen);

		scmprint("----- SequenceBegin: %s -----\r\n", scmName);
		scmSequenceActive = 1;

		// Push a new sequence
		scmsequence_t sequence;
		memcpy(&sequence.SequenceName[0], &scmName[0], sizeof(char)* kSCM_NameLen);
		memset(&sequence.FirstFrame, 0, sizeof(int32_t));
		memset(&sequence.NumFrames, 0, sizeof(int32_t));
		memcpy(&sequence.LinearVel, &linearVel, sizeof(float_t));
		memset(&sequence.bNormalized, 0, sizeof(int8_t));
		memset(&sequence.CompressedLength, 0, sizeof(int32_t));
		memset(&sequence.Compressed[0], 0, sizeof(int8_t));
		scmSequences.push_back(sequence);

		// Push a new compressed sequence array
		scmseqcompressedarray_t sequenceCompressed;
		scmSequencesCompressed.push_back(sequenceCompressed);

		// Clear out all joint frame data
		for (uint32_t i = 0; i < GetNumJoints(); i++)
		{
			scmSequenceJointFrames[i].clear();
		}
	}

	//=================================================================

	void ScmSerializer::SequenceAddJointFrame(
		const char * jointName,
		float_t posX, float_t posY, float_t posZ,
		float_t sclX, float_t sclY, float_t sclZ,
		float_t rotX, float_t rotY, float_t rotZ)
	{
		// Make sure there's a sequence active
		if (!scmSequenceActive) {
			scmprint("SequenceBegin must be called before SequenceAddJointFrame\r\n");
			return;
		}
		scmprint("SequenceAddJointFrame: %s\r\n", jointName);

		// Get this joint's index and make sure it's valid
		int32_t jointIndex = GetJointIndexFromName(jointName);
		if (jointIndex == -1) {
			scmprint("\t ERROR: Coult not locate joint by name\r\n");
			return;
		}
		scmprint("\tJoint index: %d\r\n", jointIndex);

		// Get this joint's frame array and push this frame data
		scmseqdataarray_t & seqFrames = scmSequenceJointFrames[jointIndex];
		scmseqdata_t seqData;
		seqData.pos.x = posX;
		seqData.pos.y = posY;
		seqData.pos.z = posZ;
		seqData.scale.x = sclX;
		seqData.scale.y = sclY;
		seqData.scale.z = sclZ;
		seqData.rot.x = rotX;
		seqData.rot.y = rotY;
		seqData.rot.z = rotZ;
		seqFrames.push_back(seqData);
	}

	//=================================================================

	static void CompressFloatToSequence(float_t val, scmseqcompressedarray_t & seqCompressed)
	{
		int16_t compressed = Utility::FloatToEightEight(val);
		seqCompressed.push_back(0x0);
		seqCompressed.push_back(0x0);
		memcpy(&seqCompressed[seqCompressed.size() - 2], &compressed, sizeof(int16_t));
	}

	void ScmSerializer::SequenceEnd(void)
	{
		// Make sure there's a sequence to be ended
		if (!scmSequenceActive) {
			scmprint("SequenceBegin must be called before SequenceEnd\r\n");
			return;
		}
		scmprint("----- SequenceEnd -----\r\n");

		// Get frame count by checking the frame counts of every joint
		int32_t numFrames = scmSequenceJointFrames[0].size();
		for (int32_t i = 1; i < GetNumJoints(); i++)
		{
			int32_t thisJointNumFrames = scmSequenceJointFrames[i].size();

			// Trash this sequence if joint frame counts mismatch
			if (thisJointNumFrames != numFrames) {
				scmprint("\tERROR: numFrames mismatch: %s\r\n", scmJoints[i].JointName);
				SequenceAbort();
				return;
			}
		}

		// Trash this sequence if frame count is 0
		if (numFrames <= 0) {
			scmprint("\tERROR: numFrames is %d\r\n", numFrames);
			SequenceAbort();
			return;
		}

		// Grab pointers to what we need to fill out
		int32_t sequenceIndex = GetNumSequences() - 1;
		scmsequence_t & sequence = scmSequences[sequenceIndex];
		scmseqcompressedarray_t & seqCompressed = scmSequencesCompressed[sequenceIndex];

		sequence.FirstFrame = GetNumFrames();
		sequence.NumFrames = numFrames;

		// Compress joint frame data
		for (int32_t i = 0; i < numFrames; i++)
		{
			for (int32_t j = 0; j < GetNumJoints(); j++)
			{
				scmseqdata_t & seqData = scmSequenceJointFrames[j][i];

				// Mask
				seqCompressed.push_back(0x00);
				int32_t maskIndex = seqCompressed.size() - 1;

				#define ENABLE_ANIM_BIT(x) seqCompressed[maskIndex] |= x

				// Position
				if (seqData.pos.x != 0.0) {
					ENABLE_ANIM_BIT(ANIMMASK_POSX);
					CompressFloatToSequence(seqData.pos.x, seqCompressed);
				}
				if (seqData.pos.y != 0.0) {
					ENABLE_ANIM_BIT(ANIMMASK_POSY);
					CompressFloatToSequence(seqData.pos.y, seqCompressed);
				}
				if (seqData.pos.z != 0.0) {
					ENABLE_ANIM_BIT(ANIMMASK_POSZ);
					CompressFloatToSequence(seqData.pos.z, seqCompressed);
				}

				// Scale
				if (seqData.scale.x != 1.0 || seqData.scale.z != 1.0) {
					ENABLE_ANIM_BIT(ANIMMASK_SCALEXZ);
					CompressFloatToSequence(seqData.scale.x, seqCompressed);
					CompressFloatToSequence(seqData.scale.z, seqCompressed);
				}
				if (seqData.scale.y != 1.0) {
					ENABLE_ANIM_BIT(ANIMMASK_SCALEY);
					CompressFloatToSequence(seqData.scale.y, seqCompressed);
				}

				// Rotations assumed to be in degrees
				if (seqData.rot.y != 0.0) {
					ENABLE_ANIM_BIT(ANIMMASK_PITCH);
					float_t nroty = Utility::DegreesToNormalizedRotation(seqData.rot.y);
					nroty *= 256.0;
					CompressFloatToSequence(nroty, seqCompressed);
				}
				if (seqData.rot.z != 0.0) {
					ENABLE_ANIM_BIT(ANIMMASK_YAW);
					float_t nrotz = Utility::DegreesToNormalizedRotation(seqData.rot.z);
					nrotz *= 256.0;
					CompressFloatToSequence(nrotz, seqCompressed);
				}
				if (seqData.rot.x != 0.0) {
					ENABLE_ANIM_BIT(ANIMMASK_ROLL);
					float_t nrotx = Utility::DegreesToNormalizedRotation(seqData.rot.x);
					nrotx *= 256.0;
					CompressFloatToSequence(nrotx, seqCompressed);
				}

				#undef ENABLE_ANIM_BIT
			}

			// Add a dummy frame for now
			// Not entirely sure what a frame does, but here are observations:
			//	-	Model will not render if frame's bounding box is not in player's view
			scmframe_t frame;
			frame.SequenceIndex = sequenceIndex;
			frame.eventID = 0;
			frame.bounds.ulc.x = -32.0;
			frame.bounds.ulc.y = -32.0;
			frame.bounds.ulc.z = -32.0;
			frame.bounds.lrc.x = 32.0;
			frame.bounds.lrc.y = 32.0;
			frame.bounds.lrc.z = 32.0;
			frame.bounds.i = 1;
			scmFrames.push_back(frame);
		}

		sequence.CompressedLength = seqCompressed.size();

		scmSequenceActive = 0;
	}

	//=================================================================

	void ScmSerializer::SequenceAbort(void)
	{
		// Can only abort a sequence if one is active
		if (!scmSequenceActive) {
			scmprint("SequenceAbort must be called while a sequence is active\r\n");
			return;
		}
		scmprint("!!!!! SequenceAbort !!!!!\r\n");

		// Remove the most recently added sequence
		scmSequences.pop_back();

		// Remove the most recently added compressed sequence data
		scmSequencesCompressed.pop_back();

		scmSequenceActive = 0;
	}

	//=================================================================

	const int32_t ScmSerializer::GetNumSequences(void)
	{
		return scmSequences.size();
	}

	const int32_t ScmSerializer::GetNumFrames(void)
	{
		return scmFrames.size();
	}

	//=================================================================

	void ScmSerializer::SerializeAndExport(void)
	{
		FILE * pFile;
		fopen_s(&pFile, scmExportPath, "wb");

		// SCM Header -------------------------------------------------
		scmheader_t header;
		memset(&header, 0, sizeof(scmheader_t));
		header.MagicNumber[0] = 'S';
		header.MagicNumber[1] = 'C';
		header.MagicNumber[2] = 'M';
		header.MagicNumber[3] = '\0';
		header.FileVersion = kSCM_Version;
		header.ModelType = scmModelType;
		memcpy(&header.ExportPath[0], &scmExportPath[0], sizeof(uint8_t)* kSCM_LongNameLen);
		memcpy(&header.ResourcePath[0], &scmResourcePath[0], sizeof(uint8_t)* kSCM_LongNameLen);

		fwrite(&header, sizeof(scmheader_t), 1, pFile);

		// Skins ------------------------------------------------------
		scmskinhdr_t skinHeader;
		skinHeader.NumberSkins = GetNumSkins();

		fwrite(&skinHeader.NumberSkins, sizeof(uint32_t), 1, pFile);

		for (uint32_t i = 0; i < GetNumSkins(); i++)
		{
			scmskin_t & skin = scmSkins[i];
			uint8_t * skinBmp = scmSkinBmps[i];

			fwrite(&skin.SkinName[0], sizeof(uint8_t)* kSCM_NameLen, 1, pFile);
			fwrite(&skin.palette[0], sizeof(uint8_t)* 768, 1, pFile);
			fwrite(&skin.BitmapWidth, sizeof(int32_t), 1, pFile);
			fwrite(&skin.BitmapHeight, sizeof(int32_t), 1, pFile);
			fwrite(&skinBmp[0], sizeof(uint8_t)* skin.BitmapWidth * skin.BitmapHeight, 1, pFile);
		}

		// Meshes -----------------------------------------------------
		scmmeshhdr_t meshHeader;
		meshHeader.NumberMeshes = GetNumMeshes();

		fwrite(&meshHeader.NumberMeshes, sizeof(uint32_t), 1, pFile);

		for (uint32_t i = 0; i < GetNumMeshes(); i++)
		{
			scmmeshname_t & meshName = scmMeshNames[i];
			scmtrianglearray_t & meshTriangles = scmMeshTriangles[i];
			scmvertexarray_t & meshVertices = scmMeshVertices[i];
			scmpolygrouparray_t & meshPolyGroups = scmMeshPolyGroups[i];

			fwrite(&meshName.MeshName[0], sizeof(uint8_t)* kSCM_NameLen, 1, pFile);

			scmtrianglehdr_t triangleHeader;
			triangleHeader.NumberTriangles = GetNumMeshTriangles(i);

			fwrite(&triangleHeader.NumberTriangles, sizeof(uint32_t), 1, pFile);

			for (uint32_t j = 0; j < GetNumMeshTriangles(i); j++)
			{
				scmtriangle_t & triangle = meshTriangles[j];

				fwrite(&triangle.A, sizeof(uint16_t), 1, pFile);
				fwrite(&triangle.A_S, sizeof(int8_t), 1, pFile);
				fwrite(&triangle.A_T, sizeof(int8_t), 1, pFile);

				fwrite(&triangle.B, sizeof(uint16_t), 1, pFile);
				fwrite(&triangle.B_S, sizeof(int8_t), 1, pFile);
				fwrite(&triangle.B_T, sizeof(int8_t), 1, pFile);

				fwrite(&triangle.C, sizeof(uint16_t), 1, pFile);
				fwrite(&triangle.C_S, sizeof(int8_t), 1, pFile);
				fwrite(&triangle.C_T, sizeof(int8_t), 1, pFile);

				fwrite(&triangle.polygroup, sizeof(int8_t), 1, pFile);
			}

			scmvertexhdr_t vertexHeader;
			vertexHeader.NumberVertices = GetNumMeshVertices(i);

			fwrite(&vertexHeader.NumberVertices, sizeof(uint32_t), 1, pFile);

			for (uint32_t j = 0; j < GetNumMeshVertices(i); j++)
			{
				scmvertex_t & vertex = meshVertices[j];

				fwrite(&vertex.VertOffset1, sizeof(float_t)* 3, 1, pFile);
				fwrite(&vertex.WeightJoint1, sizeof(int8_t), 1, pFile);
				fwrite(&vertex.WeightFactor1, sizeof(int8_t), 1, pFile);

				fwrite(&vertex.VertOffset2, sizeof(float_t)* 3, 1, pFile);
				fwrite(&vertex.WeightJoint2, sizeof(int8_t), 1, pFile);
				fwrite(&vertex.WeightFactor2, sizeof(int8_t), 1, pFile);
			}

			for (uint32_t j = 0; j < kSCM_NumPolyGroups; j++)
			{
				scmpolygroup_t & polyGroup = meshPolyGroups[j];

				fwrite(&polyGroup.SkinPage, sizeof(uint16_t), 1, pFile);
				fwrite(&polyGroup.PolyGroupFlags, sizeof(int32_t), 1, pFile);
				fwrite(&polyGroup.DamageJoint, sizeof(int8_t), 1, pFile);
				fwrite(&polyGroup.GroupName[0], sizeof(char)* kSCM_NameLen, 1, pFile);
			}
		}

		// Joints -----------------------------------------------------
		scmjointhdr_t jointHeader;
		jointHeader.NumberJoints = GetNumJoints();

		fwrite(&jointHeader.NumberJoints, sizeof(uint32_t), 1, pFile);

		for (uint32_t i = 0; i < GetNumJoints(); i++)
		{
			scmjoint_t & joint = scmJoints[i];

			fwrite(&joint.JointName[0], sizeof(char)* kSCM_NameLen, 1, pFile);
			fwrite(&joint.Parent, sizeof(int32_t), 1, pFile);
			fwrite(&joint.Children[0], sizeof(int32_t)* kSCM_MaxChildJoints, 1, pFile);
			fwrite(&joint.JointGroup, sizeof(int32_t), 1, pFile);
			fwrite(&joint.flags, sizeof(int32_t), 1, pFile);

			for (uint32_t j = 0; j < 6; j++)
			{
				float_t & collisionBox = joint.CollisionBox[j].normal.x;

				fwrite(&collisionBox, sizeof(float_t)* 4, 1, pFile);
			}
		}

		// Events -----------------------------------------------------
		scmeventhdr_t eventHeader;
		eventHeader.NumberEvents = GetNumEvents();

		fwrite(&eventHeader.NumberEvents, sizeof(uint32_t), 1, pFile);

		for (uint32_t i = 0; i < GetNumEvents(); i++)
		{
			scmevent_t & event = scmEvents[i];

			fwrite(&event.EventName[0], sizeof(char)* kSCM_MediumNameLen, 1, pFile);
		}

		// Sequences --------------------------------------------------
		scmsequencehdr_t sequenceHeader;
		sequenceHeader.NumberSequences = GetNumSequences();

		fwrite(&sequenceHeader.NumberSequences, sizeof(uint32_t), 1, pFile);

		for (uint32_t i = 0; i < GetNumSequences(); i++)
		{
			scmsequence_t & sequence = scmSequences[i];

			fwrite(&sequence.SequenceName[0], sizeof(char)* kSCM_NameLen, 1, pFile);
			fwrite(&sequence.FirstFrame, sizeof(int32_t), 1, pFile);
			fwrite(&sequence.NumFrames, sizeof(int32_t), 1, pFile);
			fwrite(&sequence.LinearVel, sizeof(float_t), 1, pFile);
			fwrite(&sequence.bNormalized, sizeof(int8_t), 1, pFile);
			fwrite(&sequence.CompressedLength, sizeof(int32_t), 1, pFile);

			scmseqcompressedarray_t & sequenceCompressed = scmSequencesCompressed[i];

			fwrite(&sequenceCompressed[0], sizeof(int8_t)* sequence.CompressedLength, 1, pFile);
		}

		// Frames -----------------------------------------------------
		scmframehdr_t frameHeader;
		frameHeader.NumberFrames = GetNumFrames();

		fwrite(&frameHeader.NumberFrames, sizeof(uint32_t), 1, pFile);

		for (uint32_t i = 0; i < GetNumFrames(); i++)
		{
			scmframe_t & frame = scmFrames[i];

			fwrite(&frame.SequenceIndex, sizeof(int16_t), 1, pFile);
			fwrite(&frame.eventID, sizeof(int16_t), 1, pFile);
			fwrite(&frame.bounds.ulc.x, sizeof(float_t)* 3, 1, pFile);
			fwrite(&frame.bounds.lrc.x, sizeof(float_t)* 3, 1, pFile);
			fwrite(&frame.bounds.i, sizeof(int32_t), 1, pFile);
		}

		// Cumulative Scale -------------------------------------------
		fwrite(&scmCumulativeScale.x, sizeof(float_t)* 3, 1, pFile);

		// Decimation -------------------------------------------------
		for (uint32_t i = 0; i < GetNumMeshes(); i++)
		{
			int32_t decimationCount = 0;
			int32_t decimationChunkSize = 0;

			fwrite(&decimationCount, sizeof(int32_t), 1, pFile);
			fwrite(&decimationChunkSize, sizeof(int32_t), 1, pFile);
		}

		fclose(pFile);
	}
}