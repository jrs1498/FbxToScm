#ifndef SCMSCMSERIALIZER_H
#define SCMSCMSERIALIZER_H

#include "../include/scmPreReq.h"
#include "../include/SuperCoolModel.h"

#define kSCM_DefaultExportFile "scmoutput.scm"
#define kSCM_DefaultResourceFile "scmresource.res"

namespace scm
{
	class ScmSerializer
	{
	public:
		static void Initialize(void);

		static void SetExportPath(char path[kSCM_LongNameLen]);
		static void SetExportPath(const char * path);

		static void SetResourcePath(char path[kSCM_LongNameLen]);
		static void SetResourcePath(const char * path);

		static void AddSkin(
			char name[kSCM_NameLen],
			uint8_t palette[768],
			int32_t bmpWidth,
			int32_t bmpHeight,
			uint8_t * bmpData);

		static const int32_t GetNumSkins(void);

		static void AddMesh(
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
			char groupNames[kSCM_NumPolyGroups][kSCM_NameLen]);

		static const int32_t GetNumMeshes(void);
		static const int32_t GetNumMeshTriangles(int32_t i);
		static const int32_t GetNumMeshVertices(int32_t i);

		static void AddJoint(
			char name[kSCM_NameLen],
			int32_t parent,
			int32_t children[kSCM_MaxChildJoints],
			int32_t jointGroup,
			int32_t flags,
			float_t collisionBox[24]);

		static void BuildSkeletonChildren(void);

		static const int32_t GetJointIndexFromName(const char * jointName);
		static const int32_t GetNumJoints(void);

		static void AddEvent(char name[kSCM_MediumNameLen]);

		static const int32_t GetNumEvents(void);

		static void SequenceBegin(const char * name, float_t linearVel);
		static void SequenceAddJointFrame(
			const char * jointName,
			float_t posX = 0.0, float_t posY = 0.0, float_t posZ = 0.0,
			float_t sclX = 1.0, float_t sclY = 1.0, float_t sclZ = 1.0,
			float_t rotX = 0.0, float_t rotY = 0.0, float_t rotZ = 0.0);
		static void SequenceEnd(void);
		static void SequenceAbort(void);

		static const int32_t GetNumSequences(void);
		static const int32_t GetNumFrames(void);

		static void SerializeAndExport(void);
	};
}

#endif